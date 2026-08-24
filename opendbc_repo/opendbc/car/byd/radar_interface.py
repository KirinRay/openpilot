#!/usr/bin/env python3
"""
BYD 唐DM 车内雷达 (Continental ARS4xx) — 基于 DBC (u_radar) 解码
================================================================================
★ 2026-08-22 深度简化版: 职责回归本源 = 纯解析 + 当帧如实输出 + trackId稳定 + 准确数据
  🔴 删掉所有"越俎代庖"的决策逻辑(原版抄CP还抄错的地方):
     - 消失延迟保持 (前车缺席还保持旧点 → CP该自己sticky处理)  删除
     - Main抖动容错注入 (虚拟Main用旧距离挂假点 → 起步慢元凶)  删除
     - _park_idle / 低速跳过池槽 (替CP判断低速该不该报目标)    删除
     - self.pts持久缓存 (旧点残留15帧 → CP误判前方静止车)      删除
     - yRel平滑限幅 (CP自己有|ΔyRel|断轨判断)                  删除
  ✅ 保留并做好radar_interface该做的:
     - CAN解析: 0x109主目标 + 0x380池A多目标 + 0x340方位 → 当帧真实目标
     - trackId稳定: 同一槽持续同一trackId (CP Track.update计数需要)
     - 准确数据: dRel(3-120m真实) yRel(实测方位) vRel(即时差分参考)
     - 当帧如实输出: 当帧有啥输出啥, 无就不输出(不做假前车)

【上下级接口契约】 (与 card.py 严格对应)
  L114:  RadarInterface(self.CI.CP)                    → __init__(self, CP, CP_SP=None)
  L198:  RD = self.RI.update_carrot(CS.vEgo, CS.aEgo, rcv_time, can_list)  → 4参数签名
  L249:  tracks_msg.valid = not any(RD.errors.to_dict().values())
  L250:  tracks_msg.liveTracks = RD

radard 消费 (CP selfdrive/controls/radard.py):
  L223:  ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel]}
  Track.update → cnt累积 → alive → get_lead(视觉prob>0.5 → match / low_speed_override)
  🔴 CP自己处理: sticky(消失保持), KF(速度), low_speed_override(低速选择), |ΔyRel|断轨
     radar_interface只喂当帧真实目标, 不替CP做任何决策

【DBC 解码源】 (u_radar.dbc v3补充, CANParser读取, 公式DBC处理)
  MainDist  (0x109)  主目标距离     0.5*b7-4
  AzimB7    (0x340)  方位角(128中心) 0x340.b7 (弃用347/34A: 实测恒定-0°无效)
  dRel_slot0-3 (0x380/384/388/38C) 池A槽距离 + type_TA0-3
  dRel_slot4/5 (0x390/394) 池A扩展槽(无type)
  池B (0x3C8-3CE) 禁用: 0.4244公式实证不可靠
"""
import math
import numpy as np
from typing import List, Tuple, Dict, Optional
from opendbc.can import CANParser
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.structs import RadarData

# ==================== DBC 配置 ====================
_DBC_NAME = "u_radar"                  # 合并 DBC (原外置 + v3车内补充)
CAN_BUS = 1                            # 车内雷达总线
MAX_OBJECTS = 12                       # 最大目标数 (当帧最多输出: 主目标 + 池A全24子地址中近的12个)
# 🔴 2026-08-22 学 Rick Lan 对话方式: 消失宽限期(帧). 目标消失后保持输出这么多帧,
#   期内重现有延续(UI不闪/CP能追踪), 超期仍消失才删(前车真走/雷达长无目标时清除).
#   雷达帧率~20Hz, 10帧≈0.5s. 这是"UI持续显示"与"不卡起步"的平衡点.
GONE_TIMEOUT = 10
# 消息地址
_MAIN_MSG = 0x109
_AZIM_MSGS = [0x340]  # 方位: 0x340主目标(AzimB7) + SUB帧副目标(AzimSub0-3)
# 2026-08-24: 主=0x340正前方, 副=各SUB帧b6(AzimSub, 128中心) → 主/副两套方位标准
# 🔴 2026-08-24 深挖: 池A 只读 base+1 (6槽的 +1 子地址): 0x381/385/389/38D/391/395
#   base+0 = 离散量化档(17/34/51/67, 非连续距离, 假目标源)  跳过
#   base+1 = 连续真实距离 + 方位(b6) = 真目标源  ← 唯一信任
#   base+2/3 = 大多空闲(250/255/4哨兵, 噪声)    跳过
#   DBC 信号: slot{i}b -> dRel_slot{i}b (0.4244*dat[3]+17.79), AzimSub{i} (b6-128)
#   base+1 = 连续真实距离 + 方位(b6) = 真目标源  ← 唯一信任
#   base+2/3 = 大多空闲(250/255/4哨兵, 噪声)    跳过
#   槽4/5 base+1 (0x391/395) 信号已补 DBC (dRel_slot4b/5b + AzimSub4/5)
_POOLA_ADDRS = []                              # 6个 base+1 (addr, sig)
for _i in range(6):
    _a = 0x381 + _i * 4                        # 0x381/385/389/38D/391/395
    _sig = 'dRel_slot%d%s' % (_i, 'b')
    _POOLA_ADDRS.append((_a, _sig))
_POOLA_SIG = ['dRel_slot0b', 'dRel_slot1b', 'dRel_slot2b', 'dRel_slot3b', 'dRel_slot4b', 'dRel_slot5b']
# 池B 禁用 (0.4244公式实证不可靠)
_POOLB_ADDRS = []
_POOLB_SIG = ['dRel_B3C8', 'dRel_B3C9', 'dRel_B3CA', 'dRel_B3CB', 'dRel_B3CC', 'dRel_B3CD', 'dRel_B3CE']
# CANParser 注册消息
_RADAR_MESSAGES = ([(a, 20) for a, _s in _POOLA_ADDRS] + [(m, 20) for m in _POOLB_ADDRS]
                   + [(_MAIN_MSG, 20)] + [(m, 20) for m in _AZIM_MSGS])
_TRIGGER_MSG = _MAIN_MSG


# ==================== 解析层 (CAN → slot_map) ====================
class RadarDataProcessor:
    """解析 CAN 原始帧 → slot_map (真实目标), 全部 CANParser 读取"""

    def __init__(self):
        self.rcp = CANParser(_DBC_NAME, _RADAR_MESSAGES, CAN_BUS)
        self._updated = set()

    def process_records(self, records) -> Tuple[Dict, bool]:
        """records → slot_map (当帧真实目标, 无消失延迟/无保持)"""
        try:
            vls = self.rcp.update([(0, records)])
        except Exception:
            return {}, False
        self._updated = set(vls)
        seen_radar = _TRIGGER_MSG in self._updated
        slot_map = {}

        # ---- 主目标 0x109 (只信任当帧更新, 空闲/缺席不输出) ----
        # 🔴 2026-08-22: 必须用 self._updated 判断"当帧是否真的更新" — CANParser跨帧保留
        #   未更新消息的旧值(如0x384缺席但rcp.vl[0x384]保留上帧43m) → 会输出残留假目标.
        #   只输出当帧实际更新且距离有效的消息; 未更新=该槽本帧无数据=不输出(如实际).
        if _MAIN_MSG in self.rcp.vl and _MAIN_MSG in self._updated:
            md = self.rcp.vl[_MAIN_MSG].get('MainDist', 0.0)
            if md is not None and 3.0 <= md <= 120.0:
                azim = None
                if _AZIM_MSGS and _AZIM_MSGS[0] in self._updated:
                    a = self.rcp.vl[_AZIM_MSGS[0]].get('AzimB7', 0.0)
                    azim = float(a) if a == a else None  # nan防护
                slot_map['Main'] = {'dRel': float(md), 'azim': azim, 'src': 'main'}

        # ---- 池A 只读 base+1 (6槽, 2026-08-24 深挖) ----
        # 🔴 2026-08-24 深挖: base+1(0x381/385/389/38D/391/395) = 连续真实距离+方位(b6)
        #   base+0 离散量化档(17/34/51/67, 非连续) = 假目标源, 已跳过
        #   base+2/3 大多空闲(250/255/4哨兵) = 噪声, 已跳过
        # 保留空闲码过滤: dat[3]=4 -> d=19.5m 空闲哨兵, 跳过硬凑假目标
        for i, (addr, sig) in enumerate(_POOLA_ADDRS):
            if addr not in self.rcp.vl or addr not in self._updated:
                continue  # 当帧未更新 → 该槽本帧无数据
            d = self.rcp.vl[addr].get(sig, None)
            if d is None or not (3.0 <= d <= 120.0):
                continue  # 空闲/超范围 (dat[3]=255/247 -> d≈126/122.6m 超120)
            # 🔴 空闲码过滤: dat[3]=4 -> d≈19.5m 空闲哨兵, 跳过假目标
            b3 = round((d - 17.79) / 0.4244)   # 反解 dat[3]
            if b3 in (4, 247, 255):
                continue
            slot_map[addr - 0x380] = {'dRel': float(d), 'type': 0, 'src': 'poolA'}

        # ---- 池B (禁用) ----
        for i, addr in enumerate(_POOLB_ADDRS):
            if addr not in self.rcp.vl or addr not in self._updated:
                continue
            d = self.rcp.vl[addr].get(_POOLB_SIG[i], None)
            if d is None or not (3.0 <= d <= 120.0):
                continue
            slot_map[f'B{addr:02X}'] = {'dRel': float(d), 'type': 0, 'src': 'poolB'}

        # ---- 跨槽去重 (距离±4m同源槽合并, 保留更近) ----
        main = slot_map.pop('Main', None)
        keys = sorted(slot_map.keys(), key=lambda k: slot_map[k]['dRel'])
        merged = {}
        used = set()
        for k in keys:
            if k in used:
                continue
            merged[k] = slot_map[k]
            for k2 in keys:
                if k2 in used or k2 == k:
                    continue
                if abs(slot_map[k]['dRel'] - slot_map[k2]['dRel']) < 4.0:
                    used.add(k2)
        if main is not None:
            merged['Main'] = main
        return merged, seen_radar

    def get_azimuth_pairs(self, records) -> list:
        """收集 (addr, ang) 方位角对: 0x340.b7(AzimB7, 128中心)"""
        # 2026-08-24: 主目标0x340(AzimB7) + 副目标各SUB帧(AzimSub0-3, b6方位)
        sig_by_addr = {0x340: 'AzimB7', 0x381: 'AzimSub0', 0x385: 'AzimSub1', 0x389: 'AzimSub2', 0x38D: 'AzimSub3', 0x391: 'AzimSub4', 0x395: 'AzimSub5'}
        pairs = []
        for addr in _AZIM_MSGS + [0x381, 0x385, 0x389, 0x38D, 0x391, 0x395]:  # 0x340主 + SUB副(6槽base+1)
            if addr in self.rcp.vl:
                ang = self.rcp.vl[addr].get(sig_by_addr.get(addr, 'AzimB7'), 0.0)
                ang = float(ang) if ang == ang else None
                if ang is not None:
                    pairs.append((addr, ang))
        return pairs


# ==================== 接口层 (当帧如实输出 + trackId稳定) ====================
class RadarInterface(RadarInterfaceBase):
    """BYD 唐DM 车内雷达接口 - 纯解析 + 当帧如实输出 + trackId稳定 + 准确数据
    🔴 不做消失延迟/容错/低速判断 — 那些是CP(radard)的职责, 雷达只喂当帧真实目标"""

    def __init__(self, CP, CP_SP=None):
        try:
            super().__init__(CP, CP_SP)
        except TypeError:
            super().__init__(CP)
        self._pts_cache = {}            # 当帧点 (Rick Lan 对话模式)
        self._pts_not_seen = {}         # {trackId: 消失计数} 宽限保持计数
        self._sidx_track = {}           # {sidx: trackId} 槽→track 稳定映射 (跨帧保持同一目标同id)
        self._sidx_last_drel = {}       # {sidx: 上次距离} 供vRel即时差分(兜底)
        self._sidx_last_ts = {}         # {sidx: 上次时间}
        self._next_track_id = 2         # trackId 单调递增分配 (2起, 避开Main固定trackId=1)
        self._last_radar_seen = 0.0
        self.v_ego = 0.0
        self._processor = RadarDataProcessor()
        # 🔴 2026-08-23: 第一目标(Main)持续确认显示机制
        # Main 固定 trackId=1 (恒定, 永不因消失换ID) → leadOne 持续累积 → UI 稳定显示
        self._main_track_id = 1
        self._main_last = None          # Main 最后有效点 (dRel, yRel) 用于宽限保持
        self._main_last_ts = 0.0        # Main 最后有效时刻(秒)
        self._main_hold_cnt = 0         # Main 消失保持计数

    # ---------- 数据方法 ----------
    def _estimate_velocity(self, sidx, d: float, ts: float) -> float:
        """vRel: 距离差分 (0x109/0x380 无独立速度字段, memory 2026-08-22 实证)
        🔴 2026-08-23 删 b5==3 速度帧: memory 证伪 b5=3 非相对速度帧(byte2符号位是
           滚动/抖动, byte3=14是距离高位), 不再用伪速度. 速度交 CP 读总线(视觉 vLead).
        这里 vRel 仅作距离差分参考 (粗), 不作为主目标速度依据."""
        prev_d = self._sidx_last_drel.get(sidx)
        prev_t = self._sidx_last_ts.get(sidx)
        self._sidx_last_drel[sidx] = d
        self._sidx_last_ts[sidx] = ts
        if prev_d is not None and prev_t is not None:
            dt = max(0.01, min(ts - prev_t, 0.5))
            raw = (d - prev_d) / dt
            if abs(raw) < 35.0:
                return float(np.clip(raw, -35.0, 35.0))
        return 0.0

    @staticmethod
    def _azimuth_to_yrel(offset: float, d_rel: float) -> float:
        """方位角转横向距离 (有方位就用方位, 无方位=0如实)
        🔴 2026-08-24 45°最大扫描角: 副目标b6方位每单位 = 45/128 = 0.3516°
           角度 = 偏移(b6-128) × 45/128; 横向 = 距离 × tan(角度)
           之前每单位误当1°(tan(offset)*d), 导致远处横向爆炸/旁车位错
           符号: 副目标off负=左(数学坐标), radard坐标系右正左负 → 取反
        """
        if d_rel < 0.1:
            return 0.0
        angle_deg = offset * (45.0 / 128.0)
        yrel = math.tan(math.radians(angle_deg)) * d_rel
        return float(np.clip(-yrel, -10.0, 10.0))   # 取反: 右正左负(视觉/radard)

    # ---------- 主接口 (card.py 调用) ----------
    def update_carrot(self, v_ego: float, a_ego: float, ts: float, can_list) -> Optional[RadarData]:
        """雷达更新 - Rick Lan 对话模式: 当帧解析进 _pts_cache, 合并进 self.pts(持久),
        消失宽限 GONE_TIMEOUT 保持让UI不闪/CP追踪, 超期清除不卡起步"""
        self.v_ego = float(v_ego)
        self._pts_cache = {}   # 当帧点清空 (准备装本帧新解析目标)

        if not can_list or not isinstance(can_list[0], tuple) or len(can_list[0]) < 2:
            return None
        records = can_list[0][1]
        now_s = can_list[0][0] / 1e9 if can_list[0][0] else ts

        try:
            slot_map, seen_radar = self._processor.process_records(records)
        except Exception:
            return None
        if seen_radar:
            self._last_radar_seen = now_s

        # 🔴 2026-08-23 删 b5==3 速度帧扫描 (memory 2026-08-22 证伪: 非相对速度帧)

        # 清理 vRel差分历史: 用【时间超时】而非"当帧没见"判断 → 避免雷达短暂丢帧(某帧slot_map空)
        #   就清空所有差分历史 → 下帧目标重现变"第一帧" → vRel=0 → CP vel_sane匹配失败 → UI不显示雷达
        # 🔴 2026-08-22 修复: 原逻辑 `if sidx not in slot_map: pop` 在slot_map空帧清空全部历史,
        #   vRel恒0. 改用 now_s - last_ts > 0.5s 才清(0x109~20Hz, >0.5s=真消失, 差分无意义)
        stale_keys = [s for s in self._sidx_last_ts if (now_s - self._sidx_last_ts[s]) > 0.5]
        for sidx in stale_keys:
            self._sidx_last_drel.pop(sidx, None)
            self._sidx_last_ts.pop(sidx, None)

        # 当帧真实目标排序: Main优先 → 池A(按距离)
        slot_keys = []
        if 'Main' in slot_map:
            slot_keys.append('Main')
        pool_keys = sorted([k for k in slot_map if k != 'Main'], key=lambda k: slot_map[k]['dRel'])
        slot_keys += pool_keys[:max(0, MAX_OBJECTS - 1)]

        # 方位角分配 (2026-08-24: 主/副两套标准)
        #   主目标(Main) = 0x340 AzimB7 (正前方)
        #   副目标(池槽) = 各自 SUB 帧 AzimSub (b6方位, sidx=addr-0x380 精确匹配)
        azim_by_sidx = {}
        az_pairs = self._processor.get_azimuth_pairs(records) if records else []
        # 主目标方位: 0x340
        main_az = next((a for _addr, a in az_pairs if _addr == 0x340), None)
        if main_az is not None and 'Main' in slot_keys:
            azim_by_sidx['Main'] = main_az
        # 副目标方位: 各 SUB 帧 AzimSub, 按 sidx(addr-0x380) 精确匹配
        azim_by_i = {addr - 0x380: ang for addr, ang in az_pairs if addr != 0x340}
        for k in pool_keys:
            # k 是整数槽索引 (addr-0x380) → 直接匹配
            if isinstance(k, int) and k in azim_by_i:
                azim_by_sidx[k] = azim_by_i[k]
            # 非整数sid或无法匹配: 用0x340(正前方)保守, 不臆造

        # 构建当帧输出 (Rick Lan 对话模式: 当帧点进 _pts_cache)
        # 🔴 2026-08-22 学 Rick Lan 对话方式: 目标消失后不立即删, 宽限保持让UI不闪/CP能追踪
        track_count = 0
        self._pts_cache = {}
        for sidx in slot_keys:
            if track_count >= MAX_OBJECTS:
                break
            sm = slot_map[sidx]
            d = sm.get('dRel')
            if d is None or d < 0.5:
                continue

            # trackId 稳定分配
            # 🔴 2026-08-23: Main(第一目标)固定 trackId=_main_track_id=1, 永不换;
            #   池槽(辅助目标)才用 _next_track_id 单调递增分配.
            if sidx == 'Main':
                track_id = self._main_track_id
            else:
                if sidx not in self._sidx_track:
                    while self._next_track_id in self._sidx_track.values() or self._next_track_id == self._main_track_id:
                        self._next_track_id += 1
                    self._sidx_track[sidx] = self._next_track_id
                    self._next_track_id += 1
                track_id = self._sidx_track[sidx]

            pt = RadarData.RadarPoint()
            pt.trackId = track_id
            pt.dRel = float(d)

            # yRel: 有方位用方位, 无方位=0如实 (不臆造横向)
            matched_ang = azim_by_sidx.get(sidx)
            azim = sm.get('azim')
            if matched_ang is not None:
                pt.yRel = self._azimuth_to_yrel(matched_ang, d)
            elif azim is not None:
                pt.yRel = self._azimuth_to_yrel(azim, d)
            else:
                pt.yRel = 0.0

            pt.vRel = self._estimate_velocity(sidx, d, ts)
            # 🔴 2026-08-22 对齐桌面版(Rick Lan)字段: 每个多目标都输出完整字段,
            #   CP radard 靠这些做 track 更新/cnt累积/lead选择, 缺字段会 UI 不显示/追踪断
            for _f, _v in (('measured', True), ('vLead', v_ego + pt.vRel),
                           ('aLead', 0.0), ('aRel', float('nan')), ('yvRel', 0.0)):
                try:
                    setattr(pt, _f, _v)
                except Exception:
                    pass

            self._pts_cache[track_id] = pt
            track_count += 1

            # 🔴 2026-08-23: 记录 Main 最后有效点 (供消失时宽限保持)
            if sidx == 'Main':
                self._main_last = (float(d), float(pt.yRel))
                self._main_last_ts = now_s
                self._main_hold_cnt = 0

        # 🔴 2026-08-23: Main 宽限保持 — 第一目标必须"一直确认一直显示".
        #   当帧 Main 不在 slot_map (0x109 b7 抖动/短时丢失), 但之前出现过且未超时:
        #   继续输出最后有效 Main 点 (trackId 恒定1, dRel/yRel 用最后值),
        #   保证 CP 的 leadOne track 持续 alive/累积, UI 稳定显示第一目标.
        if 'Main' not in slot_map and self._main_last is not None:
            if (now_s - self._main_last_ts) < GONE_TIMEOUT * 0.05 and self._main_hold_cnt < GONE_TIMEOUT:
                self._main_hold_cnt += 1
                pt = RadarData.RadarPoint()
                pt.trackId = self._main_track_id
                pt.dRel = self._main_last[0]
                pt.yRel = self._main_last[1]
                pt.vRel = 0.0
                for _f, _v in (('measured', True), ('vLead', self.v_ego),
                               ('aLead', 0.0), ('aRel', float('nan')), ('yvRel', 0.0)):
                    try:
                        setattr(pt, _f, _v)
                    except Exception:
                        pass
                self._pts_cache[self._main_track_id] = pt
            else:
                # 超时: Main 真消失, 清除保持状态 (下次重现当新目标)
                self._main_last = None
                self._main_hold_cnt = 0

        # 🔴 2026-08-22 学 Rick Lan 对话方式: _pts_cache(当帧) 合并进 self.pts(持久)
        #   消失宽限 GONE_TIMEOUT: 目标消失后保持输出, 期内重现有延续(UI不闪/CP追踪),
        #   超期仍消失才删(前车真走/雷达长无目标时清除, 不产生假点卡起步).
        #   这是"UI持续显示"与"不卡起步"的平衡: 宽限期内闪烁保持, 超限实事求是清除.
        for _tid in list(self.pts.keys()):
            if _tid in self._pts_cache:
                self._pts_not_seen[_tid] = GONE_TIMEOUT   # 重见: 重置宽限
            else:
                self._pts_not_seen[_tid] = self._pts_not_seen.get(_tid, GONE_TIMEOUT) - 1
                if self._pts_not_seen[_tid] <= 0:
                    del self.pts[_tid]                    # 超宽限: 真正删除
                    self._pts_not_seen.pop(_tid, None)
                    # 🔴 2026-08-23: 目标真删除时清理 trackId 映射 (避免 ID 复用混乱)
                    for _s, _t in list(self._sidx_track.items()):
                        if _t == _tid:
                            self._sidx_track.pop(_s, None)
        self.pts.update(self._pts_cache)

        ret = RadarData()
        if (now_s - self._last_radar_seen) > 2.0:
            ret.errors.canError = True
        ret.points = list(self.pts.values())
        return ret

    def reset(self) -> None:
        """重置雷达接口"""
        self.pts = {}
        self._pts_cache = {}
        self._pts_not_seen = {}
        self._sidx_track.clear()
        self._sidx_last_drel.clear()
        self._sidx_last_ts.clear()
        self._next_track_id = 2
        self._last_radar_seen = 0.0
        self._main_last = None
        self._main_last_ts = 0.0
        self._main_hold_cnt = 0


# ==================== 冒烟测试 ====================
if __name__ == "__main__":
    from opendbc.car.structs import CarParams
    inst = RadarInterface(CarParams())
    # 空闲帧: 应无目标
    idle = [(0x109, bytes([0, 0, 0, 0, 0, 0, 0, 255]), 1),
            (0x380, bytes([0, 0, 0, 255, 0, 0, 0, 255]), 1)]
    rd = inst.update_carrot(0.0, 0.0, 1.0, [(int(1e9), idle)])
    print(f"空闲帧目标数: {len(list(rd.points))} (应0)")
    # 真目标帧: 0x109主(46m) + 池A 0x380(39m) + 0x384(60m)
    real = [(0x109, bytes([0, 0, 0, 0, 0, 0, 0, 100]), 1),
            (0x380, bytes([0, 0, 0, 50, 0, 0, 0, 7]), 1),
            (0x384, bytes([0, 0, 0, 60, 0, 0, 0, 9]), 1)]
    for i in range(3):
        rd = inst.update_carrot(5.0, 0.0, i + 2.0, [(int(1e9), real)])
    pts = list(rd.points)
    print(f"真目标帧目标数: {len(pts)} (应3)")
    for p in pts:
        print(f"  track{p.trackId}: dRel={p.dRel:.1f}m yRel={p.yRel:+.2f} vRel={p.vRel:+.1f}")
    # 前车消失: 前 GONE_TIMEOUT 帧应保持输出(宽限, UI不闪), 超宽限才清除(不卡起步)
    rd = inst.update_carrot(5.0, 0.0, 5.0, [(int(1e9), idle)])
    print(f"前车消失第1帧: {len(list(rd.points))}点 (应3, 宽限保持中)")
    for _ in range(GONE_TIMEOUT):
        rd = inst.update_carrot(5.0, 0.0, 6.0, [(int(1e9), idle)])
    print(f"前车消失超宽限后: {len(list(rd.points))}点 (应0, 已清除不卡起步)")
    print("冒烟测试完成")
