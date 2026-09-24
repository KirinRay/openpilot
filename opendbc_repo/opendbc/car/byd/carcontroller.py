import numpy as np
import time
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_driver_steer_torque_limits, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.byd.tuning import Tuning

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)

    self.packer = CANPacker(dbc_names[Bus.pt])
    from cereal import messaging
    self.sm = messaging.SubMaster(['radarState', 'modelV2', 'longitudinalPlan']) 
    self.frame = 0
    self.last_steer_frame = 0
    self.last_acc_frame = 0

    self.apply_torque_last = 0

    self.mpc_lkas_counter = 0
    self.mpc_acc_counter = 0
    self.eps_fake318_counter = 0
    self.pcm_button_counter = 0

    self.lkas_req_prepare = 0
    self.lkas_active = 0
    self.lat_safeoff = 0
    # HANDSOFF (hands-off EPS protection): track sustained hands-off + wheel motion
    self.handsoff_angle_cnt = 0
    self.handsoff_last_exit = 0.0

    self.steer_softstart_limit = 0
    self.steerRateLimActive = False
    self.steerRateLim = 1.0

    # V9横向100%学习 (2026-09-02): 启停/等红灯保护
    self.lat_inactive_frames = 0   # 启停/等红灯计时 (等红灯LKAS故障: 超15000帧清零重启握手)
    self.soft_start_torque_limit = 0

    self.first_start = True
    self.rfss = 0 # resume from stand still
    self.sss = 0 #stand still state

    self.apply_accel_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    # 横向控制部分 - 保持原有逻辑
    if (self.frame - self.last_steer_frame) >= CarControllerParams.STEER_STEP:
      if self.first_start:
        self.mpc_lkas_counter = int(CS.acc_mpc_state_counter + 1) & 0xF
        self.mpc_acc_counter = int(CS.acc_cmd_counter + 1) & 0xF
        self.eps_fake318_counter = int(CS.eps_state_counter + 1) & 0xF
        self.first_start = False

      apply_torque = 0

      if CC.latActive:
        # 起步恢复：重置启停计时器
        self.lat_inactive_frames = 0
        # HANDSOFF 拟人闪断 (2026-09-04 08:48 用户定: 改回 REF need_steer 判据, 撤销 CP11 偏摆判据回归):
        #   - 弯道/有真实转向需求(OP 期望扭矩大): 方向机在正常工作有持续力矩, 不会因离手报错 -> 不闪断
        #   - 直线/无转向需求(OP 期望扭矩≈0, 仅直行稳住): 离手久了车机15秒安全会退出 -> 按时间周期闪断重置
        # 判据 = CC.actuators.torque (OP 是否有真实转向需求), 不用方向盘偏摆角 (>4° 偏摆闪断是错的/不安全,
        #         直线<4°不闪断→车机15秒退出ACC/弯道修正频繁闪断→扰动ACC状态, 影响纵向/灯闪, 已确认改回)。
        # 触发前提: ACC有效(lkas_active)+行驶有速度(vEgo>0.5m/s)+离手+直线无转向需求。
        now_s = time.time()
        is_handsoff = abs(CS.out.steeringTorque) < 10.0        # 驾驶员离手(没施加力矩)
        need_steer  = abs(CC.actuators.torque) > 0.06          # OP 有真实转向需求(弯道/纠偏)->不闪断
        if self.lkas_active and is_handsoff and not need_steer and CS.out.vEgo > 0.5:
            self.handsoff_angle_cnt += 1
            # 持续(行驶+离手+直线无转向)够 HANDSOFF_PERIOD[0] -> 周期性闪断(lat_safeoff 归零重启ACC), 规避车机15秒离手退出ACC
            if self.handsoff_angle_cnt >= Tuning.HANDSOFF_PERIOD[0] * CarControllerParams.STEER_STEP * 10:
                if now_s - self.handsoff_last_exit > Tuning.HANDSOFF_PERIOD[0]:
                    self.lat_safeoff = 1
                    self.handsoff_last_exit = now_s
                    self.handsoff_angle_cnt = 0
        else:
            self.handsoff_angle_cnt = 0

        if self.lkas_active:
          steer_desire = CC.actuators.torque

          if CarControllerParams.USE_STEERING_SPEED_LIMITER:
            # rate limit based on vehicle SPEED (m/s), not acceleration
            rate_limit = np.interp(CS.out.vEgo, [8.3, 27.8], [132, 64])
            delta_rate = CS.steeringRateDegAbs - rate_limit

            if delta_rate < 0:
              self.steerRateLim -= 0.005 * delta_rate
              if delta_rate < -0.05:
                self.steerRateLimActive = False
              if self.steerRateLim > 1.0:
                self.steerRateLim = 1.0
                self.steerRateLimActive = False
            else:
              if self.steerRateLimActive:
                self.steerRateLim -= 0.005 * delta_rate
              else:
                self.steerRateLim = steer_desire
                self.steerRateLimActive = True
              if self.steerRateLim < 0:
                self.steerRateLim = 0

            new_steer_pu = np.clip(steer_desire, -self.steerRateLim, self.steerRateLim)
          else:
            new_steer_pu = steer_desire

          new_steer = int(round(new_steer_pu * CarControllerParams.STEER_MAX))

          if self.steer_softstart_limit < CarControllerParams.STEER_MAX:
            self.steer_softstart_limit = self.steer_softstart_limit + CarControllerParams.STEER_SOFTSTART_STEP
            new_steer = np.clip(new_steer, -self.steer_softstart_limit, self.steer_softstart_limit)

          # V9横向100%学习: 防止OP/LKAS与驾驶员反向抢盘触发EPS TorqueFailed(报 lkas / steerUnavailable)。
          # 现象:过匝道时OP仍在输出横向扭矩,驾驶员强行打方向,两股相反扭矩持续对拉,
          #       EPS被顶到TorqueFailed永久故障(需重启车辆才能恢复)。
          #       OP与驾驶员对抗时, 驾驶员扭矩与命令扭矩方向相反且|驾驶员扭矩|超过允许量时,
          #       视为驾驶员强制接管, 立即将命令归零, 交由apply_driver_steer_torque_limits按速率
          #       限制平滑回零(STEER_DELTA_DOWN), EPS不再被反向顶。同向(驾驶员帮着打)时不干预。
          driver_torque = CS.out.steeringTorque
          if (new_steer != 0 and driver_torque != 0 and
              (new_steer > 0) != (driver_torque > 0) and
              abs(driver_torque) > CarControllerParams.STEER_DRIVER_ALLOWANCE):
            new_steer = 0

          # (2026-09-02 16:4x 用户定: 去除 V9 LOW_SPEED 停车扭矩限幅 -
          #  与 HANDSOFF 闪断冲突: 低速挪车方向盘>4° 且离手时会触发HANDSOFF频繁闪断, 该限幅多余+干扰)

          apply_torque = apply_driver_steer_torque_limits(new_steer, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams)
        else:
          # 官方握手 (BYD0831, 用户 09-01 18:23 定): 等原车ESC回 LKAS_Prepared=1 才激活, 否则先发预备请求
          # (黄金版路试铁证 00000002 seg3-5: 原车ESC 0x318 LKAS_Prepared 全程=1, 握手成立;
          #  OP 0x316 ReqPrepare=1 占8518帧主导 -> 必须先发预备请求等回执, 不能直接激活)
          # (rtA 18:10 seg4/seg5: 真机 ESC 回 LKAS_Prepared=181/230帧 -> 握手可行)
          # 唐DM退避: 已激活但 EPS 撤 prepared(未就绪/已TemporaryFail) → 立即清零,
          # 避免持续驱动未就绪的 EPS 触发 TorqueFailed 保护 (V9 横向正确逻辑)
          if self.lkas_active and not CS.lkas_prepared:
            self.lkas_active = 0.0
          if CS.lkas_prepared:
            self.lkas_active = 1.0
            self.steerRateLimActive = False
            self.steerRateLim = 1.0
            self.lkas_req_prepare = 0
            self.steer_softstart_limit = 0
            self.lat_safeoff = 1
          else:
            self.lkas_req_prepare = 1

      elif self.lat_safeoff:
        if self.apply_torque_last == 0:
          self.lat_safeoff = 0
        apply_torque = apply_driver_steer_torque_limits(0, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams)
      else:
        self.lkas_req_prepare = 0
        self.steerRateLimActive = False
        self.steerRateLim = 1.0
        # V9横向100%学习: 启停场景(CC.latActive因standstill短暂为False)保留lkas_active和
        # steer_softstart_limit, 起步时直接输出扭矩, 避免softstart从0爬升导致前几百毫秒方向盘没力、横向丢失。
        # 时间窗保护: 停车超过 15000 帧(约5分钟@50Hz)后强制清零, 覆盖几乎所有红绿灯场景。
        #
        # EPS TorqueFailed 永久故障保护(v9-BYD seg52复现修复):
        # 根因: BYD EPS 在 CruiseActivated=1 + vEgo<0.3 + OP发lkas_active=1+LKAS_Output=0
        #       的矛盾状态下, 给40ms宽限期后置位TorqueFailed永久故障(需重启车)。
        # 修复: 监测EPS反馈的激活状态, 仅在 EPS仍激活+CruiseAct=1+vEgo<0.3 时立即清零lkas_active释放EPS。
        #       正常stop-and-go中EPS已撤激活, 不触发清零, lkas_active保留, 起步瞬时激活。
        # 唐DM: 无CruiseActivated字段(新DBC用LKAS_State枚举), 用 lkas_state==2(Active) 判断EPS激活。
        self.lat_inactive_frames += 1
        if getattr(CS, 'is_tang_dm', False):
          cruise_activated = CS.lkas_state == 2  # LKAS_State Active
        else:
          cruise_activated = bool(CS.esc_eps.get('CruiseActivated', 0)) if CS.esc_eps else False
        if (CS.out.vEgo <= 0.3 and cruise_activated) or CS.out.vEgo > 0.3 or self.lat_inactive_frames > 15000:
          self.lkas_active = 0
          self.steer_softstart_limit = 0
          self.lat_inactive_frames = 0
        # 注意: 此处不强制同步mpc_laks_active。
        # 原因: else分支覆盖"latActive=False且非safeoff"场景(行驶中横向未激活/低速过渡/握手失败)。
        # 若强制lkas_active=1, OP未控车却发CruiseActivated=1, EPS检测双源冲突→高频steerFaultTemporary
        # →controlsd高频处理+UI高频刷新告警→CPU占满→画面卡死(别人车行驶中偶发)。
        # 等红灯LKAS故障问题(standstill场景)由lat_inactive_frames>15000超时清零+重启握手解决。
        self.soft_start_torque_limit = 0

      self.apply_torque_last = apply_torque

      self.mpc_lkas_counter = int(self.mpc_lkas_counter + 1) & 0xF
      self.eps_fake318_counter = int(self.eps_fake318_counter + 1) & 0xF
      self.last_steer_frame = self.frame

      can_sends.append(bydcan.create_steering_control(self.packer, self.CP, CS.cam_lkas,
          self.apply_torque_last, self.lkas_req_prepare, self.lkas_active, CC.hudControl, self.mpc_lkas_counter))

      can_sends.append(bydcan.create_fake_318(self.packer, self.CP, CS.esc_eps,
                                              CS.mpc_laks_output, CS.mpc_laks_reqprepare, self.lkas_active,
                                              True, self.eps_fake318_counter))

    # 纵向控制部分 - 信任MPC输出，只做安全限制
    if (self.frame + 1 - self.last_acc_frame) >= CarControllerParams.ACC_STEP:
      # 更新雷达数据
      self.sm.update(0)
      
      mpc_target_accel = CC.actuators.accel

      if CC.longActive:
        stopping = CC.actuators.longControlState == LongCtrlState.stopping
        starting = CC.actuators.longControlState == LongCtrlState.starting
        running = CC.actuators.longControlState == LongCtrlState.pid

        # 获取基本数据用于日志记录（不用于控制逻辑）
        lead_distance = getattr(CS, 'mrr_leading_dist', 199)
        v_ego = CS.out.vEgo

        # 获取雷达融合数据用于日志
        lead_speed = 0.0
        relative_speed = 0.0
        fusion_distance = 199
        data_source = "no_radar"

        if hasattr(self, 'sm') and self.sm.alive['radarState']:
            lead_one = self.sm['radarState'].leadOne
            if lead_one.status:
                lead_speed = lead_one.vLead if not math.isnan(lead_one.vLead) else 0.0
                relative_speed = lead_one.vRel if not math.isnan(lead_one.vRel) else 0.0
                fusion_distance = lead_one.dRel
                data_source = "radar"
            else:
                data_source = "no_lead"

        # 车辆特定的安全限制和平滑处理
        # 信任MPC的计算，只对极端情况进行安全限制
        if mpc_target_accel < 0:
            # 基于融合数据的动态制动缩放
            if fusion_distance < 199:
                # 距离因子：针对快速接近场景优化
                if relative_speed < -2.0 and fusion_distance < v_ego * 1.5:
                    # 快速接近时，增强制动响应
                    distance_factor = 1.0  # 不缩放制动
                    speed_factor = 1.2     # 增强制动
                else:
                    # 正常情况的缩放
                    distance_factor = np.interp(fusion_distance, [5.0, 30.0], [0.8, 0.4])
                    if relative_speed < -1.0:
                        speed_factor = 1.0
                    elif relative_speed < 0:
                        speed_factor = 0.7
                    else:
                        speed_factor = 0.5

                # 速度因子：相对速度越大（接近前车），制动缩放越大
                if relative_speed < -1.0:  # 快速接近前车
                    speed_factor = 1.0
                elif relative_speed < 0:   # 缓慢接近前车
                    speed_factor = 0.7
                else:                      # 远离前车或速度匹配
                    speed_factor = 0.5

                # 综合缩放因子
                brake_scale = distance_factor * speed_factor
                brake_scale = np.clip(brake_scale, 0.3, 0.8)
            else:
                # 无前车时大幅减少制动
                brake_scale = 0.3

            scaled_accel = mpc_target_accel * brake_scale
        else:
            # 加速指令直接使用
            scaled_accel = mpc_target_accel

        # 平滑处理 - 防止加速度突变
        if hasattr(self, 'last_final_accel'):
            # 检测MPC的极端跳跃
            if hasattr(self, 'last_mpc_accel'):
                mpc_change = abs(mpc_target_accel - self.last_mpc_accel)
                accel_change_limit = 0.1 if mpc_change > 2.0 else 0.2
            else:
                accel_change_limit = 0.25

            accel_diff = scaled_accel - self.last_final_accel
            if abs(accel_diff) > accel_change_limit:
                scaled_accel = self.last_final_accel + np.sign(accel_diff) * accel_change_limit

        self.last_mpc_accel = mpc_target_accel
        final_accel = np.clip(scaled_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
        self.last_final_accel = final_accel

        # 停车状态逻辑
        if stopping and final_accel < -0.1:
          self.rfss = 0
          self.sss = CS.out.standstill
        elif starting and final_accel > 0.1 and CS.out.vEgo < 0.8:
          self.rfss = CS.out.standstill
          self.sss = 0
        elif running:
          self.rfss = 0
          self.sss = 0
      else:
        final_accel = 0
        scaled_accel = 0
        lead_speed = 0.0
        relative_speed = 0.0
        lead_distance = 199
        fusion_distance = 199
        data_source = "no_lead"
        self.sss = 0
        self.rfss = 0

      self.mpc_acc_counter = int(self.mpc_acc_counter + 1) & 0xF

      # 发送控制命令
      can_sends.append(bydcan.acc_cmd(self.packer, self.CP, CS.cam_acc,
                                     getattr(CS, 'mrr_leading_dist', 199),
                                     final_accel, self.rfss, self.sss, CC.longActive,))

      self.apply_accel_last = final_accel
      self.last_acc_frame = self.frame + 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = float(self.apply_accel_last)
    new_actuators.steeringAngleDeg = float(CS.out.steeringAngleDeg)

    self.frame += 1
    return new_actuators, can_sends
