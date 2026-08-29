import numpy as np
from opendbc.car import structs
from opendbc.car.byd.values import  CanBus, CarControllerParams

GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def byd_checksum(byte_key, dat):
    first_bytes_sum = sum(byte >> 4 for byte in dat)
    second_bytes_sum = sum(byte & 0xF for byte in dat)
    remainder = second_bytes_sum >> 4
    second_bytes_sum += byte_key >> 4
    first_bytes_sum += byte_key & 0xF
    first_part = ((-first_bytes_sum + 0x9) & 0xF)
    second_part = ((-second_bytes_sum + 0x9) & 0xF)
    return (((first_part + (-remainder + 5)) << 4) + second_part) & 0xFF

# MPC -> Panda -> EPS
def create_steering_control(packer, CP, cam_msg: dict, req_torque, req_prepare, active, hud_control, counter):
    values = {}
    values = {s: cam_msg[s] for s in [
        "AutoFullBeamState",
        "LeftLaneState",
        "LKAS_Config",
        "SETME2_0x1",
        "MPC_State",
        "AutoFullBeam_OnOff",
        "LKAS_Output",
        "LKAS_Active",
        "SETME3_0x0",
        "TrafficSignRecognition_OnOff",
        "SETME4_0x0",
        "SETME5_0x1",
        "RightLaneState",
        "LKAS_State",
        "TrafficSignRecognition_Result",
        "LKAS_AlarmType",
        "SETME7_0x3",
    ]}

    values["ReqHandsOnSteeringWheel"] = 0
    values["LKAS_ReqPrepare"] = req_prepare
    values["Counter"] = counter

    if active:
        mpc_state = values["MPC_State"] #2: Cancelling lkas control
        values.update({
            "LKAS_Output" : req_torque,
            "LKAS_Active" : 1,
            "LKAS_State" : 4 if (mpc_state == 2) else 2,
            "LeftLaneState":  3 if hud_control.leftLaneDepart  else int(hud_control.leftLaneVisible) + 1,
            "RightLaneState": 3 if hud_control.rightLaneDepart else int(hud_control.rightLaneVisible) + 1,
        })
    else: # Note: This disables the stock AEB feature: turn steering wheel while close impacting obstacles in front.
        values.update({
            "LKAS_Output" : 0,
            "LKAS_Active" : 0,
        })

    data = packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)

# op long control
def acc_cmd(packer, CP, cam_msg: dict, mrr_leaddist, accel, rfss, sss, longActive):
    values = {}

    values = {s: cam_msg[s] for s in [
        "AccelCmd",
        "ComfortBandUpper",
        "ComfortBandLower",
        "JerkUpperLimit",
        "SETME1_0x1",
        "JerkLowerLimit",
        "ResumeFromStandstill",
        "StandstillState",
        "BrakeBehaviour",
        "AccReqNotStandstill",
        "AccControlActive",
        "AccOverrideOrStandstill",
        "EspBehaviour",
        "Counter",
        "SETME2_0xF",
    ]}

    jerk_base_upper = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_upper_fp)
    jerk_base_lower = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_lower_fp)

    if (accel < 0): #use lower factor
        jerk_upper = jerk_base_upper
        jerk_lower = jerk_base_lower + accel * CarControllerParams.K_accel_jerk_lower
    else:
        jerk_upper = jerk_base_upper + accel * CarControllerParams.K_accel_jerk_upper
        jerk_lower = jerk_base_lower

    # ACC 激活标志 — 对齐原车黄金帧(AccControlActive=1, AccReqNotStandstill=1) + 现代 SCC12.ACCMode
    # 只有长期激活(longActive)时才激活原车 ACC 执行, 否则保持 0 (不让原车 ACC 动刹车/油门)
    values["AccControlActive"] = 1 if longActive else 0
    values["AccReqNotStandstill"] = 1 if longActive else 0

    if longActive and mrr_leaddist > 3:  # 增加最小距离检测
        values.update({
            "AccelCmd" : accel,
            "ComfortBandUpper" : 0.05 if mrr_leaddist > 50 else 0.10,
            "ComfortBandLower" : 0.05 if mrr_leaddist > 50 else 0.10,
            "JerkUpperLimit" : jerk_upper,
            "JerkLowerLimit" : jerk_lower,
            "ResumeFromStandstill" : rfss,
            "StandstillState" : sss,
        })

    data = packer.make_can_msg("ACC_CMD", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_CMD", CanBus.ESC, values)


# send fake torque feedback from eps to trick MPC, preventing DTC, so that safety features such as AEB still working
def create_fake_318(packer, CP, esc_msg: dict, faketorque, laks_reqprepare, laks_active , enabled, counter):
    values = {}

    values = {s: esc_msg[s] for s in [
        "LKAS_Prepared",
        "CruiseActivated",
        "TorqueFailed",
        "SETME1_0x1",
        "SteerWarning",
        "SteerErrorCode",
        "MainTorque",
        "SETME3_0x1",
        "SETME4_0x3",
        "SteerDriverTorque",
        "SETME5_0xFF",
        "SETME6_0xFFF",
    ]}

    values["ReportHandsNotOnSteeringWheel"] = 0
    values["Counter"] = counter

    if enabled:
        # Restore the intended fake-torque injection (was fully commented out = dead code).
        # FIXME(needs real-vehicle validation): injecting LKAS_Prepared/CruiseActivated/MainTorque
        # back to the stock EPS prevents OP直连 from tripping DTC and keeps AEB alive, but the
        # exact values must be verified against a real capture so we don't break the EPS handshake.
        if laks_active:
            values.update({
                "LKAS_Prepared" : 1,
                "CruiseActivated" : 1,
                "MainTorque" : int(faketorque),
            })
        elif laks_reqprepare:
            values.update({
                "LKAS_Prepared" : 1,
                "CruiseActivated" : 0,
                "MainTorque" : 0,
            })
        else:
            values.update({
                "LKAS_Prepared" : 0,
                "CruiseActivated" : 0,
                "MainTorque" : 0,
            })


    data = packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)


# ============================================================================
# ACC_HUD_ADAS (0x32D) — 原车 ACC 视觉状态广播帧
# ============================================================================
# CP 替代原车视觉后，openpilot 纵向接管时必须自己发 ACC_HUD_ADAS 到 ESC bus，
# 诱骗原车 ACC 控制单元认为"ACC 已装备、链路正常"。
#
# 参考 CP 作者使用的现代 (Hyundai) 方法 (carcontroller.py):
#   - create_acc_opt (SCC13):   frame%20==0 且 openpilotLongitudinalControl 时发
#                               SCC_Equip=1 告诉原车"ACC 已装备"
#   - create_frt_radar_opt:     frame%50==0 时发 CF_FCA_Equip_Front_Radar=1
#                              告诉原车"前置雷达已装备"
#   - make_tester_present:      frame%100==0 时发 0x7d0 禁用原厂雷达/ADAS ECU
#
# BYD 对应: ACC_HUD_ADAS 广播 "ACC 状态/装备/前车/HUD" 给原车，
#           AccState 必须为健康值(0/3)，绝不广播 7(ERROR) 否则原车报"雷达错误"。
#
# 信号 (byd_han_dmev_2020.dbc BO_ 813): SetSpeed 0|9, HasLead 9|1,
#   SetDistance 10|3, LeadingDistance 13|3, AEB 16|1, FCW 17|1, SETME1 18|1,
#   AccState 19|3 (0=OFF 2=ON 3=ACTIVE 5=FORCE 7=ERROR), AccOn1 22|1,
#   CloseWarning 23|1, SETME2 24|1, Notify 25|7, Status 32|4,
#   SETME3 36|12, Counter 48|4, SETME4 55|4, CheckSum 56|8
def create_hud_adas(packer, CP, cam_hud: dict, CS, CC, longActive, counter):
    values = {}

    # 继承原车摄像头能读到的信号（保留大部分原车状态，只覆盖 ACC 健康状态）
    # 参考丰田 create_ui_command "if len(stock_lkas_hud): update(collected)" 模式
    if cam_hud is not None and len(cam_hud) > 0:
        values = {s: cam_hud[s] for s in [
            "SetSpeed",
            "HasLead",
            "SetDistance",
            "LeadingDistance",
            "AEB",
            "FCW",
            "SETME1_0x1",
            "AccOn1",
            "CloseWarning",
            "SETME2_0x1",
            "Notify",
            "Status",
            "SETME3_0xFFF",
            "SETME4_0xF",
        ]}
    else:
        # 无原车摄像头消息时用安全默认值（永不广播 ERROR）
        values = {
            "SetSpeed": 0,
            "HasLead": 0,
            "SetDistance": 0,
            "LeadingDistance": 0,
            "AEB": 0,
            "FCW": 0,
            "SETME1_0x1": 1,
            "AccOn1": 0,
            "CloseWarning": 0,
            "SETME2_0x1": 1,
            "Notify": 0,          # 0 = NONE（不广播 ACC_ERROR）
            "Status": 0,
            "SETME3_0xFFF": 4095,
            "SETME4_0xF": 3,
        }

    # 覆盖为健康状态（openpilot 接管视觉后，让原车 ACC 认为链路正常）
    # AccState 状态机(对齐路试黄金帧): 3=ACTIVE(ENGAGED) / 1=READY(主开关开未engage) / 0=OFF
    # 绝不广播 7(ERROR) — 对齐黄金 ENGAGED=3 READY=1
    acc_active = bool(longActive and (CC.enabled if CC is not None else False))
    main_sw = getattr(CS, 'lkas_isMainSwOn', None) if CS is not None else None
    if acc_active:
        values["AccState"] = 3   # ACC_ACTIVE (ENGAGED) — 对齐黄金
    elif main_sw:
        values["AccState"] = 1   # ACC_READY (主开关开, 未engage) — 对齐黄金 READY
    else:
        values["AccState"] = 0   # OFF
    # 设定速度（来自 carState），0.5 scale
    set_speed_kph = 0.0
    if CS is not None and CS.out.cruiseState.speed > 0:
        set_speed_kph = CS.out.cruiseState.speed * 3.6  # m/s -> kph
    values["SetSpeed"] = float(set_speed_kph) / 0.5
    # SetDistance: 跟车距离档, 对齐黄金 ENGAGED=3
    values["SetDistance"] = 3
    # Status: ACC 状态显示, 对齐黄金 ENGAGED=4
    values["Status"] = 4 if acc_active else 3
    # 有前车：唐DM 用雷达 mrr_leading_dist (<200 = 有前车)
    mrr_dist = getattr(CS, 'mrr_leading_dist', 199) if CS is not None else 199
    has_lead = 1 if (mrr_dist < 200) else 0
    values["HasLead"] = has_lead
    if has_lead:
        # LeadingDistance 粗略分挡
        values["LeadingDistance"] = 0 if mrr_dist > 80 else (1 if mrr_dist > 40 else 2)
    # HUD 通知: 对齐黄金 — ENGAGED 时 0(NONE), READY(主开关on未engage) 时 8, OFF 时 0
    if acc_active:
        values["Notify"] = 0    # ENGAGE 时黄金=0
    elif main_sw:
        values["Notify"] = 8    # READY 预备, 对齐黄金
    else:
        values["Notify"] = 0    # NONE
    # AccOn1 = 原车 ACC 主开关状态 (PCM_BUTTONS.BTN_TOGGLE_ACC_OnOff)
    # 关键: cruiseState.available = lkas_isMainSwOn and lkas_config_isAccOn and AccOn1
    # 若 AccOn1 只在激活(AccState!=0)时=1, 会导致 available=False → wrongCarMode → 无法engage (死锁)
    # 所以 AccOn1 必须跟随原车主开关, 用户按ACC主开关后即可engage
    main_sw = getattr(CS, 'lkas_isMainSwOn', None) if CS is not None else None
    if main_sw is not None:
        values["AccOn1"] = 1 if main_sw else 0
    else:
        # 无 CS 时保持 AccState 关联（兼容测试桩）
        values["AccOn1"] = 1 if (values["AccState"] != 0) else int(values.get("AccOn1", 0))
    values["Counter"] = counter

    data = packer.make_can_msg("ACC_HUD_ADAS", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_HUD_ADAS", CanBus.ESC, values)

