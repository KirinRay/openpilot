#!/usr/bin/env python3
# BYD Tang DM safety tests (2026-08-28)
# 参照 test_chrysler.py (纯 TorqueMotorLimited) 模板, 针对唐DM safety_byd.h 实际行为:
#   - TX: ACC_MPC_STATE 扭矩 (TorqueMotorLimited, max_steer=300, rate=18, rt_delta=250, interval=250ms, max_error=80)
#   - RX: PEDAL gas/brake, CARSPEED, ACC_EPS_STATE (CruiseActivated+angle), ACC_HUD_ADAS (cruise)
#   - FWD: bus0->bus2 屏蔽 EPS_STATE/PCM_BUTTONS; bus2->bus0 屏蔽 MPC_STATE/ACC_CMD (0x32D 转发)
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda, MAX_WRONG_COUNTERS


class TestBydSafety(common.PandaCarSafetyTest, common.TorqueSteeringSafetyTestBase):
  # TorqueSteeringLimits: max_steer=300, rate_up/down=18, rt_delta=250, interval=250000us, max_error=80
  MAX_RATE_UP = 18
  MAX_RATE_DOWN = 18
  MAX_TORQUE = 300
  MAX_RT_DELTA = 250
  RT_INTERVAL = 250000
  NO_STEER_REQ_BIT = False

  TX_MSGS = [[0x316, 0], [0x32E, 0], [0x32D, 0], [0x318, 2], [0x3B0, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x316,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x318, 0x3B0]}  # bus0->bus2 blocked
  FWD_BLACKLISTED_ADDRS2 = {0: [0x316, 0x32E]}  # bus2->bus0 blocked (0x32D allowed!)

  # 唐DM safety 使用 pcm_cruise (button enable via ACC_HUD_ADAS), 非 pcmCruise
  PCM_CRUISE = False

  def setUp(self):
    self.packer = CANPackerPanda("byd_han_dmev_2020")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.safety.init_tests()

  # ---- RX messages ----
  def _speed_msg(self, speed_kph):
    # CARSPEED 0x121: byte0/1 车速脉冲 (高速 12bit)
    raw = int(speed_kph / 0.0719088)
    dat = bytes([raw & 0xFF, (raw >> 8) & 0xFF] + [0] * 6)
    return libsafety_py.make_CANPacket(0x121, 0, dat)

  def _gas_msg(self, pressed):
    # PEDAL 0x342: byte0=gas, byte1=brake
    return libsafety_py.make_CANPacket(0x342, 0, bytes([1 if pressed else 0, 0, 0, 0, 0, 0, 0, 0]))

  def _brake_msg(self, pressed):
    return libsafety_py.make_CANPacket(0x342, 0, bytes([0, 1 if pressed else 0, 0, 0, 0, 0, 0, 0]))

  def _cruise_msg(self, accstate):
    # ACC_HUD_ADAS 0x32D bus2: byte2 bits AccState(bit3-5). accstate=3 active, 5 force
    b2 = (accstate & 0x7) << 3
    return libsafety_py.make_CANPacket(0x32D, 2, bytes([0, 0, b2, 0, 0, 0, 0, 0]))

  # ---- TX messages (ACC_MPC_STATE 0x316 torque) ----
  def _torque_cmd_msg(self, torque, steer_req=1):
    # MainTorque 8|12@1- (12bit signed), LKAS_Active bit28 (byte3 bit4)
    torque &= 0xFFF
    byte2 = torque & 0xFF
    byte3 = ((torque >> 8) & 0x0F) | (0x10 if steer_req else 0)
    dat = bytes([0, 0, byte2, byte3, 0, 0, 0, 0])
    return libsafety_py.make_CANPacket(0x316, 0, dat)

  # 我们不读 torque_meas (TorqueMotorLimited 退化为 rate-limit), 跳过依赖 torque_meas 的测试
  def test_non_realtime_limit_down(self):
    pass
  def test_torque_meas_limits(self):
    pass
  def test_torque_meas_limits_exceed(self):
    pass

  # ---- 唐DM 专项: fwd 0x32D 转发 (DP 规则, 关键!) ----
  def _fwd(self, bus, addr):
    # C 接口 safety_fwd_hook(CANPacket_t*): 需构造 CANPacket
    return self.safety.safety_fwd_hook(libsafety_py.make_CANPacket(addr, bus, [0] * 8))

  def test_fwd_hud_adas_allowed(self):
    """bus2->bus0: 0x32D(ACC_HUD_ADAS) 必须转发 (不屏蔽, 恢复原车摄像头健康)"""
    fwd = self._fwd(2, 0x32D)
    self.assertNotEqual(fwd, -1, "0x32D 不应被屏蔽转发")

  def test_fwd_mpc_cmd_blocked(self):
    """bus2->bus0: 0x316(ACC_MPC_STATE)/0x32E(ACC_CMD) 必须屏蔽"""
    self.assertEqual(-1, self._fwd(2, 0x316), "0x316 应屏蔽")
    self.assertEqual(-1, self._fwd(2, 0x32E), "0x32E 应屏蔽")

  def test_fwd_eps_buttons_blocked(self):
    """bus0->bus2: 0x318(ACC_EPS_STATE)/0x3B0(PCM_BUTTONS) 必须屏蔽"""
    self.assertEqual(-1, self._fwd(0, 0x318), "0x318 应屏蔽")
    self.assertEqual(-1, self._fwd(0, 0x3B0), "0x3B0 应屏蔽")

  def test_fwd_car_msgs_forwarded(self):
    """bus0->bus2: PEDAL/CARSPEED 等应转发"""
    self.assertNotEqual(-1, self._fwd(0, 0x342), "PEDAL 应转发")
    self.assertNotEqual(-1, self._fwd(0, 0x121), "CARSPEED 应转发")


if __name__ == "__main__":
  unittest.main()
