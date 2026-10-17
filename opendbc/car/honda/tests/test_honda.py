import unittest

from opendbc.can import CANPacker
from opendbc.car.honda import hondacan, hud_objects, lane_path
from opendbc.car.honda.carstate import TickReference
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, HondaFlags, HondaSafetyFlags
from opendbc.car.interfaces import gen_empty_fingerprint


class TestHondaFingerprint(unittest.TestCase):
  def test_tja_bosch_only(self):
    for car_model in CAR:
      if car_model.config.flags & HondaFlags.BOSCH_TJA_CONTROL:
        assert car_model.config.flags & HondaFlags.BOSCH, "Nidec car found with TJA control"


class TestHondaCanfdDashHud(unittest.TestCase):
  """CAN FD dash look-alikes (LANE_PATH, HUD_OBJECTS, lane summary) go where each car's own radar sends them."""

  def test_flag_only_on_canfd(self):
    for car_model in CAR:
      if car_model.config.flags & HondaFlags.CANFD_RADARLESS_HUD_ADDR:
        assert car_model.config.flags & HondaFlags.BOSCH_CANFD, car_model
    assert CAR.HONDA_CRV_6G.config.flags & HondaFlags.CANFD_RADARLESS_HUD_ADDR
    assert CAR.HONDA_PILOT_4G.config.flags & HondaFlags.CANFD_RADARLESS_HUD_ADDR
    assert not CAR.ACURA_MDX_4G_MMR.config.flags & HondaFlags.CANFD_RADARLESS_HUD_ADDR

  def test_safety_param(self):
    # set only with openpilot longitudinal, where the look-alikes are authored; stock ACC keeps its param
    for car_model, flagged in ((CAR.HONDA_CRV_6G, True), (CAR.HONDA_PILOT_4G, True), (CAR.ACURA_MDX_4G_MMR, False)):
      for alpha_long in (False, True):
        CP = CarInterface.get_params(car_model, gen_empty_fingerprint(), [], alpha_long, False, False)
        param = CP.safetyConfigs[-1].safetyParam
        expected = flagged and CP.openpilotLongitudinalControl
        assert bool(param & HondaSafetyFlags.CANFD_RADARLESS_HUD_ADDR) == expected, (car_model, alpha_long, param)

  def test_packed_addresses(self):
    packer = CANPacker('honda_common_canfd_generated')
    lane = lane_path.create_lane_path(packer, 0, [lane_path.OFFSET_UNAVAILABLE] * lane_path.NUM_PTS, 1)
    lane_alt = lane_path.create_lane_path(packer, 0, [lane_path.OFFSET_UNAVAILABLE] * lane_path.NUM_PTS, 1, "LANE_PATH_ALT")
    hud = hud_objects.create_hud_object(packer, 0, 1, None)
    hud_alt = hud_objects.create_hud_object(packer, 0, 1, None, "HUD_OBJECTS_ALT")
    # RADAR_LEAD2 first, then the lane summary
    lead = hondacan.create_canfd_5hz_radar_messages(packer, 0, 1)[-1]
    lead_alt = hondacan.create_canfd_5hz_radar_messages(packer, 0, 1, radar_lead_name='RADAR_LEAD_ALT')[-1]
    assert (lane[0], hud[0], lead[0]) == (0x6CD5558, 0x6CD5559, 0xF31AA5C)
    assert (lane_alt[0], hud_alt[0], lead_alt[0]) == (0x6CD5554, 0x6CD5557, 0xF31AA54)
    # same signal layout: identical payloads apart from the address-dependent checksum/counter byte
    for a, b in ((lane, lane_alt), (hud, hud_alt), (lead, lead_alt)):
      assert a[1][:7] == b[1][:7]

  def test_matches_stock_crv_radar_frames(self):
    # idle frames recorded from a 2026 CR-V's own radar on bus 0 (route 0000018b--31e9814e11, segment 5)
    packer = CANPacker('honda_common_canfd_generated')
    frames = [
      ('LANE_PATH_ALT', {'MUX': 8, 'PATH_OFFSET_1': 2047, 'PATH_OFFSET_2': 2047, 'PATH_OFFSET_3': 2047, 'PATH_OFFSET_4': 2047, 'COUNTER': 2},
       0x6CD5554, '207ff7ff7ff7ff28'),
      ('HUD_OBJECTS_ALT', {'MUX': 8, **hud_objects.INACTIVE, 'COUNTER': 2}, 0x6CD5557, '2000f080ffc7ff23'),
      ('RADAR_LEAD_ALT', {'CNTR_REF': 1, 'SET_ME_X01': 1, 'TARGET_SPEED_MAYBE': 140, 'LANE_PATH_LENGTH': 6, 'COUNTER': 3},
       0xF31AA54, '608c00180000003c'),
    ]
    for name, values, addr, stock in frames:
      packed_addr, dat, _ = packer.make_can_msg(name, 0, values)
      assert packed_addr == addr and dat.hex() == stock, (name, dat.hex())


class TestTickReference(unittest.TestCase):
  """The radar's tick references pace the CAN FD dash look-alikes: one pulse per reference, delay frames later."""

  @staticmethod
  def pulses(refs_per_frame, delay):
    tick = TickReference(delay)
    return [i for i, n in enumerate(refs_per_frame) if tick.update(n)]

  def test_steady_50hz(self):
    # a reference every other frame pulses the frame after each one
    assert self.pulses([1, 0] * 5, 1) == [1, 3, 5, 7, 9]

  def test_consecutive_references(self):
    # radar jitter puts two references in back-to-back frames: both still pulse, in turn
    assert self.pulses([1, 1, 0, 0, 1, 0], 1) == [1, 2, 5]

  def test_early_10hz_reference(self):
    # the third reference arrives a frame early (9 frames after the second): one pulse each, none skipped
    refs = [1] + [0] * 9 + [1] + [0] * 8 + [1] + [0] * 10
    assert self.pulses(refs, 9) == [9, 19, 28]

  def test_backlog_is_capped(self):
    # a late batch carrying many references catches up by at most MAX_PENDING pulses
    assert self.pulses([5, 0, 0, 0], 1) == [1, 2]
