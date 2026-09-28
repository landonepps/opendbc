from dataclasses import asdict

import pytest

from opendbc.car import structs
from opendbc.car.honda.bosch_c_radar_live import BoschCLiveRadarInterface, configure
from opendbc.car.honda.radar_interface import RadarInterface
from opendbc.car.honda.tests.test_bosch_c_radar import bank, cp
from opendbc.car.honda.values import HondaFlags
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP
from opendbc.sunnypilot.car.interfaces import _initialize_honda


def params():
  p = cp()
  p.flags = int(HondaFlags.BOSCH | HondaFlags.BOSCH_CANFD)
  p.openpilotLongitudinalControl = True
  p.safetyConfigs = [{'safetyModel': 'hondaBosch', 'safetyParam': 18}]
  return p, structs.CarParamsSP()


def live():
  p, sp = params()
  configure(p, sp, True)
  clock = [1_000_000_000]
  return BoschCLiveRadarInterface(p, sp, clock=lambda: clock[0]), clock


def test_default_off_preserves_entire_configuration_and_normal_selection():
  p, sp = params()
  before = p.to_dict(), asdict(sp)
  _initialize_honda(p, sp, {})
  assert (p.to_dict(), asdict(sp)) == before
  assert RadarInterface(p, sp).bosch_c is None


def test_enabled_changes_only_radar_availability_and_recorded_experimental_flag():
  p, sp = params()
  before = p.to_dict(), asdict(sp)
  _initialize_honda(p, sp, {'HondaBoschCExperimentalRadar': '1'})
  before[0]['radarUnavailable'] = False
  before[1]['flags'] |= HondaFlagsSP.EXPERIMENTAL_BOSCH_C_RADAR
  assert (p.to_dict(), asdict(sp)) == before
  ri = RadarInterface(p, sp)
  ri.bosch_c.clock = lambda: 1_000_000_000
  result = ri.update([(999_000_000, bank())])
  assert result.points and not result.errors.radarUnavailableTemporary


@pytest.mark.parametrize('field,value', [('carFingerprint', 'HONDA_CIVIC'), ('brand', 'toyota'),
                                        ('openpilotLongitudinalControl', False), ('flags', int(HondaFlags.BOSCH))])
def test_unsupported_configuration_stays_off(field, value):
  p, sp = params()
  setattr(p, field, value)
  before = p.to_dict(), asdict(sp)
  configure(p, sp, True)
  assert (p.to_dict(), asdict(sp)) == before


def test_stock_override_wins_without_enabling_radar_or_longitudinal():
  p, sp = params()
  _initialize_honda(p, sp, {'HondaBoschCExperimentalRadar': '1', 'HondaEnforceStockLongitudinal': '1'})
  assert not p.openpilotLongitudinalControl and p.radarUnavailable
  assert not sp.flags & HondaFlagsSP.EXPERIMENTAL_BOSCH_C_RADAR


def test_untested_bus_offset_stays_off():
  p, sp = params()
  p.safetyConfigs = [{'safetyModel': 'noOutput'}, {'safetyModel': 'hondaBosch', 'safetyParam': 18}]
  configure(p, sp, True)
  assert p.radarUnavailable and not sp.flags & HondaFlagsSP.EXPERIMENTAL_BOSCH_C_RADAR


def test_manually_inconsistent_selection_fails_closed():
  p, sp = params()
  sp.flags |= HondaFlagsSP.EXPERIMENTAL_BOSCH_C_RADAR.value
  with pytest.raises(ValueError, match='explicit supported'):
    RadarInterface(p, sp)


def test_stale_receive_batch_never_publishes_old_points_and_fresh_bank_recovers():
  ri, now = live()
  result = ri.update([(now[0]-300_000_000, bank())])
  assert result.errors.radarUnavailableTemporary and not result.points
  now[0] += 10_000_000
  result = ri.update([(now[0]-1_000_000, bank(1))])
  assert result.points and not result.errors.radarUnavailableTemporary


def test_healthy_calls_do_not_advance_past_inflight_can():
  ri, now = live()
  assert ri.update([(now[0]-10_000_000, bank())]).points
  # New CAN can legitimately predate the previous receive wall-clock time.
  now[0] += 10_000_000
  result = ri.update([(now[0]-15_000_000, bank(1))])
  assert result.points and not result.errors.radarUnavailableTemporary
  assert not ri.decoder.counters['live_discarded_packets']


def test_partial_first_bank_uses_packet_clock_until_complete():
  ri, now = live()
  frames = bank()
  ri.update([(now[0]-10_000_000, frames[:8])])
  now[0] += 10_000_000
  result = ri.update([(now[0]-15_000_000, frames[8:])])
  assert result.points and not result.errors.radarUnavailableTemporary


def test_timeout_suspend_and_late_packet_recovery():
  ri, now = live()
  assert ri.update([(now[0], bank())]).points
  now[0] += 60_000_000_000
  result = ri.update([])
  assert result.errors.radarUnavailableTemporary and not result.points
  now[0] += 60_000_000
  result = ri.update([(now[0]-100_000_000, bank(1))])
  assert result.errors.radarUnavailableTemporary and not result.points
  now[0] += 10_000_000
  result = ri.update([(now[0]-1_000_000, bank(2))])
  assert result.points and not result.errors.radarUnavailableTemporary


def test_processing_delay_cannot_publish_a_bank_that_expired_during_update():
  p, sp = params()
  configure(p, sp, True)
  clocks = iter([1_000_000_000, 1_300_000_000])
  ri = BoschCLiveRadarInterface(p, sp, clock=lambda: next(clocks))
  result = ri.update([(1_000_000_000, bank())])
  assert result.errors.radarUnavailableTemporary and not result.points


def test_future_clock_packet_is_invalid_and_does_not_poison_decoder():
  ri, now = live()
  result = ri.update([(now[0]+60_000_000_000, bank())])
  assert result.errors.canError and not result.points
  assert ri.decoder.now_ns is None
  assert ri.update([(now[0], bank())]).points
