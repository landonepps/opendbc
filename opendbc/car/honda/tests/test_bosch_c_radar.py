from dataclasses import asdict

import pytest

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda.bosch_c_radar import BoschCRadarInterface, CandidateCalibration, OBJECT_IDS, object_crc
from opendbc.car.honda.radar_interface import RadarInterface


CALIBRATION = CandidateCalibration(.05, 4096, -4.296, 1 / 128, .1, 1539)


def frame(address, counter=0, phase=None, wire=0, life=None, x=4700, y=0, velocity=1519, quality=1):
  value = 0
  fields = [(16, counter), (24, counter % 16 if phase is None else phase), (32, wire), (48, x),
            (64, y), (80, velocity), (144, quality), (271, 3 * counter % 4096 if life is None else life)]
  for start, raw in fields:
    value |= raw << start
  payload = value.to_bytes(64, 'little')
  return CanData(address, object_crc(address, payload).to_bytes(2, 'little') + payload[2:], 1)


def bank(counter=0, **kwargs):
  return [frame(address, counter, wire=1 if slot == 0 else 0, **kwargs) for slot, address in enumerate(OBJECT_IDS)]


def cp():
  return structs.CarParams.new_message(brand='honda', carFingerprint='HONDA_CRV_6G', radarUnavailable=True)


@pytest.fixture
def adapter():
  return BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION, clock=lambda: 300_000_000)


def test_required_explicit_calibration_and_platform_gate():
  with pytest.raises(TypeError):
    BoschCRadarInterface(cp(), structs.CarParamsSP())
  other = cp()
  other.carFingerprint = 'HONDA_CIVIC'
  with pytest.raises(ValueError, match='restricted'):
    BoschCRadarInterface(other, structs.CarParamsSP(), calibration=CALIBRATION)
  with pytest.raises(ValueError, match='receive bus'):
    BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION, bus=129)
  for value in (float('nan'), float('inf'), 0, -1):
    with pytest.raises(ValueError):
      CandidateCalibration(value, 4096, 0, 1 / 128, .1, 1539)


def test_normal_honda_interface_and_carparams_unchanged():
  params = cp()
  before = params.to_dict()
  normal = RadarInterface(params, structs.CarParamsSP())
  for _ in range(5):
    result = normal.update([(0, bank())])
  assert not result.points
  assert params.to_dict() == before


def test_all_slots_required_and_raw_diagnostics_preserved(adapter):
  frames = bank(quality=1023, y=8191)
  assert not adapter.update([(0, frames[:15])]).points
  result = adapter.update([(1, frames[15:])])
  assert len(result.points) == 1 and not result.errors.canError
  with structs.RadarData.from_bytes(result.to_bytes()) as read:
    assert read.points[0].dRel == pytest.approx(25.904)
    assert read.points[0].yRel == pytest.approx(-1 / 128)
    assert read.points[0].vRel == pytest.approx(-2)
  report = adapter.diagnostics()
  assert report['tracks'][0]['raw']['quality_container_raw'] == 1023
  assert report['calibration'] == asdict(CALIBRATION)


@pytest.mark.parametrize('bus', [0, 2, 129])
def test_other_buses_do_not_supply_or_refresh_tracks(adapter, bus):
  frames = [CanData(f.address, f.dat, bus) for f in bank()]
  result = adapter.update([(0, frames)])
  assert result.errors.canError and not result.points
  assert not adapter.decoder.counters


@pytest.mark.parametrize('fault', ['crc', 'length', 'duplicate_slot', 'duplicate_id', 'invalid_id', 'phase', 'counter'])
def test_bad_bank_does_not_refresh_points_and_clean_bank_recovers(adapter, fault):
  first = adapter.update([(0, bank())])
  old = [p.to_dict() for p in first.points]
  frames = bank(1)
  if fault == 'crc':
    f = frames[5]
    frames[5] = CanData(f.address, f.dat[:-1] + bytes([f.dat[-1] ^ 1]), f.src)
  elif fault == 'length':
    f = frames[5]
    frames[5] = CanData(f.address, f.dat[:-1], f.src)
  elif fault == 'duplicate_slot':
    frames.insert(1, frames[0])
  elif fault == 'duplicate_id':
    frames[1] = frame(OBJECT_IDS[1], 1, wire=1)
  elif fault == 'invalid_id':
    frames[1] = frame(OBJECT_IDS[1], 1, wire=64)
  elif fault == 'phase':
    frames = bank(1, phase=5)
  else:
    frames = bank(255)
  result = adapter.update([(60_000_000, frames)])
  assert result.errors.canError and [p.to_dict() for p in result.points] == old
  assert adapter.decoder.last_bank_ns == 0
  recovered = adapter.update([(130_000_000, bank(2))])
  assert not recovered.errors.canError and len(recovered.points) == 1


def test_replayed_bank_cannot_refresh_age_and_heartbeat_clears_tracks(adapter):
  initial = adapter.update([(0, bank())])
  adapter.update([(150_000_000, bank())])
  assert adapter.decoder.last_bank_ns == 0
  expired = adapter.update([])  # injected monotonic clock, no CAN traffic
  assert expired.errors.canError and not expired.points
  assert adapter.decoder.counters['repeated_frames'] == 16
  recovered = adapter.update([(310_000_000, bank())])
  assert recovered.points[0].trackId != initial.points[0].trackId


def test_missing_sweep_keeps_identity_but_lifecycle_reset_does_not(adapter):
  first = adapter.update([(0, bank())]).points[0].trackId
  gap = adapter.update([(130_000_000, bank(2))])
  assert gap.points[0].trackId == first
  reset = adapter.update([(190_000_000, bank(3, life=99))])
  assert reset.points[0].trackId != first


def test_counter_phase_lifecycle_wrap_and_slot_migration(adapter):
  tid = adapter.update([(0, bank(254, life=4092))]).points[0].trackId
  moved = bank(255, life=4095)
  moved[0] = frame(OBJECT_IDS[0], 255)
  moved[1] = frame(OBJECT_IDS[1], 255, wire=1, life=4095)
  assert adapter.update([(60_000_000, moved)]).points[0].trackId == tid
  assert adapter.update([(130_000_000, bank(0, life=2))]).points[0].trackId == tid


def test_complete_empty_bank_removes_objects(adapter):
  adapter.update([(0, bank())])
  empty = [frame(address, 1) for address in OBJECT_IDS]
  result = adapter.update([(60_000_000, empty)])
  assert not result.points and not result.errors.canError


def test_partial_banks_never_mix(adapter):
  adapter.update([(0, bank()[:8])])
  result = adapter.update([(60_000_000, bank(1)[8:])])
  assert not result.points and result.errors.canError
  assert adapter.decoder.counters['incomplete_banks'] == 1
  assert len(adapter.update([(130_000_000, bank(2))]).points) == 1


def test_batch_contract_preserves_latest_complete_bank(adapter):
  result = adapter.update([(0, bank()), (60_000_000, bank(1)), (130_000_000, bank(2, x=4800))])
  assert result.points[0].dRel == pytest.approx(30.904)
  assert adapter.decoder.counters['accepted_banks'] == 3


def test_out_of_order_batch_rejected_before_any_mutation(adapter):
  before = adapter.diagnostics()
  with pytest.raises(ValueError, match='nondecreasing'):
    adapter.update([(10, bank()), (5, bank(1))])
  assert adapter.diagnostics() == before


def test_track_order_stays_stable_when_new_object_uses_earlier_slot(adapter):
  first = bank()
  first[0] = frame(OBJECT_IDS[0])
  first[1] = frame(OBJECT_IDS[1], wire=1)
  first[2] = frame(OBJECT_IDS[2], wire=2)
  initial = adapter.update([(0, first)])
  second = bank(1)
  second[0] = frame(OBJECT_IDS[0], 1, wire=3)
  second[1] = frame(OBJECT_IDS[1], 1, wire=1)
  second[2] = frame(OBJECT_IDS[2], 1, wire=2)
  result = adapter.update([(60_000_000, second)])
  assert [p.trackId for p in result.points][:2] == [p.trackId for p in initial.points]
  assert [t.raw.wire_id for t in adapter.decoder.tracks.values()] == [1, 2, 3]


def test_healthy_stream_emits_banks_and_fault_stream_emits_heartbeats(adapter):
  assert adapter.update([(0, bank())]) is not None
  assert adapter.update([(50_000_000, [])]) is None
  assert adapter.update([(60_000_000, bank(1))]) is not None
  stale = adapter.update([], now_nanos=270_000_000)
  assert stale.errors.canError and not stale.points
  assert adapter.update([], now_nanos=280_000_000) is None
  assert adapter.update([], now_nanos=320_000_000).errors.canError
