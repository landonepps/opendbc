from dataclasses import asdict

import pytest

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda.bosch_c_radar import BoschCRadarInterface, CandidateCalibration, OBJECT_IDS, object_crc
from opendbc.car.honda.radar_interface import RadarInterface


CALIBRATION = CandidateCalibration(.05, 0, -4.296, 1 / 128, .1, 1539)


def frame(address, counter=0, phase=None, wire=0, life=None, x=4700, y=0, velocity=1519, quality=1, uncertainty=4, y14=None):
  # y is the 13-bit two's complement lateral; the radar sets bit 77 to the inverse of bit 76 within +-40.95 m.
  # y14 overrides it with the full 14-bit offset-binary value at bits 64-77.
  value = 0
  y14 = y | ((0 if y & 4096 else 1) << 13) if y14 is None else y14
  fields = [(16, counter), (24, counter % 16 if phase is None else phase), (32, wire), (48, x),
            (64, y14), (80, velocity), (144, quality), (176, uncertainty), (271, 3 * counter % 4096 if life is None else life)]
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
      CandidateCalibration(value, 0, 0, 1 / 128, .1, 1539)


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
  assert len(result.points) == 1 and not result.errors.radarUnavailableTemporary
  with structs.RadarData.from_bytes(result.to_bytes()) as read:
    assert read.points[0].dRel == pytest.approx(25.904)
    assert read.points[0].yRel == pytest.approx(-1 / 128)
    assert read.points[0].vRel == pytest.approx(-2)
  report = adapter.diagnostics()
  assert report['tracks'][0]['raw']['quality_container_raw'] == 1023
  assert report['calibration'] == asdict(CALIBRATION)


@pytest.mark.parametrize('y14, meters', [(8192 + 4115, 41.15), (8192 - 4115, -41.15), (8192 + 4095, 40.95), (8192 - 4096, -40.96)])
def test_lateral_is_14bit_offset_binary(y14, meters):
  # a 13-bit two's complement decode reads 12307 as -40.77 m; bit 77 carries the sign past +-40.96 m
  from opendbc.car.honda.bosch_c_radar import unpack
  f = frame(OBJECT_IDS[0], y14=y14)
  assert CandidateCalibration(.05, 0, -4.296, .01, .1, 1539).convert(unpack(f.address, f.dat))[1] == pytest.approx(meters)


def test_far_lateral_object_does_not_wrap_into_the_path():
  # +72 m lateral would read as -9.92 m under a 13-bit decode, inside the |y| < 20 m publication guard
  calibrated = BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CandidateCalibration(.05, 0, -4.296, .01, .1, 1539),
                                    clock=lambda: 300_000_000)
  frames = [frame(address, 0, wire=1 if slot == 0 else 0, y14=8192 + 7200) for slot, address in enumerate(OBJECT_IDS)]
  result = calibrated.update([(0, frames)])
  assert not result.points and calibrated.guard_rejected == 1


@pytest.mark.parametrize('bus', [0, 2, 129])
def test_other_buses_do_not_supply_or_refresh_tracks(adapter, bus):
  frames = [CanData(f.address, f.dat, bus) for f in bank()]
  result = adapter.update([(0, frames)])
  assert result.errors.radarUnavailableTemporary and not result.points
  assert not adapter.decoder.counters


@pytest.mark.parametrize('fault', ['crc', 'length', 'duplicate_slot', 'duplicate_id', 'invalid_id', 'phase', 'counter'])
def test_bad_bank_does_not_refresh_or_flag_points_and_clean_bank_recovers(adapter, fault):
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
  # A rejected bank publishes nothing: no error flag, and the last accepted
  # points stand until they expire or a clean bank replaces them.
  assert adapter.update([(60_000_000, frames)]) is None
  assert adapter.decoder.fault and adapter.decoder.last_bank_ns == 0
  assert [p.to_dict() for p in adapter.snapshot(60_000_000).points] == old
  recovered = adapter.update([(130_000_000, bank(2))])
  assert not any(recovered.errors.to_dict().values()) and len(recovered.points) == 1


def test_replayed_bank_cannot_refresh_age_and_heartbeat_clears_tracks(adapter):
  initial = adapter.update([(0, bank())])
  adapter.update([(150_000_000, bank())])
  assert adapter.decoder.last_bank_ns == 0
  expired = adapter.update([])  # injected monotonic clock, no CAN traffic
  assert expired.errors.radarUnavailableTemporary and not expired.points
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
  assert not result.points and not result.errors.radarUnavailableTemporary


def test_partial_banks_never_mix(adapter):
  adapter.update([(0, bank()[:8])])
  result = adapter.update([(60_000_000, bank(1)[8:])])
  assert not result.points and result.errors.radarUnavailableTemporary
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
  assert stale.errors.radarUnavailableTemporary and not stale.points
  assert adapter.update([], now_nanos=280_000_000) is None
  assert adapter.update([], now_nanos=320_000_000).errors.radarUnavailableTemporary


def test_default_timeout_clock_matches_can_boottime_after_suspend(monkeypatch):
  import time
  from opendbc.car.honda import bosch_c_radar

  # Linux CAN Event.logMonoTime includes suspended time; CLOCK_MONOTONIC does not.
  can_ns = time.monotonic_ns() + 60_000_000_000
  boot_clock = 9876
  now = [can_ns + 60_000_000]
  clocks = []
  monkeypatch.setattr(bosch_c_radar.time, 'CLOCK_BOOTTIME', boot_clock, raising=False)

  def read_clock(clock):
    clocks.append(clock)
    return now[0]

  monkeypatch.setattr(bosch_c_radar.time, 'clock_gettime_ns', read_clock)
  native = BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION)
  initial = native.update([(can_ns, bank())])
  assert initial.points
  assert native.update([]) is None
  assert native.decoder.tracks
  now[0] = can_ns + 201_000_000
  expired = native.update([])
  assert expired.errors.radarUnavailableTemporary and not expired.points
  assert clocks == [boot_clock, boot_clock]


def test_default_timeout_clock_has_monotonic_fallback(monkeypatch):
  from opendbc.car.honda import bosch_c_radar

  clocks = []
  monkeypatch.delattr(bosch_c_radar.time, 'CLOCK_BOOTTIME', raising=False)
  monkeypatch.setattr(bosch_c_radar.time, 'clock_gettime_ns', lambda clock: clocks.append(clock) or 300_000_000)
  native = BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION)
  native.update([(0, bank())])
  expired = native.update([])
  assert expired.errors.radarUnavailableTemporary and not expired.points
  assert clocks == [bosch_c_radar.time.CLOCK_MONOTONIC]


def objects_bank(counter, objects):
  """objects: {slot: dict(wire=..., x=..., uncertainty=...)}; other slots empty."""
  return [frame(address, counter, **objects.get(slot, {})) for slot, address in enumerate(OBJECT_IDS)]


def gated(clock_ns=300_000_000, enabled=True):
  return BoschCRadarInterface(cp(), structs.CarParamsSP(), calibration=CALIBRATION, clock=lambda: clock_ns,
                              uncertainty_gate=enabled)


FAR, NEAR = 5000, 4700  # about 40.9 m and 25.9 m with CALIBRATION
SETTLED = dict(wire=2, x=NEAR, uncertainty=3)


def feed(radar, counter, objects):
  return radar.update([(counter * 66_000_000, objects_bank(counter, objects))])


def test_uncertainty_gate_is_off_by_default():
  radar = gated(enabled=False)
  result = feed(radar, 0, {0: dict(wire=1, x=FAR, uncertainty=100), 1: SETTLED})
  assert {p.trackId for p in result.points} == {1, 2}


def test_uncertainty_gate_omits_far_newborn_until_settled():
  radar = gated()
  for counter, u in enumerate((112, 49, 17, 13)):
    result = feed(radar, counter, {0: dict(wire=1, x=FAR, uncertainty=u), 1: SETTLED})
    assert [p.trackId for p in result.points] == [2]
  result = feed(radar, 4, {0: dict(wire=1, x=FAR, uncertainty=12), 1: SETTLED})
  assert sorted(p.trackId for p in result.points) == [1, 2]
  assert radar.decoder.counters['gate_omitted_points'] == 4


def test_uncertainty_gate_never_omits_near_points():
  radar = gated()
  result = feed(radar, 0, {0: dict(wire=1, x=NEAR, uncertainty=500), 1: SETTLED})
  assert sorted(p.trackId for p in result.points) == [1, 2]


def test_uncertainty_gate_publishes_unmodified_points_under_their_own_id():
  radar = gated()
  ungated = gated(enabled=False)
  for counter, u in enumerate((5, 40, 5)):
    objects = {0: dict(wire=1, x=FAR, y=300, velocity=1500, uncertainty=u), 1: SETTLED}
    points = {p.trackId: p for p in feed(radar, counter, objects).points}
    expected = {p.trackId: p for p in feed(ungated, counter, objects).points}
    assert points.keys() == ({2} if u > 12 else {1, 2})
    for track_id, p in points.items():
      assert p.to_dict() == expected[track_id].to_dict()


def test_uncertainty_gate_fails_open_until_the_field_has_ever_settled():
  radar = gated()
  result = feed(radar, 0, {0: dict(wire=1, x=FAR, uncertainty=1023)})
  assert [p.trackId for p in result.points] == [1]
  assert radar.decoder.counters['gate_fail_open_banks'] == 1


def test_uncertainty_gate_omits_a_lone_newborn_but_fails_open_when_the_field_sticks():
  radar = gated()
  feed(radar, 0, {1: SETTLED})
  # A bank with only an unsettled far newborn shortly after a settled object: still gated.
  assert not feed(radar, 1, {0: dict(wire=1, x=FAR, uncertainty=300)}).points
  # Stuck above the threshold for more than GATE_FAIL_OPEN_NS: publish ungated.
  counter = 1
  while counter * 66_000_000 <= 2_100_000_000:
    counter += 1
    result = feed(radar, counter, {0: dict(wire=1, x=FAR, uncertainty=300)})
  assert [p.trackId for p in result.points] == [2] and radar.decoder.counters['gate_fail_open_banks'] >= 1


def test_rejected_banks_become_unavailable_only_after_the_stale_limit(adapter):
  adapter.update([(0, bank())])
  corrupt = []
  for counter in range(1, 4):
    frames = bank(counter)
    f = frames[5]
    frames[5] = CanData(f.address, f.dat[:-1] + bytes([f.dat[-1] ^ 1]), f.src)
    corrupt.append(adapter.update([(counter * 66_000_000, frames)]))
  assert corrupt == [None, None, None]
  stale = adapter.update([], now_nanos=270_000_000)
  errors = stale.errors.to_dict()
  assert errors.pop('radarUnavailableTemporary') and not any(errors.values()) and not stale.points


def test_shipped_dbc_matches_adapter_decode():
  # opendbc/dbc/honda_bosch_c_radar.dbc is for analysis tools; the adapter decodes the same bits itself.
  import random
  from opendbc.can import CANParser
  from opendbc.car.honda.bosch_c_radar import unpack
  from opendbc.car.honda.bosch_c_radar_live import PROVISIONAL_CALIBRATION

  names = dict(wire_id='OBJECT_ID_RAW', frame_counter='FRAME_COUNTER_RAW', frame_phase='FRAME_PHASE_RAW',
               lifecycle='LIFECYCLE_RAW', range_raw='POSITION_X_RAW', status='OBJECT_CLASS_RAW', y_raw='POSITION_Y_RAW',
               y_companion_raw='Y_COMPANION_RAW', velocity_raw='VELOCITY_RAW', quality_container_raw='QUALITY_CONTAINER_RAW',
               lateral_velocity_candidate_raw='LATERAL_VELOCITY_CANDIDATE_RAW',
               normalized_rate_candidate_raw='NORMALIZED_RATE_CANDIDATE_RAW',
               uncertainty_candidate_raw='REL_VELOCITY_UNCERTAINTY_RAW')
  rng = random.Random(0)
  parser = CANParser('honda_bosch_c_radar', [(address, 0) for address in OBJECT_IDS], 1)
  for step in range(50):
    payloads = {address: bytes(rng.getrandbits(8) for _ in range(64)) for address in OBJECT_IDS}
    parser.update([(step, [(address, payload, 1) for address, payload in payloads.items()])])
    for address, payload in payloads.items():
      raw, dbc = unpack(address, payload), parser.vl[address]
      assert {field: getattr(raw, field) for field in names} == {field: dbc[name] for field, name in names.items()}
      x, y, v = PROVISIONAL_CALIBRATION.convert(raw)
      assert (dbc['DREL'], dbc['YREL'], dbc['VREL']) == pytest.approx((x, y, v))
      angles = int.from_bytes(payload, 'little')
      assert dbc['TTC'] == pytest.approx(((angles >> 201) & 2047) / 128 - 8)
      for start, name in ((411, 'AZIMUTH_CENTER'), (424, 'AZIMUTH_EDGE_A'), (440, 'AZIMUTH_EDGE_B')):
        assert dbc[name] == pytest.approx(((angles >> start) & 8191) / 4096 - 1)


def test_shipped_dbc_ego_motion_fields():
  # 0x401 on the private camera-radar bus (research: bosch-c-research docs/bus1-ego-motion-0x401.md)
  import random
  from opendbc.can import CANParser

  def signed(value, bits):
    return value - (1 << bits) if value >= 1 << (bits - 1) else value

  rng = random.Random(0)
  parser = CANParser('honda_bosch_c_radar', [(0x401, 0)], 1)
  for step in range(50):
    payload = bytes(rng.getrandbits(8) for _ in range(64))
    parser.update([(step, [(0x401, payload, 1)])])
    dbc, value = parser.vl[0x401], int.from_bytes(payload, 'little')
    # yaw rate: big-endian, byte 45 bits 6-0 then byte 46 bits 7-3
    assert dbc['YAW_RATE'] == pytest.approx(signed(((payload[45] & 0x7F) << 5) | (payload[46] >> 3), 12) * 0.0005257)
    assert dbc['SPEED_CANDIDATE_RAW'] == (payload[3] << 2) | (payload[4] >> 6)
    assert dbc['STEER_ANGLE_CANDIDATE_RAW'] == signed((value >> 103) & 511, 9)
    assert dbc['ACCEL_CANDIDATE_RAW'] == signed(payload[11], 8)
    assert dbc['LAT_ACCEL'] == pytest.approx(signed(payload[10], 8) * 0.198)


def status_x(raw12, status):
  # 48:12 range plus the 60:4 status field, laid out as the 16 bits from bit 48
  return raw12 | (status << 12)


def test_status_and_12bit_range_for_display(adapter):
  # classes 1 (car), 6 (bit 60 clear) and 7; only the car passes the RadarData guard, and 7 is not drawn
  objs = {0: (1, status_x(600, 1)), 1: (2, status_x(700, 6)), 2: (3, status_x(500, 7))}
  frames = [frame(address, 0, wire=objs[slot][0] if slot in objs else 0, x=objs[slot][1] if slot in objs else 4700)
            for slot, address in enumerate(OBJECT_IDS)]
  result = adapter.update([(0, frames)])
  statuses = {t.raw.wire_id: t.raw.status for t in adapter.decoder.tracks.values()}
  assert statuses == {1: 1, 2: 6, 3: 7}
  assert [p.dRel for p in result.points] == [pytest.approx(600 * .05 - 4.296)]
  shown = {o.status: o.d_rel for o in adapter.display_objects()}
  assert shown == {1: pytest.approx(600 * .05 - 4.296), 6: pytest.approx(700 * .05 - 4.296)}
