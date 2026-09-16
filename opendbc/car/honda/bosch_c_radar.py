"""Experimental Bosch C receive-only adapter for the existing RadarData contract.

Not registered with Honda's production RadarInterface. Units and measurement
validity remain research hypotheses. Construction requires explicit calibration;
use through the passive diagnostic tool or offline replay until validated.
"""
from binascii import crc_hqx
from collections import Counter
from dataclasses import asdict, dataclass
import math
import time

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda.values import CAR
from opendbc.car.interfaces import RadarInterfaceBase


OBJECT_IDS = tuple(range(0x62, 0x82, 2))
STALE_NS = 200_000_000


def object_crc(address: int, payload: bytes) -> int:
  if address not in OBJECT_IDS or len(payload) != 64:
    raise ValueError('Expected a 64-byte Bosch C object message')
  data_id = 0x0FAB + (address - 0x62) // 2
  return crc_hqx(payload[2:] + data_id.to_bytes(2, 'little'), 0xFFFF)


@dataclass(frozen=True)
class RawObject:
  slot: int
  wire_id: int
  frame_counter: int
  frame_phase: int
  lifecycle: int
  x_raw: int
  x_companion_raw: int
  y_raw: int
  y_companion_raw: int
  velocity_raw: int
  quality_container_raw: int
  lateral_velocity_candidate_raw: int
  normalized_rate_candidate_raw: int

  @property
  def signed_y_raw(self):
    return self.y_raw - 8192 if self.y_raw & 4096 else self.y_raw


def unpack(address: int, payload: bytes) -> RawObject:
  if address not in OBJECT_IDS or len(payload) != 64:
    raise ValueError('Expected a 64-byte Bosch C object message')
  value = int.from_bytes(payload, 'little')

  def bits(start, width):
    return (value >> start) & ((1 << width) - 1)

  return RawObject((address - 0x62) // 2, bits(32, 16), payload[2], payload[3] & 15,
                   bits(271, 12), bits(48, 13), bits(61, 3), bits(64, 13), bits(77, 3),
                   bits(80, 11), bits(144, 16), bits(96, 10), bits(128, 16))


@dataclass(frozen=True)
class CandidateCalibration:
  x_scale: float
  x_zero: int
  x_reference_offset: float
  y_scale: float
  velocity_scale: float
  velocity_zero: int

  def __post_init__(self):
    if not all(math.isfinite(v) for v in asdict(self).values()):
      raise ValueError('Calibration must be finite')
    if min(self.x_scale, self.y_scale, self.velocity_scale) <= 0:
      raise ValueError('Calibration scales must be positive')

  def convert(self, raw):
    return ((raw.x_raw - self.x_zero) * self.x_scale + self.x_reference_offset,
            raw.signed_y_raw * self.y_scale,
            (raw.velocity_raw - self.velocity_zero) * self.velocity_scale)


@dataclass
class ObjectTrack:
  track_id: int
  time_ns: int
  first_seen_ns: int
  raw: RawObject


class BoschCDecoder:
  """Coherent banks, capture-verified CRC, and bounded identity continuity.

  Reject malformed or ambiguous banks without refreshing tracks. A clean bank
  clears the integrity fault. No hardware fault/status bits have been decoded.
  """
  def __init__(self, bus: int):
    if not 0 <= bus < 128:
      raise ValueError('Expected a receive bus, not a Panda transmit receipt bus')
    self.bus = bus
    self.now_ns = None
    self.pending = {}
    self.pending_key = None
    self.pending_time_ns = None
    self.pending_bad = False
    self.last_key = None
    self.last_bank_ns = None
    self.tracks = {}
    self.next_track_id = 1
    self.fault = False
    self.counters = Counter()

  def expire(self, now_ns):
    if not isinstance(now_ns, int) or now_ns < 0:
      raise ValueError('Expected nonnegative integer nanoseconds')
    if self.now_ns is not None and now_ns < self.now_ns:
      raise ValueError('CAN timestamps must be nondecreasing')
    self.now_ns = now_ns
    for wire in list(self.tracks):
      if now_ns - self.tracks[wire].time_ns > STALE_NS:
        del self.tracks[wire]
        self.counters['expired_tracks'] += 1
    if self.pending_time_ns is not None and now_ns - self.pending_time_ns > STALE_NS:
      self.counters['incomplete_banks'] += 1
      self._clear_pending()
    if self.last_bank_ns is not None and now_ns - self.last_bank_ns > STALE_NS:
      self.last_key = None  # Allow a sensor restart or repeated counter after a gap.

  def _clear_pending(self):
    self.pending = {}
    self.pending_key = None
    self.pending_time_ns = None
    self.pending_bad = False

  def _fault(self, name):
    self.counters[name] += 1
    self.fault = True
    if self.pending:
      self.pending_bad = True

  def feed(self, now_ns, frame: CanData):
    self.expire(now_ns)
    address, payload, bus = frame
    if bus != self.bus or address not in OBJECT_IDS:
      return None
    self.counters['object_frames'] += 1
    if len(payload) != 64:
      self._fault('malformed_frames')
      return None
    if int.from_bytes(payload[:2], 'little') != object_crc(address, payload):
      self._fault('checksum_failures')
      return None
    raw = unpack(address, payload)
    key = (raw.frame_counter, raw.frame_phase)
    if key == self.last_key:
      self.counters['repeated_frames'] += 1
      return None
    if key != self.pending_key:
      previous_key = self.pending_key if self.pending_key is not None else self.last_key
      if previous_key is not None:
        delta = (key[0] - previous_key[0]) % 256
        if not 0 < delta <= 3 or (key[1] - previous_key[1]) % 16 != delta:
          self._fault('counter_errors')
          return None
        self.counters['missing_counter_steps'] += delta - 1
      if self.pending:
        self.counters['incomplete_banks'] += 1
      self._clear_pending()
      self.pending_key, self.pending_time_ns = key, now_ns
    if raw.slot in self.pending:
      self._fault('duplicate_slots')
    self.pending[raw.slot] = raw
    if len(self.pending) != 16:
      return None
    self.counters['complete_banks'] += 1
    active = [obj for obj in self.pending.values() if obj.wire_id]
    ids = [obj.wire_id for obj in active]
    if len(ids) != len(set(ids)):
      self._fault('duplicate_ids')
    if any(not 1 <= wire <= 63 for wire in ids):
      self._fault('invalid_ids')
    bad = self.pending_bad
    self._clear_pending()
    if bad:
      self.counters['rejected_banks'] += 1
      return None
    # Preserve insertion order across slot migration, matching the offline
    # baseline and keeping consumer tie-breaking independent of slot order.
    updated = self.tracks.copy()
    active_ids = set(ids)
    for obj in active:
      old = self.tracks.get(obj.wire_id)
      frames = (obj.frame_counter - old.raw.frame_counter) % 256 if old else 0
      continuing = old is not None and 0 < frames <= 3 and (obj.lifecycle - old.raw.lifecycle) % 4096 == 3 * frames
      if continuing:
        track_id, first_seen = old.track_id, old.first_seen_ns
      else:
        track_id, first_seen = self.next_track_id, now_ns
        self.next_track_id += 1
        if old is not None:
          self.counters['lifecycle_restarts'] += 1
      updated[obj.wire_id] = ObjectTrack(track_id, now_ns, first_seen, obj)
    self.tracks = {wire: track for wire, track in updated.items() if wire in active_ids}
    self.last_key, self.last_bank_ns = key, now_ns
    self.fault = False
    self.counters['accepted_banks'] += 1
    return list(self.tracks.values())


class BoschCRadarInterface(RadarInterfaceBase):
  """Explicit research adapter, never selected by the production Honda interface.

  Input is the same timestamped CanData packet list consumed by card. Empty
  updates use the monotonic clock for timeout reporting; replay may supply
  now_nanos explicitly. No messages are sent and CarParams is never modified.
  """
  def __init__(self, CP, CP_SP, *, calibration: CandidateCalibration, bus: int = 1, clock=time.monotonic_ns):
    if CP.carFingerprint != CAR.HONDA_CRV_6G or CP.brand != 'honda':
      raise ValueError('Bosch C research adapter is restricted to HONDA_CRV_6G')
    super().__init__(CP, CP_SP)
    self.calibration = calibration
    self.decoder = BoschCDecoder(bus)
    self.clock = clock
    self.last_output_ns = None
    self.last_error = None
    self.guard_rejected = 0

  def snapshot(self, now_nanos: int):
    self.decoder.expire(now_nanos)
    points = []
    self.guard_rejected = 0
    for track in self.decoder.tracks.values():
      raw = track.raw
      x, y, v = self.calibration.convert(raw)
      # Preserve the offline empirical display guard. This is NOT a recovered
      # validity rule, and quality-container values never gate the baseline.
      if not (raw.x_companion_raw == 0 and 0 < x < 160 and abs(y) < 20 and abs(v) < 90):
        self.guard_rejected += 1
        continue
      points.append(dict(trackId=track.track_id, dRel=x, yRel=y, vRel=v))
    stale = self.decoder.last_bank_ns is None or now_nanos - self.decoder.last_bank_ns > STALE_NS
    return structs.RadarData.new_message(points=points, errors={'canError': stale or self.decoder.fault})

  def update(self, can_packets: list[tuple[int, list[CanData]]], *, now_nanos: int | None = None):
    # Validate the entire batch before mutating state. Preserve frame ordering
    # within equal timestamps; callers must supply monotonic packet timestamps.
    times = [t for t, _ in can_packets]
    now = now_nanos if now_nanos is not None else (times[-1] if times else self.clock())
    previous = self.decoder.now_ns
    for stamp in [*times, now]:
      if not isinstance(stamp, int) or stamp < 0 or (previous is not None and stamp < previous):
        raise ValueError('CAN timestamps must be nonnegative, integer nanoseconds and nondecreasing')
      previous = stamp
    completed = False
    for stamp, frames in can_packets:
      for frame in frames:
        if self.decoder.feed(stamp, frame) is not None:
          completed = True
    result = self.snapshot(now)
    changed = result.errors.canError != self.last_error
    heartbeat = result.errors.canError and (self.last_output_ns is None or now - self.last_output_ns >= 50_000_000)
    if completed or changed or heartbeat:
      self.last_output_ns, self.last_error = now, result.errors.canError
      return result
    return None

  def diagnostics(self):
    now = self.decoder.now_ns
    last = self.decoder.last_bank_ns
    return dict(schema_version=1, experimental=True, platform=str(self.CP.carFingerprint), receive_bus=self.decoder.bus,
                timestamp_ns=now, last_bank_ns=last, bank_age_s=(now-last)*1e-9 if last is not None else None,
                counters=dict(self.decoder.counters), pending_slots=len(self.decoder.pending),
                calibration=asdict(self.calibration), guard_rejected=self.guard_rejected,
                tracks=[dict(track_id=t.track_id, time_ns=t.time_ns, first_seen_ns=t.first_seen_ns, raw=asdict(t.raw))
                        for t in self.decoder.tracks.values()])
