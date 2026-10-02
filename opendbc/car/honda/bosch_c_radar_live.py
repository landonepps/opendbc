"""Default-off Bosch C integration for the tested CR-V and receive-bus layout.

Distance, velocity and lateral position use the radar firmware's own decode
(0.05 m, 0.01 m and 0.1 m/s per count; lateral zero 8191, velocity zero 1540,
since 2026-10-02; bosch-c-research docs/radar-firmware-a230.md). The -4.296 m
range offset is the measured offset to the bumper reference: the firmware reads
range with no offset, in the camera's frame. This module sends no CAN and
changes no safety, silencing, fusion or controller algorithms. Activation is
read at startup.
"""
import time

from opendbc.car import structs
from opendbc.car.honda.bosch_c_radar import BoschCRadarInterface, CandidateCalibration, STALE_NS
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.honda.values import CAR, HondaFlags
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP


PROVISIONAL_CALIBRATION = CandidateCalibration(.05, 0, -4.296000000000001, .01, .1, 1540)


def supported(CP):
  return (CP.brand == 'honda' and CP.carFingerprint == CAR.HONDA_CRV_6G and
          bool(CP.flags & HondaFlags.BOSCH_CANFD) and CP.openpilotLongitudinalControl and CanBus(CP).radar == 1)


def configure(CP, CP_SP, enabled, uncertainty_gate=False):
  """Called after stock-longitudinal overrides; never enable longitudinal here."""
  if enabled and supported(CP):
    CP_SP.flags |= HondaFlagsSP.EXPERIMENTAL_BOSCH_C_RADAR.value
    CP.radarUnavailable = False
    if uncertainty_gate:
      CP_SP.flags |= HondaFlagsSP.BOSCH_C_UNCERTAINTY_GATE.value


class BoschCLiveRadarInterface(BoschCRadarInterface):
  def __init__(self, CP, CP_SP, **kwargs):
    if not supported(CP) or CP.radarUnavailable or not (CP_SP.flags & HondaFlagsSP.EXPERIMENTAL_BOSCH_C_RADAR):
      raise ValueError('Bosch C live radar requires explicit supported enabled-mode configuration')
    kwargs.setdefault('uncertainty_gate', bool(CP_SP.flags & HondaFlagsSP.BOSCH_C_UNCERTAINTY_GATE))
    super().__init__(CP, CP_SP, calibration=PROVISIONAL_CALIBRATION, bus=1, **kwargs)

  def update(self, can_packets):
    # A delayed card receive batch can contain valid but already stale banks.
    # Check source ages against boot time, not only their recorded timestamps.
    now = self.clock()
    watermark = self.decoder.now_ns
    timely = []
    for stamp, frames in can_packets:
      if stamp > now:
        # Wrong clock domain must not create apparently fresh points.
        return structs.RadarData.new_message(errors={'canError': True})
      if now - stamp <= STALE_NS and (watermark is None or stamp >= watermark):
        timely.append((stamp, frames))
        watermark = stamp
      else:
        self.decoder.counters['live_discarded_packets'] += 1

    # Do not advance the decoder to wall time on every healthy call: CAN can
    # arrive just behind that watermark. Advance only to expire a stale bank.
    result = super().update(timely) if timely else None
    now = self.clock()
    last_bank = self.decoder.last_bank_ns
    if last_bank is None and timely:
      return result
    if last_bank is None or now - last_bank > STALE_NS:
      return super().update([], now_nanos=now)
    if result is not None:
      fresh_ids = {t.track_id for t in self.decoder.tracks.values() if now - t.time_ns <= STALE_NS}
      result.points = [p.to_dict() for p in result.points if p.trackId in fresh_ids]
      publish_display(self.display_objects(now))
    return result


# The dash author (carcontroller, same card process) reads the latest decoded vehicles from here. Display only.
DISPLAY_MAX_AGE_S = 0.3
_display: tuple[float, list] | None = None


def publish_display(objects, now_s: float | None = None):
  global _display
  _display = (time.monotonic() if now_s is None else now_s, objects)


def latest_display(now_s: float | None = None):
  """Latest decoded vehicles, or None when the live radar isn't publishing (then the dash keeps model leads)."""
  if _display is None:
    return None
  stamp, objects = _display
  now = time.monotonic() if now_s is None else now_s
  return objects if now - stamp <= DISPLAY_MAX_AGE_S else None
