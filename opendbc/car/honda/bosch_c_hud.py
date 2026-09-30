"""Dash objects from the experimental Bosch C radar: decoded vehicles with their radar-reported class icons.

Under openpilot longitudinal on CAN FD the car controller authors HUD_OBJECTS itself (OP's lead in slot 0).
With the experimental Bosch C radar enabled, this supplies the other slots, plus the class of the object matched
to OP's lead, in the same form as the camera HudObject snapshot on radarless cars, so HudObjectAuthor needs no
new logic. Display only; nothing here affects control.
"""
from opendbc.car.honda import bosch_c_radar_live, lane_path
from opendbc.car.honda.hud_objects import MAX_OBJECT_ID, NUM_SLOTS, HudObject, lead_rotation

# Radar status -> dash CAR_TYPE. 7 and -7 are the dash's car and truck icons. 6 is the motorcycle icon, which the
# stock dash draws for moving status-6 objects. Status 6 is often a car on video, so here it is a debugging aid.
CAR_TYPE_BY_STATUS = {1: 7, 3: -7, 6: 6}
MAX_DISTANCE_M = 120.0
# Which objects the CR-V's own radar puts on the dash under stock ACC (bosch-c-research scripts/dash_hud_fit.py): 95% of
# shown objects are the nearest car in their lane, in the ego lane or one lane either side, at most 2-4 at a time. It
# almost never shows oncoming traffic (1 of 61) or objects that have never moved (1 of 63), but does show stopped cars
# that were moving before. Its finer choice among those nearest cars isn't in the radar fields.
MOVING_MPS = 3.0      # over-ground speed our way that counts as moving
MAX_LANE = 1          # ego lane and one lane either side
LEAD_MATCH_Y_M = 1.5
RADAR_TO_CAMERA = 1.52  # m; openpilot radard's model-to-radar distance offset


def lead_gap(lead, obj) -> float:
  return abs(obj.d_rel - (lead.dRel - RADAR_TO_CAMERA))


def lead_match(lead, obj) -> bool:
  # radard's distance sanity window for matching a vision lead to a radar track, plus a lateral bound
  return lead_gap(lead, obj) < max(0.25 * (lead.dRel - RADAR_TO_CAMERA), 5.0) and abs(obj.y_rel - lead.yRel) <= LEAD_MATCH_Y_M


class BoschCHud:
  def __init__(self):
    self._ids: dict[int, int] = {}  # radar track id -> dash OBJECT_ID (1..31), stable while the track lives
    self._moved: set[int] = set()   # live radar track ids that have moved our way

  def _object_id(self, track_id: int, used: set[int]) -> int:
    oid = self._ids.get(track_id)
    if oid is None:
      oid = next((i for i in range(1, MAX_OBJECT_ID + 1) if i not in used), 0)
      self._ids[track_id] = oid
    return oid

  def tracks(self, lead, now_s: float | None = None, dash_lane=None, v_ego: float | None = None) -> list[HudObject] | None:
    """Ten slot-indexed HudObjects, or None when the live radar isn't publishing (the author then keeps its
    model-lead extras). The object matching OP's lead goes in slot 0 flagged is_lead_car, so the author borrows its
    id and class icon for OP's lead and never renders it twice. The others are the nearest object in each of the ego
    and adjacent lanes, placed in their lanes relative to `dash_lane` (lane_path.lane_position); with `v_ego`, only
    objects that are moving our way or have been."""
    objects = bosch_c_radar_live.latest_display(now_s)
    if objects is None:
      return None
    if v_ego is not None:
      self._moved |= {o.track_id for o in objects if o.v_rel + v_ego > MOVING_MPS}
      self._moved &= {o.track_id for o in objects}
      objects = [o for o in objects if o.track_id in self._moved]
    placed = {}  # object -> (lane, dash lateral)
    for o in objects:
      if o.d_rel <= MAX_DISTANCE_M:
        lane, y_dash = lane_path.lane_position(dash_lane, o.d_rel + RADAR_TO_CAMERA, o.y_rel)
        if abs(lane) <= MAX_LANE:
          placed[o] = (lane, y_dash)
    near = sorted(placed, key=lambda o: o.d_rel)
    lead_obj = None
    if lead.status:
      matches = [o for o in near if lead_match(lead, o)]
      lead_obj = min(matches, key=lambda o: lead_gap(lead, o)) if matches else None
    others, lanes = [], set()
    for o in near:
      lane = placed[o][0]
      hidden = lead_obj is not None and lane == placed[lead_obj][0] and o.d_rel > lead_obj.d_rel
      if o is not lead_obj and lane not in lanes and not hidden:
        others.append(o)
        lanes.add(lane)

    live = {o.track_id for o in ([lead_obj] if lead_obj else []) + others}
    self._ids = {k: v for k, v in self._ids.items() if k in live}
    used = set(self._ids.values())
    out = [HudObject(slot=i, object_id=0, d_rel=0.0, y_rel=0.0, is_lead_car=False, valid=False) for i in range(NUM_SLOTS)]
    for slot, obj in ([(0, lead_obj)] if lead_obj else []) + list(enumerate(others, start=1)):
      oid = self._object_id(obj.track_id, used)
      used.add(oid)
      out[slot] = HudObject(slot=slot, object_id=oid, d_rel=obj.d_rel, y_rel=placed[obj][1], is_lead_car=slot == 0, valid=True,
                            car_type=CAR_TYPE_BY_STATUS[obj.status], rotation=lead_rotation(obj.y_rel))
    return out
