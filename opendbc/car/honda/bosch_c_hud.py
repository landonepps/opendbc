"""Dash objects from the experimental Bosch C radar: decoded vehicles with their radar-reported class icons.

Under openpilot longitudinal on CAN FD the car controller authors HUD_OBJECTS itself (OP's lead in slot 0).
With the experimental Bosch C radar enabled, this supplies the other slots, plus the class of the object matched
to OP's lead, in the same form as the camera HudObject snapshot on radarless cars, so HudObjectAuthor needs no
new logic. Display only; nothing here affects control.
"""
from opendbc.car.honda import bosch_c_radar_live, lane_path
from opendbc.car.honda.hud_objects import LAT_SCALE, MAX_OBJECT_ID, NUM_SLOTS, HudObject, lead_rotation

# Radar status -> dash CAR_TYPE. 7 and -7 are the dash's car and truck icons. 6 is the motorcycle icon, which the
# stock dash draws for moving status-6 objects. Status 6 is often a car on video, so here it is a debugging aid.
CAR_TYPE_BY_STATUS = {1: 7, 3: -7, 6: 6}
MAX_LATERAL_M = 5.5   # the ego lane and one lane either side
MAX_DISTANCE_M = 120.0
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

  def _object_id(self, track_id: int, used: set[int]) -> int:
    oid = self._ids.get(track_id)
    if oid is None:
      oid = next((i for i in range(1, MAX_OBJECT_ID + 1) if i not in used), 0)
      self._ids[track_id] = oid
    return oid

  def tracks(self, lead, now_s: float | None = None) -> list[HudObject] | None:
    """Ten slot-indexed HudObjects, or None when the live radar isn't publishing (the author then keeps its
    model-lead extras). The object matching OP's lead goes in slot 0 flagged is_lead_car, so the author borrows its
    id and class icon for OP's lead and never renders it twice."""
    objects = bosch_c_radar_live.latest_display(now_s)
    if objects is None:
      return None
    near = sorted((o for o in objects if abs(o.y_rel) <= MAX_LATERAL_M and o.d_rel <= MAX_DISTANCE_M), key=lambda o: o.d_rel)
    lead_obj = None
    if lead.status:
      matches = [o for o in near if lead_match(lead, o)]
      lead_obj = min(matches, key=lambda o: lead_gap(lead, o)) if matches else None
    others = [o for o in near if o is not lead_obj][:NUM_SLOTS - 1]

    live = {o.track_id for o in ([lead_obj] if lead_obj else []) + others}
    self._ids = {k: v for k, v in self._ids.items() if k in live}
    used = set(self._ids.values())
    out = [HudObject(slot=i, object_id=0, d_rel=0.0, y_rel=0.0, is_lead_car=False, valid=False) for i in range(NUM_SLOTS)]
    for slot, obj in ([(0, lead_obj)] if lead_obj else []) + list(enumerate(others, start=1)):
      oid = self._object_id(obj.track_id, used)
      used.add(oid)
      # dash lateral uses the same lane-gain scaling as OP's lead so objects sit on the rendered lanes
      y_dash = LAT_SCALE * lane_path.curve_boost(obj.d_rel) * obj.y_rel
      out[slot] = HudObject(slot=slot, object_id=oid, d_rel=obj.d_rel, y_rel=y_dash, is_lead_car=slot == 0, valid=True,
                            car_type=CAR_TYPE_BY_STATUS[obj.status], rotation=lead_rotation(obj.y_rel))
    return out
