"""Dash objects from the experimental Bosch C radar: decoded vehicles with their radar-reported class icons.

Under openpilot longitudinal on CAN FD the car controller authors HUD_OBJECTS itself (OP's lead in slot 0).
With the experimental Bosch C radar enabled, this supplies the other slots, plus the class of the object matched
to OP's lead, in the same form as the camera HudObject snapshot on radarless cars, so HudObjectAuthor needs no
new logic. Display only; nothing here affects control.
"""
import time
from dataclasses import replace

from opendbc.car.honda import bosch_c_radar_live, lane_path
from opendbc.car.honda.bosch_c_radar import DisplayObject
from opendbc.car.honda.hud_objects import MAX_OBJECT_ID, NUM_SLOTS, HudObject, lead_rotation

# Radar status -> dash CAR_TYPE. 7 and -7 are the dash's car and truck icons. 6 is the motorcycle icon, which the
# stock dash draws for moving status-6 objects. Status 6 is often a car on video, so here it is a debugging aid.
CAR_TYPE_BY_STATUS = {1: 7, 3: -7, 6: 6}
MAX_DISTANCE_M = 120.0
# Which objects the CR-V's own radar puts on the dash under stock ACC (bosch-c-research scripts/dash_hud_fit.py): cars in
# the ego lane or one lane either side, up to 5 at a time, with 2 in one lane 14% of the time. It almost never shows
# oncoming traffic (1 of 61) or objects that have never moved (1 of 63), but does show stopped cars that were moving
# before. 95% of what it shows is the nearest car in its lane, but showing only that car made a lane's dash car slide
# between a car beside us and one ahead each time the near one dropped out. Every car in those lanes is shown instead.
MOVING_MPS = 3.0      # over-ground speed our way that counts as moving
MAX_LANE = 1          # ego lane and one lane either side
LEAD_MATCH_Y_M = 1.5
RADAR_TO_CAMERA = 1.52  # m; openpilot radard's model-to-radar distance offset
# The radar drops and re-creates its track for a car about 6 times a minute in traffic (routes 1bf, 1ce, 1d3, 1db): the
# new track came a median 0.18 s later and 1 m away, and within 0.9 s for 99%. A shown car the radar drops stays drawn
# for HANDOFF_S where its speed takes it, until it passes behind us, and the first new track drawn there takes its
# OBJECT_ID, so the dash keeps one car instead of dropping it and drawing another. A car that leaves the shown lanes for
# less than HANDOFF_S gets its OBJECT_ID back.
HANDOFF_S = 1.0
HANDOFF_D_M = 5.0
HANDOFF_Y_M = 1.5


def lead_gap(lead, obj) -> float:
  return abs(obj.d_rel - (lead.dRel - RADAR_TO_CAMERA))


def lead_match(lead, obj) -> bool:
  # radard's distance sanity window for matching a vision lead to a radar track, plus a lateral bound
  return lead_gap(lead, obj) < max(0.25 * (lead.dRel - RADAR_TO_CAMERA), 5.0) and abs(obj.y_rel - lead.yRel) <= LEAD_MATCH_Y_M


class BoschCHud:
  def __init__(self):
    self._ids: dict[int, int] = {}       # radar track id -> dash OBJECT_ID (1..31), kept for HANDOFF_S after it's shown
    self._shown_s: dict[int, float] = {}  # radar track id -> when it was last shown
    self._held: dict[int, tuple[DisplayObject, float]] = {}   # shown track the radar dropped -> (its last DisplayObject, when)
    self._last: dict[int, DisplayObject] = {}  # live tracks shown at the last update
    self._last_s = 0.0
    self._moved: set[int] = set()         # live radar track ids that have moved our way
    self._last_id = 0                     # the last OBJECT_ID given out

  def _object_id(self, track_id: int, used: set[int]) -> int:
    oid = self._ids.get(track_id)
    if oid is None:
      # the next free id after the last one given out: an id that just left one car would draw as that car sliding to
      # the next one
      oid = next((i for k in range(MAX_OBJECT_ID) if (i := (self._last_id + k) % MAX_OBJECT_ID + 1) not in used), 0)
      self._last_id = oid or self._last_id
      self._ids[track_id] = oid
    return oid

  def _held_cars(self, objects, now: float) -> list[DisplayObject]:
    """Shown cars the radar dropped in the last HANDOFF_S, moved on by their speed, until they pass behind us."""
    live = {o.track_id for o in objects}
    for tid, o in self._last.items():
      if tid not in live and tid not in self._held:
        self._held[tid] = (o, self._last_s)
    self._held = {tid: (o, t) for tid, (o, t) in self._held.items()
                  if tid not in live and now - t <= HANDOFF_S and o.d_rel + o.v_rel * (now - t) > 0}
    return [replace(o, d_rel=o.d_rel + o.v_rel * (now - t)) for o, t in self._held.values()]

  def _hand_off(self, placed: dict) -> None:
    """Give a held car's OBJECT_ID to a new track drawn in its place, and stop drawing a held car where a track that
    started before it ended is already drawn. Only a drawn track takes over, since a new track's first lateral can be
    a lane off."""
    held = [o for o in placed if o.track_id in self._held]
    for o in [o for o in placed if o.track_id not in self._held]:
      near = [h for h in held if abs(h.d_rel - o.d_rel) < HANDOFF_D_M and abs(h.y_rel - o.y_rel) < HANDOFF_Y_M]
      if near:
        h = min(near, key=lambda h: abs(h.d_rel - o.d_rel))
        if o.track_id not in self._ids and h.track_id in self._ids:
          self._ids[o.track_id] = self._ids.pop(h.track_id)
        held.remove(h)
        del placed[h], self._held[h.track_id]

  def tracks(self, lead, now_s: float | None = None, dash_lane=None, v_ego: float | None = None, lead_id: int = 0) -> list[HudObject] | None:
    """Ten slot-indexed HudObjects, or None when the live radar isn't publishing (the author then keeps its
    model-lead extras). The object matching OP's lead goes in slot 0 flagged is_lead_car, so the author borrows its
    id and class icon for OP's lead and never renders it twice. The others are the cars in the ego and adjacent lanes,
    nearest first, placed in their lanes relative to `dash_lane` (lane_path.lane_position); with `v_ego`, only
    objects that are moving our way or have been. `lead_id` is the OBJECT_ID the author shows OP's lead with, which
    no other car is given."""
    objects = bosch_c_radar_live.latest_display(now_s)
    if objects is None:
      return None
    now = time.monotonic() if now_s is None else now_s
    if v_ego is not None:
      self._moved |= {o.track_id for o in objects if o.v_rel + v_ego > MOVING_MPS}
      self._moved &= {o.track_id for o in objects}
      objects = [o for o in objects if o.track_id in self._moved]
    placed = {}  # object -> (lane, dash lateral)
    for o in objects + self._held_cars(objects, now):
      if o.d_rel <= MAX_DISTANCE_M:
        lane, y_dash = lane_path.lane_position(dash_lane, o.d_rel + RADAR_TO_CAMERA, o.y_rel)
        if abs(lane) <= MAX_LANE:
          placed[o] = (lane, y_dash)
    self._hand_off(placed)
    near = sorted(placed, key=lambda o: o.d_rel)
    lead_obj = None
    if lead.status:
      matches = [o for o in near if lead_match(lead, o)]
      lead_obj = min(matches, key=lambda o: lead_gap(lead, o)) if matches else None
    others = []
    for o in near:
      hidden = lead_obj is not None and placed[o][0] == placed[lead_obj][0] and o.d_rel > lead_obj.d_rel
      if o is not lead_obj and not hidden and len(others) < NUM_SLOTS - 1:
        others.append(o)

    shown = ([lead_obj] if lead_obj else []) + others
    held = set(self._held)
    self._shown_s.update({o.track_id: now for o in shown if o.track_id not in held})
    self._shown_s = {k: t for k, t in self._shown_s.items() if now - t <= HANDOFF_S}
    self._ids = {k: v for k, v in self._ids.items() if k in self._shown_s}
    self._last = {o.track_id: o for o in shown if o.track_id not in held}
    self._last_s = now
    used = set(self._ids.values()) | {lead_id}
    out = [HudObject(slot=i, object_id=0, d_rel=0.0, y_rel=0.0, is_lead_car=False, valid=False) for i in range(NUM_SLOTS)]
    for slot, obj in ([(0, lead_obj)] if lead_obj else []) + list(enumerate(others, start=1)):
      oid = self._object_id(obj.track_id, used)
      used.add(oid)
      out[slot] = HudObject(slot=slot, object_id=oid, d_rel=obj.d_rel, y_rel=placed[obj][1], is_lead_car=slot == 0, valid=True,
                            car_type=CAR_TYPE_BY_STATUS[obj.status], rotation=lead_rotation(obj.y_rel))
    return out
