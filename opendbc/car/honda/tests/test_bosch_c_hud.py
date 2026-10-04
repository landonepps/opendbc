import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.honda import bosch_c_radar_live, hud_objects
from opendbc.car.honda.bosch_c_hud import BoschCHud, CAR_TYPE_BY_STATUS, HANDOFF_S
from opendbc.car.honda.bosch_c_radar import DisplayObject


@pytest.fixture(autouse=True)
def reset_display():
  bosch_c_radar_live._display = None
  yield
  bosch_c_radar_live._display = None


def lead(d, y=0.0, status=True):
  return hud_objects.ModelLead(status, d, y, 0.0, prob=0.9 if status else 0.0)


def publish(*objects, now=100.0):
  bosch_c_radar_live.publish_display(list(objects), now)


def test_no_live_radar_keeps_model_extras():
  assert BoschCHud().tracks(lead(30), now_s=100.0) is None
  publish(DisplayObject(1, 30, 0, 0, 1))
  assert BoschCHud().tracks(lead(30), now_s=100.0 + bosch_c_radar_live.DISPLAY_MAX_AGE_S + .01) is None


def test_lead_gets_radar_class_and_others_fill_by_distance():
  publish(DisplayObject(1, 40, 3.5, 0, 1), DisplayObject(2, 30.5, 0.2, 0, 3), DisplayObject(3, 20, -3.4, 0, 6),
          DisplayObject(4, 25, 9.0, 0, 1))  # beyond the adjacent lanes
  tracks = BoschCHud().tracks(lead(30), now_s=100.0)
  assert len(tracks) == hud_objects.NUM_SLOTS
  assert tracks[0].valid and tracks[0].is_lead_car and tracks[0].car_type == CAR_TYPE_BY_STATUS[3]
  assert [(t.car_type, t.is_lead_car) for t in tracks[1:3]] == [(6, False), (7, False)]
  assert tracks[1].d_rel == 20 and tracks[1].y_rel < 0 < tracks[2].y_rel
  assert not any(t.valid for t in tracks[3:])
  ids = [t.object_id for t in tracks[:3]]
  assert len(set(ids)) == 3 and all(1 <= i <= 31 for i in ids)


def test_object_ids_stable_while_tracks_live():
  hud = BoschCHud()
  publish(DisplayObject(7, 50, 3.5, 0, 1), DisplayObject(8, 60, -3.5, 0, 1))
  first = {t.d_rel: t.object_id for t in hud.tracks(lead(0, status=False), now_s=100.0) if t.valid}
  publish(DisplayObject(8, 59, -3.5, 0, 1), DisplayObject(9, 45, 0, 0, 1), DisplayObject(7, 49, 3.5, 0, 1))
  second = {t.d_rel: t.object_id for t in hud.tracks(lead(0, status=False), now_s=100.0) if t.valid}
  assert second[49] == first[50] and second[59] == first[60] and second[45] not in (first[50], first[60])


def test_author_draws_radar_class_for_ops_lead():
  packer = CANPacker('honda_common_canfd_generated')
  parser = CANParser('honda_common_canfd_generated', [('HUD_OBJECTS_ALT', float('nan'))], 0)
  publish(DisplayObject(1, 30.5, 0.2, 0, 6))
  tracks = BoschCHud().tracks(lead(30), now_s=100.0)
  author = hud_objects.HudObjectAuthor()
  addr, dat, bus = author.create(packer, 0, lead(30), tracks, 1, 100.0, canfd=True, name='HUD_OBJECTS_ALT')
  parser.update([(0, [(addr, dat, bus)])])
  assert parser.vl['HUD_OBJECTS_ALT']['IS_LEAD_CAR'] == 1 and parser.vl['HUD_OBJECTS_ALT']['CAR_TYPE'] == 6


def test_oncoming_parked_and_crossing_objects_are_not_shown():
  # at 20 m/s: oncoming (-20 over ground), never moved (0), and a car moving our way (18)
  publish(DisplayObject(1, 30, 3.4, -40.0, 1), DisplayObject(2, 20, -3.2, -20.0, 1), DisplayObject(3, 40, 0.1, -2.0, 1))
  tracks = BoschCHud().tracks(lead(0, status=False), now_s=100.0, v_ego=20.0)
  assert [t.d_rel for t in tracks if t.valid] == [40]


def test_stopped_car_that_was_moving_stays_shown():
  hud = BoschCHud()
  publish(DisplayObject(5, 30, -3.3, -5.0, 1))
  assert [t.d_rel for t in hud.tracks(lead(0, status=False), now_s=100.0, v_ego=20.0) if t.valid] == [30]
  publish(DisplayObject(5, 25, -3.3, -20.0, 1))  # now stopped
  assert [t.d_rel for t in hud.tracks(lead(0, status=False), now_s=100.0, v_ego=20.0) if t.valid] == [25]


def test_every_car_in_the_ego_and_adjacent_lanes():
  publish(DisplayObject(1, 20, 3.3, 0, 1), DisplayObject(2, 35, 3.1, 0, 1), DisplayObject(3, 25, 6.4, 0, 1),
          DisplayObject(4, 30.5, 0.1, 0, 1), DisplayObject(5, 50, 0.3, 0, 1))
  tracks = BoschCHud().tracks(lead(32), now_s=100.0)
  assert tracks[0].is_lead_car and tracks[0].d_rel == 30.5
  # left lane: both cars, nearest first; two lanes over: none; ego lane: the car behind OP's lead is hidden
  assert [t.d_rel for t in tracks[1:] if t.valid] == [20, 35]


def shown(hud, now, **kwargs):
  return {t.object_id: t.d_rel for t in hud.tracks(lead(0, status=False), now_s=now, **kwargs) if t.valid}


def test_an_id_that_left_one_car_is_not_given_to_the_next():
  hud = BoschCHud()
  publish(DisplayObject(1, 3, -3.3, 0, 1), DisplayObject(2, 40, -3.3, 0, 1))
  first = shown(hud, 100.0)
  publish(DisplayObject(2, 40, -3.3, 0, 1), DisplayObject(3, 70, 3.3, 0, 1), now=102.0)
  second = shown(hud, 102.0)
  near_id = next(i for i, d in first.items() if d == 3)
  assert near_id not in second and len(second) == 2


def test_a_new_track_takes_the_id_of_the_car_the_radar_dropped():
  hud = BoschCHud()
  publish(DisplayObject(1, 30, -3.3, -2.0, 1))
  [car] = shown(hud, 100.0)
  publish(now=100.2)  # dropped: the car stays drawn where its speed takes it
  assert shown(hud, 100.2) == {car: pytest.approx(29.6)}
  publish(DisplayObject(2, 29.0, -3.4, -2.0, 1), now=100.4)  # re-created
  assert shown(hud, 100.4) == {car: 29.0}
  publish(DisplayObject(2, 28.6, -3.4, -2.0, 1), now=100.6)
  assert shown(hud, 100.6) == {car: 28.6}


def test_a_dropped_car_goes_after_the_hold_or_once_behind_us():
  hud = BoschCHud()
  publish(DisplayObject(1, 30, -3.3, 0, 1), DisplayObject(2, 1.0, 3.3, -5.0, 1))
  assert len(shown(hud, 100.0)) == 2
  publish(now=100.3)
  assert list(shown(hud, 100.3).values()) == [30]  # the car beside us has passed behind
  publish(now=100.0 + HANDOFF_S + .05)
  assert shown(hud, 100.0 + HANDOFF_S + .05) == {}


def test_a_new_track_drawn_a_lane_off_takes_over_once_in_the_lane():
  hud = BoschCHud()
  publish(DisplayObject(1, 10, -3.3, 0, 1))
  [car] = shown(hud, 100.0)
  publish(DisplayObject(2, 10.5, -5.4, 0, 1), now=100.2)  # the new track's first lateral is two lanes over
  assert shown(hud, 100.2) == {car: 10}
  publish(DisplayObject(2, 10.5, -4.0, 0, 1), now=100.3)
  assert shown(hud, 100.3) == {car: 10.5}


def test_a_new_track_beside_the_dropped_car_replaces_it():
  hud = BoschCHud()
  publish(DisplayObject(1, 20, 3.3, 0, 1))
  shown(hud, 100.0)
  publish(DisplayObject(1, 20, 3.3, 0, 1), DisplayObject(2, 21, 3.2, 0, 1), now=100.1)  # started before 1 ended
  ids = shown(hud, 100.1)
  publish(DisplayObject(2, 21, 3.2, 0, 1), now=100.2)
  assert list(shown(hud, 100.2).values()) == [21] and set(ids) >= set(shown(hud, 100.2))


def test_the_authors_lead_id_is_not_given_to_another_car():
  publish(DisplayObject(1, 40, 3.3, 0, 1))
  assert list(shown(BoschCHud(), 100.0, lead_id=1)) != [1]


def test_objects_are_placed_in_lanes_relative_to_the_drawn_lane():
  from opendbc.car.honda import lane_path
  from opendbc.car.honda.tests.test_lane_path import model_at
  curve = lane_path.LanePathFitter().update(model_at(-2.0), 30.0, 0.0, canfd=True, scale=lane_path.CRV6G_SCALE)
  publish(DisplayObject(1, 50, 2.1, 0, 1), DisplayObject(2, 45, 5.3, 0, 1))
  tracks = BoschCHud().tracks(lead(0, status=False), now_s=100.0, dash_lane=curve)
  placed = {t.d_rel: t.y_rel for t in tracks if t.valid}
  assert abs(placed[50] - 0.05) < 1e-6 and abs(placed[45] - 3.15) < 1e-6
