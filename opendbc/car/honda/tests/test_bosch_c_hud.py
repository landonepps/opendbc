import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.honda import bosch_c_radar_live, hud_objects
from opendbc.car.honda.bosch_c_hud import BoschCHud, CAR_TYPE_BY_STATUS
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
