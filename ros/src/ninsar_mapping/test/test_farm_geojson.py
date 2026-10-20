"""Checks the farm GeoJSON library: frame, io, ids, derived structure, rules G1-G16, generators."""

import copy
import json
import os
import sys

import numpy as np
import pytest
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import farm_geojson as fg  # noqa: E402

MINI_FARM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "mini_farm.geojson")


@pytest.fixture
def farm():
    return fg.load(MINI_FARM)


def by_id(fc, fid):
    return next(f for f in fc["features"] if f["id"] == fid)


def add(fc, geom, **props):
    """Append a feature given in ENU metres; returns its id."""
    spec = fg.default_schema().type_of(props)
    fid = fg.new_id(fc, spec["prefix"] if spec else "x")
    fc["features"].append({"type": "Feature", "id": fid, "properties": props,
                           "geometry": fg.to_geojson(geom, fg.frame_of(fc))})
    return fid


def rules(fc, level="error"):
    return {(i.rule, i.id) for i in fg.validate(fc) if i.level == level}


# --- frame -------------------------------------------------------------------------------

def test_enu_round_trip_below_1mm():
    frame = fg.Frame(46.3374, 3.434, 250.0)
    xy = np.random.default_rng(0).uniform(-2000, 2000, size=(500, 2))
    back = frame.enu(frame.wgs84(xy))
    assert np.abs(back - xy).max() < 1e-3


def test_enu_axes_point_east_and_north():
    frame = fg.Frame(46.3374, 3.434)
    east, north = frame.enu([[3.435, 46.3374], [3.434, 46.3384]])
    assert east[0] == pytest.approx(76.9, abs=0.2) and abs(east[1]) < 0.01
    assert north[1] == pytest.approx(111.2, abs=0.2) and abs(north[0]) < 1e-6


# --- io and ids --------------------------------------------------------------------------

def test_mini_farm_is_clean(farm):
    assert fg.validate(farm) == []


def test_save_load_round_trip(farm, tmp_path):
    path = str(tmp_path / "farm.geojson")
    fg.save(path, farm)
    again = fg.load(path)
    assert again["features"] == farm["features"]
    assert again["origin"] == farm["origin"] and again["next_id"] == farm["next_id"]
    fg.save(path, again)
    assert os.path.exists(path + ".bak")
    lines = open(path).read().splitlines()
    assert sum(line.startswith('    {"type": "Feature"') for line in lines) == len(farm["features"])


def test_save_sets_origin_from_farm_once(tmp_path):
    fc = fg.new_collection("t")
    fc["features"].append({"type": "Feature", "id": "F0", "properties": {"layer": "farm", "kind": "farm", "name": "t"},
                           "geometry": {"type": "Polygon", "coordinates": [[[3, 46], [3.01, 46], [3.01, 46.01],
                                                                            [3, 46.01], [3, 46]]]}})
    saved = fg.save(str(tmp_path / "t.geojson"), fc)
    assert saved["origin"] == {"lat": 46.005, "lon": 3.005, "alt": 0.0}
    saved["features"][0]["geometry"]["coordinates"][0][1][0] = 3.05
    assert fg.save(str(tmp_path / "t.geojson"), saved)["origin"]["lon"] == 3.005


def test_new_id_never_reuses_deleted(farm):
    last = fg.new_id(farm, "c")
    farm["features"] = [f for f in farm["features"] if f["id"] != last]
    assert fg.parse_id(fg.new_id(farm, "c"))[1] == fg.parse_id(last)[1] + 1


def test_load_without_next_id_recomputes(tmp_path, farm):
    del farm["next_id"]
    path = tmp_path / "f.geojson"
    path.write_text(json.dumps(farm))
    assert fg.load(str(path))["next_id"] == fg.used_counters(farm)


# --- derived structure (§6) ---------------------------------------------------------------

def test_derived_tree(farm):
    d = fg.derive(farm)
    assert d["A0"]["parent"] == "F0" and d["r0"]["parent"] == "A0"
    assert d["A0"]["n_rows"] == 8 and d["A0"]["crops"] == ["grapevine"] and d["A0"]["heading_deg"] == 0.0
    assert [d[f"r{i}"]["row_index"] for i in range(8)] == list(range(8))
    plants = [k for k in d["r0"]["children"] if by_id(farm, k)["properties"]["kind"] == "grapevine"]
    assert sorted(d[k]["row_index"] for k in plants) == list(range(17)) == [d[k]["row_index"] for k in plants]
    barn = next(f["id"] for f in farm["features"] if f["properties"].get("name") == "Barn")
    valve = next(f["id"] for f in farm["features"] if f["properties"]["kind"] == "valve")
    assert d[barn]["parent"] == "A3" and d[valve]["parent"] == "F0"


def test_rotated_parcel_heading(farm):
    d = fg.derive(farm)
    assert d["A2"]["heading_deg"] == pytest.approx(30.0, abs=0.01)
    assert all(d[f"r{i}"]["heading_deg"] == pytest.approx(30.0, abs=0.01) for i in range(18, 24))


def test_overlay_membership_highest_node_and_inheritance(farm):
    d = fg.derive(farm)
    assert d["Z0"]["members"] == ["A0"]
    assert d["Z3"]["members"] == ["r13", "r14", "r15", "r16", "r17"]
    plant = d["r15"]["children"][0]
    assert d[plant]["overlays"] == ["Z1", "Z3"]
    assert d["r8"]["overlays"] == ["Z1"]
    assert d["Z2"]["members"] == ["A4"] and d["Z0"]["roads"] == []


def test_nearest_axis_wins_between_strips(farm):
    d = fg.derive(farm)
    by_id(farm, "A0")["properties"]["row_spacing_m"] = 4.0  # strips 2 m wide each side: they overlap
    r0, r1 = (fg.to_enu(by_id(farm, r)["geometry"], fg.frame_of(farm)) for r in ("r0", "r1"))
    y0, y1 = r0.coords[0][1], r1.coords[0][1]
    near_r1 = add(farm, Point(-70, y0 + 0.7 * (y1 - y0)), layer="object", category="equipment", kind="soil_sensor")
    assert fg.derive(farm)[near_r1]["parent"] == "r1"
    assert d["r0"]["n_plants"] == 17


# --- rules (§5) ----------------------------------------------------------------------------

def _second_farm(fc):
    add(fc, box(-10, -10, 10, 10), layer="farm", kind="farm", name="x")


def _no_farm(fc):
    fc["features"] = [f for f in fc["features"] if f["id"] != "F0"]


def _duplicate_id(fc):
    fc["features"].append(copy.deepcopy(by_id(fc, "e1")))


def _wrong_prefix(fc):
    by_id(fc, "r0")["id"] = "A900"
    fc["next_id"]["A"] = 1000


def _counter_too_high(fc):
    by_id(fc, "e1")["id"] = f"e{fc['next_id']['e'] + 5}"


def _unknown_kind(fc):
    by_id(fc, "e1")["properties"]["kind"] = "robot"


def _wrong_geometry(fc):
    add(fc, LineString([(0, 0), (5, 5)]), layer="area", kind="yard")


def _bowtie(fc):
    add(fc, Polygon([(60, -50), (70, -40), (70, -50), (60, -40)]), layer="area", kind="yard")


def _degenerate_line(fc):
    add(fc, LineString([(1, 1), (1, 1)]), layer="navigation", kind="road")


def _missing_required(fc):
    del by_id(fc, "A4")["properties"]["water_type"]


def _bad_enum(fc):
    by_id(fc, "Z2")["properties"]["restriction"] = "forbidden"


def _negative_length(fc):
    by_id(fc, "A0")["properties"]["row_spacing_m"] = -2


def _reserved_key(fc):
    by_id(fc, "e1")["properties"]["parent"] = "F0"


def _outside_farm(fc):
    add(fc, Point(150, 0), layer="object", category="obstacle", kind="rock")


def _overlapping_areas(fc):
    add(fc, box(-60, -45, -45, -35), layer="area", kind="yard")


def _row_in_yard(fc):
    add(fc, LineString([(25, 45), (80, 45)]), layer="row", kind="cabbage")


def _row_outside_areas(fc):
    add(fc, LineString([(-30, 10), (-10, 10)]), layer="row", kind="cabbage")


def _crop_off_row(fc):
    add(fc, Point(-5, 10), layer="object", category="crop", kind="cabbage")


def _crop_on_wrong_row(fc):
    add(fc, Point(-38, -40), layer="object", category="crop", kind="lemon")


def _building_in_row_strip(fc):
    add(fc, box(-38.3, -40.3, -37.7, -39.7), layer="object", category="structure", kind="shed")


def _crossing_rows(fc):
    add(fc, LineString([(-39, -45), (-1, -35)]), layer="row", kind="cabbage")


def _overlapping_sectors(fc):
    add(fc, box(-60, -50, -30, -30), layer="overlay", kind="irrigation_sector", valve="e0")


def _overlapping_same_scheme(fc):
    add(fc, box(30, -40, 70, -10), layer="overlay", kind="management_zone", scheme="vigor_2026", value="low")


def _missing_valve(fc):
    by_id(fc, "Z0")["properties"]["valve"] = "e99"


def _valve_not_a_valve(fc):
    by_id(fc, "Z0")["properties"]["valve"] = "e1"


RULE_CASES = [
    ("G1", None, _second_farm), ("G1", None, _no_farm),
    ("G2", "e1", _duplicate_id), ("G2", "A900", _wrong_prefix), ("G2", None, _counter_too_high),
    ("G3", "e1", _unknown_kind), ("G3", None, _wrong_geometry),
    ("G4", None, _bowtie), ("G4", None, _degenerate_line),
    ("G5", "A4", _missing_required), ("G5", "Z2", _bad_enum), ("G5", "A0", _negative_length),
    ("G6", "e1", _reserved_key),
    ("G7", None, _outside_farm),
    ("G8", None, _overlapping_areas),
    ("G9", None, _row_in_yard), ("G9", None, _row_outside_areas),
    ("G10", None, _crop_off_row), ("G10", None, _crop_on_wrong_row),
    ("G11", None, _building_in_row_strip),
    ("G12", None, _crossing_rows),
    ("G13", None, _overlapping_sectors), ("G13", None, _overlapping_same_scheme),
    ("G14", "Z0", _missing_valve), ("G14", "Z0", _valve_not_a_valve),
]


@pytest.mark.parametrize("rule,fid,mutate", RULE_CASES, ids=[c[2].__name__.strip("_") for c in RULE_CASES])
def test_rule_fires(farm, rule, fid, mutate):
    mutate(farm)
    fired = rules(farm)
    assert rule in {r for r, _ in fired}, fired
    if fid:
        assert (rule, fid) in fired


def test_other_scheme_may_overlap(farm):
    add(farm, box(30, -40, 70, -10), layer="overlay", kind="management_zone", scheme="soil_2025", value="clay")
    assert rules(farm) == set()


def test_restricted_areas_may_overlap(farm):
    add(farm, box(-85, 15, -55, 45), layer="overlay", kind="restricted_area", restriction="caution")
    assert rules(farm) == set()


def test_g15_empty_overlay_warns(farm):
    zid = add(farm, box(-5, 20, 5, 30), layer="overlay", kind="restricted_area", restriction="caution")
    assert rules(farm, "warning") == {("G15", zid)}


def test_g16_service_point_far_from_road_warns(farm):
    sid = add(farm, Point(60, 50), layer="object", category="service_point", kind="dock")
    assert rules(farm, "warning") == {("G16", sid)}


def test_shared_area_boundary_is_not_an_overlap(farm):
    add(farm, box(-50, -50, -40, -30), layer="area", kind="yard")  # touches A0 and A1
    assert rules(farm) == set()


# --- generators ---------------------------------------------------------------------------

def test_generate_rows_rectangle():
    rows = fg.generate_rows(box(0, 0, 40, 20), 0.0, 2.5, headland_m=4.0)
    assert len(rows) == 8
    assert [r.coords[0][1] for r in rows] == pytest.approx([1.25 + 2.5 * k for k in range(8)])
    assert all(r.length == pytest.approx(32.0) and r.coords[0][0] == pytest.approx(4.0) for r in rows)


def test_generate_rows_rotated_and_count():
    parcel = affinity.rotate(box(0, 0, 30, 30), 30, origin="centroid")
    rows = fg.generate_rows(parcel, 30.0, 5.0, headland_m=3.0, count=4)
    assert len(rows) == 4
    for r in rows:
        (x0, y0), (x1, y1) = r.coords[0], r.coords[-1]
        assert fg.heading_deg(x1 - x0, y1 - y0) == pytest.approx(30.0)
        assert r.length == pytest.approx(24.0)


def test_generate_rows_concave_parcel_splits_rows():
    u_shape = Polygon([(0, 0), (30, 0), (30, 20), (20, 20), (20, 10), (10, 10), (10, 20), (0, 20)])
    rows = fg.generate_rows(u_shape, 0.0, 2.0, headland_m=1.0)
    upper = [r for r in rows if r.coords[0][1] > 10]
    assert len(upper) == 2 * 5 and all(r.length == pytest.approx(8.0) for r in upper)


def test_generate_rows_from_longest_edge():
    parcel = affinity.rotate(box(0, 0, 60, 10), 45, origin=(0, 0))
    assert fg.edge_heading(parcel) % 180 == pytest.approx(45.0)


def test_generate_plants_and_posts():
    row = LineString([(0, 0), (10, 0)])
    assert [p.x for p in fg.generate_plants(row, 2.0)] == pytest.approx([0, 2, 4, 6, 8, 10])
    assert [p.x for p in fg.generate_plants(row, 2.0, 1.0, 1.0)] == pytest.approx([1, 3, 5, 7, 9])
    posts, anchors = fg.generate_posts(row, 4.0)
    assert [p.x for p in posts] == pytest.approx([0, 10 / 3, 20 / 3, 10])
    assert [(a.x, a.y) for a in anchors] == pytest.approx([(-1, 0), (11, 0)])


def test_make_rows_and_plants_features_validate(farm):
    out = fg.make_rows(farm, "A1", {"edge": 0, "row_spacing_m": 2.0, "headland_m": 1.0, "crop": "cabbage"})
    assert out["stats"]["existing_rows"] == 10 and out["stats"]["existing_objects"] == 90
    assert len(out["existing"]) == 100 and out["updates"] == {"A1": {"row_spacing_m": 2.0, "headland_m": 1.0}}
    farm["features"] = [f for f in farm["features"] if f["id"] not in out["existing"]]
    for f in out["features"]:
        f["id"] = fg.new_id(farm, "r")
        farm["features"].append(f)
    by_id(farm, "A1")["properties"].update(out["updates"]["A1"])
    assert len(out["features"]) == 20
    new_rows = [f["id"] for f in out["features"]]
    plants = fg.make_plants(farm, new_rows, {"plant_spacing_m": 0.5})
    assert plants["existing"] == [] and len(plants["features"]) == 20 * 37
    for f in plants["features"]:
        f["id"] = fg.new_id(farm, "c")
        farm["features"].append(f)
    assert {r for r, _ in rules(farm)} == set()
    assert fg.derive(farm)["A1"]["n_rows"] == 20


def test_schema_prefixes_match_dsg_spec():
    expected = {("farm", None): "F", ("overlay", None): "Z", ("area", None): "A", ("row", None): "r",
                ("object", "crop"): "c", ("object", "weed"): "k", ("object", "vegetation"): "v",
                ("object", "row_support"): "i", ("object", "equipment"): "e", ("object", "structure"): "B",
                ("object", "service_point"): "s", ("object", "movable"): "m", ("object", "obstacle"): "o",
                ("navigation", None): "R"}
    got = {(t["layer"], t["category"]): t["prefix"] for t in fg.default_schema().type_list}
    assert got == expected


def test_workflow_buttons_name_existing_types():
    schema = fg.default_schema()
    for step in schema.workflow:
        for b in step["buttons"]:
            if "action" in b:
                continue
            t = schema.entry(b["layer"], b.get("category"), b.get("kind"))
            assert t is not None, b
            assert b.get("geometry", t["geometry"][0]) in t["geometry"], b
            for key, value in b.get("props", {}).items():
                attr = next(a for a in t["attributes"] if a["name"] == key)
                assert attr["type"] != "enum" or value in attr["values"], b


def test_make_rows_reverse_and_typed_heading(farm):
    base = {"row_spacing_m": 4.0, "headland_m": 2.0, "crop": "cabbage"}
    fwd = fg.make_rows(farm, "A1", {**base, "heading_deg": 90})
    rev = fg.make_rows(farm, "A1", {**base, "heading_deg": 90, "reverse": True})
    assert fwd["stats"]["heading_deg"] == 90 and rev["stats"]["heading_deg"] == 270
    a, b = fwd["features"][0]["geometry"]["coordinates"], rev["features"][-1]["geometry"]["coordinates"]
    assert a[0] == pytest.approx(b[-1], abs=1e-7) and a[-1] == pytest.approx(b[0], abs=1e-7)
    assert fwd["stats"]["length_min_m"] == pytest.approx(16.0, abs=0.01)


def test_plants_and_posts_accept_a_parcel(farm):
    plants = fg.make_plants(farm, ["A0"], {"plant_spacing_m": 4.0})
    assert plants["stats"] == {"count": 8 * 9, "rows": 8, "existing_objects": 8 * 17}
    assert set(plants["updates"]) == {f"r{i}" for i in range(8)}
    posts = fg.make_posts(farm, ["A0"], {"post_spacing_m": 8.0, "end_anchors": False})
    assert posts["stats"] == {"count": 8 * 5, "rows": 8, "existing_objects": 8 * (8 + 2)}  # 8 posts + 2 anchors
    assert {f["properties"]["kind"] for f in posts["features"]} == {"post"}


@pytest.mark.parametrize("call", [
    lambda fc: fg.make_rows(fc, "A1", {"row_spacing_m": 0, "crop": "cabbage"}),
    lambda fc: fg.make_rows(fc, "A1", {"row_spacing_m": 0.0001, "crop": "cabbage"}),
    lambda fc: fg.make_rows(fc, "A1", {"row_spacing_m": 2, "crop": "banana"}),
    lambda fc: fg.make_rows(fc, "A1", {"row_spacing_m": 2, "headland_m": -1, "crop": "cabbage"}),
    lambda fc: fg.make_plants(fc, ["r0"], {"plant_spacing_m": -1}),
    lambda fc: fg.make_plants(fc, ["A0"], {"plant_spacing_m": 0.0001}),
    lambda fc: fg.make_plants(fc, ["e1"], {"plant_spacing_m": 1}),
])
def test_generators_reject_bad_parameters(farm, call):
    with pytest.raises((ValueError, KeyError)):
        call(farm)


def test_end_anchors_belong_to_their_row_at_tight_spacing(farm):
    rows = fg.make_rows(farm, "A1", {"heading_deg": 90, "row_spacing_m": 2.0, "headland_m": 2.0, "crop": "cabbage"})
    farm["features"] = [f for f in farm["features"] if f["id"] not in rows["existing"]]
    for f in rows["features"]:
        f["id"] = fg.new_id(farm, "r")
        farm["features"].append(f)
    by_id(farm, "A1")["properties"].update(rows["updates"]["A1"])
    posts = fg.make_posts(farm, ["A1"], {"post_spacing_m": 5.0, "anchor_offset_m": 1.0})
    for f in posts["features"]:
        f["id"] = fg.new_id(farm, "i")
        farm["features"].append(f)
    assert rules(farm) == set()
    d = fg.derive(farm)
    anchors = [f["id"] for f in posts["features"] if f["properties"]["kind"] == "end_anchor"]
    assert all(d[a]["parent"].startswith("r") for a in anchors)
