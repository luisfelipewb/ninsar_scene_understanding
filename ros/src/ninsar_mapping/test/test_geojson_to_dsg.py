"""Tests for scripts/geojson_to_dsg.py and scripts/farm_dsg.py on test/data/mini_farm.geojson."""
import copy
import math
import os
import sys
from collections import Counter

import numpy as np
import pytest
import spark_dsg as dsg
from shapely.geometry import Point, shape

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import farm_dsg as fd  # noqa: E402
import farm_geojson as fg  # noqa: E402
import geojson_to_dsg as g2d  # noqa: E402

MINI_FARM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "mini_farm.geojson")


@pytest.fixture(scope="module")
def fc():
    return fg.load(MINI_FARM)


@pytest.fixture(scope="module")
def idx(fc):
    return fg.check(fc)[1]


@pytest.fixture(scope="module")
def G(fc):
    return g2d.build(fc, source_path=MINI_FARM)[0]


def relations(G):
    return Counter(fd.relation(e) for e in G.edges)


def node_meta(G, u):
    return fd.meta(G.get_node(fd.symbol(u)).attributes)


def nav_edges(G, rel=None):
    return [(fd.uid(e.source), fd.uid(e.target), fd.meta(e)) for e in G.edges
            if fd.relation(e) in fd.NAVIGATION and (rel is None or fd.relation(e) == rel)]


def add(fc, geom, **props):
    """Append a feature given in ENU metres; returns its id."""
    fid = fg.new_id(fc, fg.default_schema().type_of(props)["prefix"])
    fc["features"].append({"type": "Feature", "id": fid, "properties": props,
                           "geometry": fg.to_geojson(geom, fg.frame_of(fc))})
    return fid


# --- structure ---------------------------------------------------------------------------

def test_graph_meets_the_spec(G):
    assert fd.verify(G) == []


def test_node_and_edge_counts(G):
    counts = {name: (G.get_layer(l, p) if p else G.get_layer(l)).num_nodes() for (l, p), name in fd.LAYERS.items()}
    assert {k: counts[k] for k in ("FARM", "OVERLAYS", "AREAS", "ROWS", "OBJECTS", "AGENTS")} == \
        {"FARM": 1, "OVERLAYS": 5, "AREAS": 6, "ROWS": 24, "OBJECTS": 354, "AGENTS": 0}
    rel = relations(G)
    assert (rel["contains"], rel["next_in_row"], rel["adjacent_row"]) == (384, 244, 21)
    assert rel["adjacent_area"] == 2  # A0-A1 and A4-A5, 10 m apart


def test_every_feature_is_a_node_with_its_id(G, idx):
    where = {"farm": (fd.FARM, fd.P0), "overlay": (fd.FARM, fd.OVERLAYS), "area": (fd.AREAS, fd.P0),
             "row": (fd.ROWS, fd.P0), "object": (fd.OBJECTS, fd.P0)}
    for fid, n in idx.nodes.items():
        if n.layer == "navigation":
            assert not G.has_node(fd.symbol(fid))  # roads are sampled into waypoints
            continue
        node = G.get_node(dsg.NodeSymbol(*fg.parse_id(fid)))
        assert fd.layer_key(node) == where[n.layer]
        assert node_meta(G, fid)["uid"] == fid
        assert node_meta(G, fid)["source"] == "prior" and node_meta(G, fid)["status"] == "expected"


def test_contains_parent_matches_the_derived_tree(G, idx):
    for fid, n in idx.nodes.items():
        if n.layer != "navigation" and n.layer != "overlay":
            assert fd.parent(G, fid) == idx.derived[fid].get("parent"), fid
    # r13 is in Z3 directly, so spark sees two parents (A1, Z3) and get_parent() gives up
    assert G.get_node(fd.symbol("r13")).get_parent() is None
    assert fd.lineage(G, "c0") == idx.lineage("c0") == ["r0", "A0", "F0"]


def test_overlay_membership(G, idx):
    for z in idx.by_layer["overlay"]:
        tree_members = {m for m in fd.neighbours(G, z.id, "member_of") if not m.startswith("w")}
        assert tree_members == set(idx.derived[z.id]["members"])
    plant = next(c for c in idx.derived["r14"]["children"] if idx.nodes[c].category == "crop")
    assert {"Z1", "Z3"} <= set(fd.overlays(G, plant))
    assert plant in fd.members(G, "Z3")
    assert "members" not in node_meta(G, "Z3") and node_meta(G, "Z3")["restriction"] == "caution"


# --- node attributes ---------------------------------------------------------------------

def test_row_and_parcel_metadata(G, idx):
    a2 = node_meta(G, "A2")
    assert a2["heading_deg"] == pytest.approx(30.0) and a2["n_rows"] == 6 and a2["crops"] == ["lemon"]
    assert a2["row_spacing_m"] == 5.0
    for r in idx.derived["A2"]["children"]:
        box = G.get_node(fd.symbol(r)).attributes.bounding_box
        yaw = math.atan2(box.world_R_center[1, 0], box.world_R_center[0, 0]) % math.pi
        assert yaw == pytest.approx(math.radians(30.0), abs=1e-3)
    r0 = node_meta(G, "r0")
    assert (r0["crop"], r0["row_index"], r0["n_plants"]) == ("grapevine", 0, 17)
    assert r0["plant_spacing_m"] == pytest.approx(2.0, abs=1e-3)
    assert all(node_meta(G, c)["variety"] == "gamay" for c in fd.children(G, "r0") if c.startswith("c"))
    assert "geometry" in r0 and "geometry" not in node_meta(G, "c0")


def test_bounding_boxes(G, idx):
    schema = fg.default_schema()
    c0 = G.get_node(fd.symbol("c0")).attributes
    assert np.allclose(c0.bounding_box.dimensions, schema.default_size_m["grapevine"])
    assert c0.position[2] == pytest.approx(schema.default_size_m["grapevine"][2] / 2)
    assert c0.semantic_label == schema.label_ids["grapevine"] == 7
    b0 = G.get_node(fd.symbol("B0")).attributes.bounding_box
    rect = idx.nodes["B0"].geom.minimum_rotated_rectangle
    assert b0.dimensions[0] * b0.dimensions[1] == pytest.approx(rect.area, rel=1e-3)
    assert b0.dimensions[2] == pytest.approx(schema.default_size_m["building"][2])


# --- navigation --------------------------------------------------------------------------

def test_lanes_per_parcel_and_crop_access(G, idx):
    for parcel in ("A0", "A1", "A2"):
        wps = [n.id.str() for n in G.get_layer(fd.OBJECTS, fd.NAV).nodes
               if node_meta(G, n.id.str()).get("parcel") == parcel]
        assert {node_meta(G, w)["lane_index"] for w in wps} == set(range(idx.derived[parcel]["n_rows"] + 1))
    for fid, n in idx.nodes.items():
        if n.category != "crop":
            continue
        row = idx.derived[fid]["parent"]
        i, parcel = idx.derived[row]["row_index"], idx.derived[row]["parent"]
        targets = [node_meta(G, w) for w in fd.neighbours(G, fid, "access")]
        assert sorted(t["lane_index"] for t in targets) == [i, i + 1], fid
        assert {t["parcel"] for t in targets} == {parcel}


def test_service_point_route_reaches_every_parcel(G, idx):
    assert fd.neighbours(G, "s0", "access")
    for parcel in ("A0", "A1", "A2"):
        row = next(r for r in idx.derived[parcel]["children"] if idx.derived[r]["row_index"] == 0)
        length, path = fd.route(G, "s0", idx.derived[row]["children"][0])
        assert path and math.isfinite(length), parcel


def test_restricted_area_waypoints(G, idx):
    in_z3 = [w for w in fd.neighbours(G, "Z3", "member_of") if w.startswith("w")]
    assert in_z3
    zone = idx.zone(idx.nodes["Z3"])
    assert all(zone.covers(Point(G.get_node(fd.symbol(w)).attributes.position[:2])) for w in in_z3)
    assert not [w for w in fd.neighbours(G, "Z2", "member_of") if w.startswith("w")]


def test_entrance_avoids_the_weather_station(G):
    # e1 stands on the straight line from A1 to the nearest road waypoint
    entrances = nav_edges(G, "entrance")
    assert len(entrances) == 3 and not any(m.get("blocked") for _, _, m in entrances)


def test_obstacle_on_a_lane_blocks_it_and_routes_detour():
    fc = fg.load(MINI_FARM)
    rock = add(fc, Point(-36.0, -40.0), layer="object", category="obstacle", kind="rock")
    G, _ = g2d.build(fc)
    assert fd.verify(G) == []
    blocked = [(a, b) for a, b, m in nav_edges(G) if rock in m.get("blocked_by", [])]
    assert blocked and all(m["relation"] == "lane" for _, _, m in nav_edges(G) if rock in m.get("blocked_by", []))
    target = next(c for r in fd.children(G, "A1") for c in fd.children(G, r) if c.startswith("c"))
    _, path = fd.route(G, "s0", target)
    assert path
    hops = {frozenset(p) for p in zip(path, path[1:])}
    assert not hops & {frozenset(e) for e in blocked}


# --- refusal, output, determinism --------------------------------------------------------

def test_file_with_errors_is_refused(tmp_path):
    fc = fg.load(MINI_FARM)
    fc["features"] = [f for f in fc["features"] if f["id"] != "F0"]  # G1
    with pytest.raises(g2d.ImportRefused):
        g2d.build(fc)
    path, out = tmp_path / "bad.geojson", tmp_path / "bad_dsg.json"
    fg.save(str(path), fc)
    assert g2d.main([str(path), "-o", str(out), "-q"]) == 1
    assert not out.exists()


def test_cli_output_round_trip_and_overrides(tmp_path, G):
    out = tmp_path / "mini_dsg.json"
    assert g2d.main([MINI_FARM, "-o", str(out), "-q", "--set", "near_m=0.25"]) == 0
    H = dsg.DynamicSceneGraph.load(str(out))
    meta = dict(H.metadata.get())
    assert meta["thresholds"]["near_m"] == 0.25
    assert meta["spec_version"] == fd.DSG_SPEC_VERSION and meta["frame"] == "map" and meta["site"] == "mini_farm"
    assert meta["labelspaces"]["2"] == {"7": "grapevine", "8": "lemon", "9": "cabbage"}
    assert meta["prior"]["file"] == "mini_farm.geojson" and len(meta["prior"]["sha256"]) == 64
    assert (H.num_nodes(), H.num_edges()) == (G.num_nodes(), G.num_edges())
    assert relations(H) == relations(G)
    assert H.get_labelspace(fd.ROWS, 0).labels_to_names == {7: "grapevine", 8: "lemon", 9: "cabbage"}


def test_build_is_deterministic(tmp_path, fc):
    paths = []
    for k in range(2):
        paths.append(tmp_path / f"g{k}.json")
        g2d.build(copy.deepcopy(fc), source_path=MINI_FARM)[0].save(str(paths[-1]), include_mesh=False)
    assert paths[0].read_bytes() == paths[1].read_bytes()


# --- schema and checker ------------------------------------------------------------------

def test_label_ids_cover_every_class_once():
    schema = fg.default_schema()
    classes = {k for t in schema.type_list if t["layer"] == "object" for k in t["kinds"]}
    assert classes == set(schema.label_ids)
    assert len(set(schema.label_ids.values())) == len(schema.label_ids)
    isaac = {"unknown": 2, "fence": 4, "building": 5, "tree": 6, "grapevine": 7, "lemon": 8, "cabbage": 9}
    assert {k: schema.label_ids[k] for k in isaac} == isaac
    assert not {0, 1, 3} & set(schema.label_ids.values())


def test_object_prefixes_match_the_schema():
    schema = fg.default_schema()
    assert {t["prefix"] for t in schema.type_list if t["layer"] == "object"} == fd.OBJECT_PREFIXES


def test_verify_catches_tree_violations(G):
    H = G.clone()
    H.remove_edge(fd.symbol("A0").value, fd.symbol("r0").value)
    w = next(n.id for n in H.get_layer(fd.OBJECTS, fd.NAV).nodes)
    info = dsg.EdgeAttributes()
    info.metadata.set({"relation": "contains"})
    H.insert_edge(fd.symbol("A1"), w, info)
    rules = {(v.rule, v.uid) for v in fd.verify(H)}
    assert ("T1", "r0") in rules
    assert any(r == "T5" for r, _ in rules)


def test_verify_catches_exclusive_overlap(G):
    H = G.clone()
    info = dsg.EdgeAttributes()
    info.metadata.set({"relation": "member_of"})
    H.insert_edge(fd.symbol("r0"), fd.symbol("Z1"), info)  # r0 is already in Z0 through A0
    assert ("O4", "r0") in {(v.rule, v.uid) for v in fd.verify(H)}


def test_geometry_of_areas_is_enu(G, idx):
    g = shape(node_meta(G, "A1")["geometry"])
    assert g.hausdorff_distance(idx.nodes["A1"].geom) < 1e-3  # rounded to 1 mm
