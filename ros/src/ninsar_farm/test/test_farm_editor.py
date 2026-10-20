"""Checks the farm editor server API with Flask's test client."""

import json
import os
import shutil

import pytest

from ninsar_farm import farm_geojson as fg
from ninsar_farm.editor.serve import create_app

HERE = os.path.dirname(os.path.abspath(__file__))

MINI_FARM = os.path.join(HERE, "data", "mini_farm.geojson")


@pytest.fixture
def farm_path(tmp_path):
    path = tmp_path / "farm.geojson"
    shutil.copy(MINI_FARM, path)
    return str(path)


@pytest.fixture
def client(farm_path):
    return create_app(farm_path).test_client()


def test_index_and_static_files(client):
    assert b"Farm editor" in client.get("/").data
    for path in ("/static/js/app.js", "/static/style.css", "/static/vendor/leaflet/leaflet.js",
                 "/static/vendor/geoman/leaflet-geoman.js"):
        assert client.get(path).status_code == 200, path


def test_schema_has_types_and_workflow(client):
    schema = client.get("/api/schema").get_json()
    assert {"types", "workflow", "crops", "thresholds"} <= set(schema)
    rows = next(t for t in schema["types"] if t["layer"] == "row")
    assert rows["kinds"] == list(schema["crops"])


def test_get_farm_existing_and_new(client, tmp_path):
    body = client.get("/api/farm").get_json()
    assert body["exists"] and len(body["fc"]["features"]) == 391 and not body["autosave_newer"]
    new = create_app(str(tmp_path / "sub" / "new_farm.geojson")).test_client().get("/api/farm").get_json()
    assert not new["exists"] and new["fc"]["features"] == [] and new["fc"]["site"] == "new_farm"


def test_save_writes_file_backup_and_report(client, farm_path):
    fc = client.get("/api/farm").get_json()["fc"]
    fc["features"] = [f for f in fc["features"] if f["id"] != "e1"]
    res = client.put("/api/farm", json=fc).get_json()
    assert res["saved"] and res["issues"] == [] and "A0" in res["derived"]
    assert os.path.exists(farm_path + ".bak")
    assert all(f["id"] != "e1" for f in fg.load(farm_path)["features"])


def test_first_save_sets_origin(tmp_path):
    path = str(tmp_path / "f.geojson")
    client = create_app(path).test_client()
    fc = client.get("/api/farm").get_json()["fc"]
    fc["features"].append({"type": "Feature", "id": "F0", "properties": {"layer": "farm", "kind": "farm", "name": "x"},
                           "geometry": {"type": "Polygon", "coordinates": [[[3, 46], [3.002, 46], [3.002, 46.002],
                                                                            [3, 46.002], [3, 46]]]}})
    fc["next_id"] = {"F": 1}
    res = client.put("/api/farm", json=fc).get_json()
    assert res["origin"] == {"lat": 46.001, "lon": 3.001, "alt": 0.0} and res["issues"] == []
    assert json.load(open(path))["origin"] == res["origin"]


def test_validate_reports_issues(client):
    fc = client.get("/api/farm").get_json()["fc"]
    next(f for f in fc["features"] if f["id"] == "A4")["properties"].pop("water_type")
    res = client.post("/api/validate", json=fc).get_json()
    assert [(i["rule"], i["id"]) for i in res["issues"]] == [("G5", "A4")]
    assert res["derived"]["r0"]["parent"] == "A0"


def test_autosave_round_trip(client, farm_path):
    fc = client.get("/api/farm").get_json()["fc"]
    fc["site"] = "autosaved"
    assert client.post("/api/autosave", json=fc).get_json() == {"ok": True}
    os.utime(farm_path, (0, 0))  # make the file older than the autosave
    assert client.get("/api/farm").get_json()["autosave_newer"]
    assert client.get("/api/autosave").get_json()["fc"]["site"] == "autosaved"
    client.put("/api/farm", json=fc)
    assert not os.path.exists(farm_path + ".autosave")


def test_generate_rows_and_errors(client):
    fc = client.get("/api/farm").get_json()["fc"]
    res = client.post("/api/generate/rows", json={"fc": fc, "parcel_id": "A1", "params": {
        "heading_deg": 90, "row_spacing_m": 4.0, "headland_m": 2.0, "crop": "cabbage"}}).get_json()
    assert len(res["features"]) == 10 and res["stats"]["existing_rows"] == 10 and len(res["existing"]) == 100
    bad = client.post("/api/generate/rows", json={"fc": fc, "parcel_id": "A99", "params": {}})
    assert bad.status_code == 400 and "error" in bad.get_json()
    assert client.post("/api/generate/nothing", json={}).status_code == 404


@pytest.fixture
def imagery_dir(tmp_path):
    from PIL import Image
    d = tmp_path / "imagery"
    d.mkdir()
    Image.new("RGB", (40, 30), (90, 140, 60)).save(d / "isaac.png")
    (d / "isaac.json").write_text(json.dumps({
        "label": "Isaac top-down", "frame": "enu", "origin": {"lat": 46.3374, "lon": 3.434, "alt": 250.0},
        "corners": {"topleft": [-100, 60], "topright": [100, 60], "bottomleft": [-100, -60]}}))
    Image.new("RGB", (8, 8)).save(d / "drone.jpg")
    (d / "drone.json").write_text(json.dumps({"corners": {"topleft": [3.433, 46.338], "topright": [3.435, 46.338],
                                                          "bottomleft": [3.433, 46.337]}}))
    Image.new("RGB", (8, 8)).save(d / "broken.png")
    (d / "broken.json").write_text("{not json")
    Image.new("RGB", (8, 8)).save(d / "no_sidecar.png")
    (d / "ortho" / "18" / "1").mkdir(parents=True)
    (d / "ortho" / "tiles.json").write_text(json.dumps({"label": "Orthomosaic", "min_zoom": 15, "max_zoom": 21}))
    Image.new("RGB", (256, 256)).save(d / "ortho" / "18" / "1" / "2.png")
    return d


def test_imagery_scan_and_serving(farm_path, imagery_dir):
    client = create_app(farm_path, str(imagery_dir)).test_client()
    items = {i["id"]: i for i in client.get("/api/imagery").get_json()}
    assert set(items) == {"isaac.png", "drone.jpg", "broken.png", "ortho"}
    isaac = items["isaac.png"]
    frame = fg.Frame(46.3374, 3.434, 250.0)
    tl = frame.enu([[isaac["corners"]["topleft"][1], isaac["corners"]["topleft"][0]]])[0]
    assert tl == pytest.approx([-100, 60], abs=1e-3) and isaac["label"] == "Isaac top-down"
    assert items["drone.jpg"]["corners"]["topright"] == [46.338, 3.435]
    assert "error" in items["broken.png"]
    assert items["ortho"]["url"] == "/imagery/ortho/{z}/{x}/{y}.png" and items["ortho"]["max_zoom"] == 21
    assert client.get("/imagery/isaac.png").status_code == 200
    assert client.get("/imagery/ortho/18/1/2.png").status_code == 200
    assert client.get("/imagery/../farm.geojson").status_code == 404


def test_imagery_without_folder(client):
    assert client.get("/api/imagery").get_json() == []
    assert client.get("/imagery/x.png").status_code == 404


def test_generate_offset_rows(client):
    fc = client.get("/api/farm").get_json()["fc"]
    res = client.post("/api/generate/offset", json={"fc": fc, "row_id": "r7", "params": {"distance_m": 2.5, "count": 3}}).get_json()
    assert res["stats"] == {"count": 3, "span_m": 7.5}
    assert {f["properties"]["kind"] for f in res["features"]} == {"grapevine"}
    assert client.post("/api/generate/offset", json={"fc": fc, "row_id": "A0", "params": {"distance_m": 1}}).status_code == 400
