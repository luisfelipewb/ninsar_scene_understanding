"""Checks the farm editor server API with Flask's test client."""

import json
import os
import shutil
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts", "farm_editor"))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import farm_geojson as fg  # noqa: E402
from serve import create_app  # noqa: E402

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
