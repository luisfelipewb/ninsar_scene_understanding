#!/usr/bin/env python3
"""Farm editor: a local web UI that writes the farm GeoJSON (docs/FARM_EDITOR.md).

The browser does the drawing; this server holds the working file and runs the shared
library (scripts/farm_geojson.py) for validation, derived values and the generators.

Usage:
  .venv/bin/python src/ninsar_mapping/scripts/farm_editor/serve.py ~/data/ninsar/farms/montoldre.geojson \
      [--imagery DIR] [--port 8765] [--host 127.0.0.1] [--no-browser]
"""
import argparse
import json
import os
import sys
import threading
import webbrowser
from dataclasses import asdict

from flask import Flask, abort, jsonify, request, send_from_directory

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import farm_geojson as fg  # noqa: E402

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".webp")
CORNERS = ("topleft", "topright", "bottomleft")


def scan_imagery(root):
    """Overlays in `root` (docs/FARM_EDITOR.md §7): images with a JSON sidecar, and XYZ tile dirs.

    Image corners are returned as [lat, lon]; `enu` sidecars are converted with their origin.
    Entries that cannot be read are returned with an `error` so the UI can say why.
    """
    if not root or not os.path.isdir(root):
        return []
    out = []
    for name in sorted(os.listdir(root)):
        full = os.path.join(root, name)
        stem, ext = os.path.splitext(name)
        if os.path.isdir(full) and os.path.exists(os.path.join(full, "tiles.json")):
            try:
                meta = json.load(open(os.path.join(full, "tiles.json")))
                out.append({"id": name, "type": "tiles", "label": meta.get("label", name),
                            "url": f"/imagery/{name}/{{z}}/{{x}}/{{y}}.{meta.get('ext', 'png')}",
                            "min_zoom": meta.get("min_zoom", 0), "max_zoom": meta.get("max_zoom", 22)})
            except (OSError, ValueError) as e:
                out.append({"id": name, "type": "tiles", "label": name, "error": str(e)})
        elif ext.lower() in IMAGE_EXTS and os.path.exists(os.path.join(root, stem + ".json")):
            entry = {"id": name, "type": "image", "label": stem, "url": f"/imagery/{name}"}
            try:
                meta = json.load(open(os.path.join(root, stem + ".json")))
                entry["label"] = meta.get("label", stem)
                corners = meta["corners"]
                if meta.get("frame", "wgs84") == "enu":
                    frame = fg.Frame.from_origin(meta["origin"])
                    lonlat = frame.wgs84([corners[c] for c in CORNERS])
                else:
                    lonlat = [corners[c] for c in CORNERS]
                entry["corners"] = {c: [float(ll[1]), float(ll[0])] for c, ll in zip(CORNERS, lonlat)}
                entry["opacity"] = meta.get("opacity", 0.8)
            except (OSError, ValueError, KeyError, TypeError) as e:
                entry["error"] = f"{type(e).__name__}: {e}"
            out.append(entry)
    return out


def create_app(path, imagery=None, schema=None):
    path = os.path.abspath(os.path.expanduser(path))
    schema = schema or fg.default_schema()
    app = Flask(__name__, static_folder=STATIC, static_url_path="/static")

    def report(fc):
        issues, idx = fg.check(fc, schema)
        return {"issues": [asdict(i) for i in issues], "derived": idx.derived}

    def body_fc():
        fc = request.get_json(force=True)
        if isinstance(fc, dict) and "fc" in fc:
            fc = fc["fc"]
        if not isinstance(fc, dict) or not isinstance(fc.get("features"), list):
            abort(400, "expected a FeatureCollection")
        return fg.normalize(fc)

    @app.get("/")
    def index():
        return send_from_directory(STATIC, "index.html")

    @app.get("/api/schema")
    def get_schema():
        return jsonify(schema.to_json())

    @app.get("/api/farm")
    def get_farm():
        if os.path.exists(path):
            fc = fg.load(path)
        else:
            fc = fg.new_collection(os.path.splitext(os.path.basename(path))[0])
        autosave = path + ".autosave"
        newer = os.path.exists(autosave) and (not os.path.exists(path)
                                              or os.path.getmtime(autosave) > os.path.getmtime(path))
        return jsonify({"fc": fc, "path": path, "exists": os.path.exists(path), "autosave_newer": newer})

    @app.get("/api/autosave")
    def get_autosave():
        if not os.path.exists(path + ".autosave"):
            abort(404)
        return jsonify({"fc": fg.load(path + ".autosave")})

    @app.put("/api/farm")
    def put_farm():
        saved = fg.save(path, body_fc())
        if os.path.exists(path + ".autosave"):
            os.remove(path + ".autosave")
        return jsonify({"saved": True, "path": path, "origin": saved.get("origin"),
                        "next_id": saved["next_id"], **report(saved)})

    @app.post("/api/autosave")
    def post_autosave():
        fc = body_fc()
        with open(path + ".autosave", "w") as f:
            f.write(fg.dumps(fc))
        return jsonify({"ok": True})

    @app.post("/api/validate")
    def post_validate():
        return jsonify(report(body_fc()))

    generators = {"rows": fg.make_rows, "plants": fg.make_plants, "posts": fg.make_posts, "offset": fg.make_offset_rows}

    @app.post("/api/generate/<what>")
    def post_generate(what):
        """rows: {fc, parcel_id, params}; offset: {fc, row_id, params};
        plants/posts: {fc, target_ids (rows or parcels), params}."""
        if what not in generators:
            abort(404)
        body = request.get_json(force=True)
        fc = fg.normalize(body["fc"])
        target = {"rows": body.get("parcel_id"), "offset": body.get("row_id")}.get(
            what, body.get("target_ids", body.get("row_ids")))
        try:
            return jsonify(generators[what](fc, target, body.get("params", {}), schema))
        except (KeyError, ValueError, TypeError) as e:
            return jsonify({"error": f"{type(e).__name__}: {e}"}), 400

    imagery_root = os.path.abspath(os.path.expanduser(imagery)) if imagery else None

    @app.get("/api/imagery")
    def get_imagery():
        return jsonify(scan_imagery(imagery_root))

    @app.get("/imagery/<path:name>")
    def get_imagery_file(name):
        if not imagery_root:
            abort(404)
        return send_from_directory(imagery_root, name)  # refuses paths outside the folder

    app.config["FARM_PATH"] = path
    app.config["IMAGERY"] = imagery
    return app


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="farm GeoJSON to edit (created on first save)")
    ap.add_argument("--imagery", help="directory with local overlays (docs/FARM_EDITOR.md §7)")
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to allow other devices")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args(argv)

    app = create_app(args.file, args.imagery)
    url = f"http://{'127.0.0.1' if args.host == '0.0.0.0' else args.host}:{args.port}"
    print(f"farm editor: {app.config['FARM_PATH']} at {url}")
    if not args.no_browser:
        threading.Timer(1.0, webbrowser.open, (url,)).start()
    app.run(host=args.host, port=args.port, debug=False)


if __name__ == "__main__":
    main()
