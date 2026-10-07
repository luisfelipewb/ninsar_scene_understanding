// Local ENU frame, the same formulas as farm_geojson.Frame (WGS84 ellipsoid), so typed and
// displayed x/y match the validator and the importer.

const A = 6378137.0;
const F = 1 / 298.257223563;
const E2 = F * (2 - F);
const EARTH_R = 6371000.0;
const rad = (d) => (d * Math.PI) / 180;
const deg = (r) => (r * 180) / Math.PI;

function geodeticToEcef(lat, lon, alt) {
  const [la, lo] = [rad(lat), rad(lon)];
  const n = A / Math.sqrt(1 - E2 * Math.sin(la) ** 2);
  return [(n + alt) * Math.cos(la) * Math.cos(lo), (n + alt) * Math.cos(la) * Math.sin(lo),
    (n * (1 - E2) + alt) * Math.sin(la)];
}

function ecefToGeodetic([x, y, z]) {
  const lon = Math.atan2(y, x);
  const p = Math.hypot(x, y);
  let lat = Math.atan2(z, p * (1 - E2));
  for (let i = 0; i < 6; i += 1) {
    const n = A / Math.sqrt(1 - E2 * Math.sin(lat) ** 2);
    const alt = p / Math.cos(lat) - n;
    lat = Math.atan2(z, p * (1 - E2 * (n / (n + alt))));
  }
  return [deg(lat), deg(lon)];
}

export class Frame {
  constructor(lat, lon, alt = 0) {
    this.lat = lat; this.lon = lon; this.alt = alt;
    this.ecef0 = geodeticToEcef(lat, lon, alt);
    const [sl, cl, so, co] = [Math.sin(rad(lat)), Math.cos(rad(lat)), Math.sin(rad(lon)), Math.cos(rad(lon))];
    this.R = [[-so, co, 0], [-sl * co, -sl * so, cl], [cl * co, cl * so, sl]];  // rows: E, N, U
  }

  // [lon, lat] -> [x, y] metres (at the origin altitude)
  enu([lon, lat]) {
    const e = geodeticToEcef(lat, lon, this.alt);
    const d = [e[0] - this.ecef0[0], e[1] - this.ecef0[1], e[2] - this.ecef0[2]];
    const [r0, r1] = this.R;
    return [r0[0] * d[0] + r0[1] * d[1] + r0[2] * d[2], r1[0] * d[0] + r1[1] * d[1] + r1[2] * d[2]];
  }

  // [x, y] metres -> [lon, lat], inverse of enu()
  wgs84([x, y]) {
    const z = -(x * x + y * y) / (2 * EARTH_R);
    const [r0, r1, r2] = this.R;
    const e = [0, 1, 2].map((k) => this.ecef0[k] + x * r0[k] + y * r1[k] + z * r2[k]);
    const [lat, lon] = ecefToGeodetic(e);
    return [lon, lat];
  }
}

export function frameOf(fc) {
  const o = fc?.origin;
  return o ? new Frame(o.lat, o.lon, o.alt || 0) : null;
}

// area centroid of a [lon, lat] ring, as shapely computes it (used to set the origin once)
export function ringCentroid(ring) {
  let a = 0; let cx = 0; let cy = 0;
  for (let i = 0; i < ring.length - 1; i += 1) {
    const [x0, y0] = ring[i]; const [x1, y1] = ring[i + 1];
    const c = x0 * y1 - x1 * y0;
    a += c; cx += (x0 + x1) * c; cy += (y0 + y1) * c;
  }
  return a ? [cx / (3 * a), cy / (3 * a)] : ring[0];
}

// counter-clockwise from east in [0, 360), rounded to 0.01° first (as farm_geojson.heading_deg)
export function headingDeg(dx, dy) {
  const h = ((((Math.atan2(dy, dx) * 180) / Math.PI) % 360) + 360) % 360;
  return (Math.round(h * 100) / 100) % 360;
}
