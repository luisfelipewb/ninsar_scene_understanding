// Entry point: loads the catalog and the working file, wires the top bar, presets,
// keyboard shortcuts and autosave.
import { api } from './api.js';
import {
  addImagery, fitAll, getMap, goTo, imageryBounds, initMap, isBusy, isMeasuring, setPick, startDraw, startMeasure, stopDraw,
  stopMeasure,
} from './map.js';
import { closeTool, isToolOpen, openOffset, openPlants, openPosts, openRows } from './generators.js';
import { Frame, frameOf, headingDeg, ringCentroid } from './geo.js';
import { deleteSelection, initPanels, showTab } from './panels.js';
import {
  addFeatures, byId, canRedo, canUndo, entryFor, ID_RE, markSaved, newId, on, redo, roundGeometry, select,
  setCollection, setReport, state, typeOf, undo, updateFeature,
} from './state.js';
import { initWorkflow } from './workflow.js';

const $ = (id) => document.getElementById(id);
const SIMPLESTYLE = ['stroke', 'stroke-width', 'stroke-opacity', 'fill', 'fill-opacity',
  'marker-color', 'marker-size', 'marker-symbol'];

function status(msg, level = '') {
  const el = $('status');
  el.textContent = msg;
  el.className = level;
}

// --- presets: draw a feature of a catalog type -------------------------------------------

function newFeature(entry, kind, geometry, preset = {}) {
  const properties = { layer: entry.layer };
  if (entry.category) properties.category = entry.category;
  properties.kind = kind;
  for (const a of entry.attributes) if (a.default != null) properties[a.name] = a.default;
  Object.assign(properties, preset);
  return { type: 'Feature', id: newId(entry.prefix), properties, geometry: roundGeometry(geometry) };
}

on('preset', ({ button, entry, geometry, kind }) => {
  closeTool();
  setPick(null);
  const color = state.schema.crops[kind]?.color || entry.color;
  startDraw(geometry, { continueDrawing: !!button.continue, color }, (drawn) => {
    if (drawn.type !== geometry) return;
    const f = newFeature(entry, kind, drawn, button.props);
    if (entry.layer === 'farm' && !state.fc.origin) {  // fix the local frame now; it never moves (§2)
      const [lon, lat] = ringCentroid(f.geometry.coordinates[0]);
      state.fc.origin = { lat: Math.round(lat * 1e8) / 1e8, lon: Math.round(lon * 1e8) / 1e8, alt: 0.0 };
    }
    addFeatures([f]);
    select([f.id]);
  });
});

// a hole: a polygon drawn inside the selected polygon becomes one of its interior rings
function insideRing([x, y], ring) {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i, i += 1) {
    const [xi, yi] = ring[i]; const [xj, yj] = ring[j];
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

function cutHole() {
  const f = state.selection.size === 1 ? byId([...state.selection][0]) : null;
  if (f?.geometry.type !== 'Polygon') {
    status('Select one area (or other polygon) first, then draw the hole inside it', 'warn');
    return;
  }
  closeTool();
  startDraw('Polygon', { color: '#ffffff' }, (hole) => {
    const ring = hole.coordinates[0];
    const [outer, ...holes] = f.geometry.coordinates;
    if (!ring.every((c) => insideRing(c, outer)) || holes.some((h) => ring.some((c) => insideRing(c, h)))) {
      status('The hole must lie inside the polygon and outside its other holes', 'warn');
      return;
    }
    updateFeature(f.id, { geometry: { type: 'Polygon', coordinates: [...f.geometry.coordinates, ring] } });
    select([f.id]);
    status(`Cut a hole in ${f.id}`, 'ok');
  });
}

// --- actions -------------------------------------------------------------------------------

async function save() {
  try {
    status('Saving…');
    const revision = state.revision;
    const res = await api.save(state.fc);
    if (res.origin) state.fc.origin = res.origin;
    for (const [prefix, n] of Object.entries(res.next_id)) {  // ids made during the request stay valid
      state.fc.next_id[prefix] = Math.max(state.fc.next_id[prefix] || 0, n);
    }
    if (state.revision === revision) markSaved();
    setReport(res, revision);
    const errors = res.issues.filter((i) => i.level === 'error').length;
    status(`Saved${errors ? ` with ${errors} errors` : ''}`, errors ? 'warn' : 'ok');
  } catch (e) {
    status(`Save failed: ${e.message}`, 'error');
  }
}

let validating = null;  // the request in flight, so live validation never overlaps

async function validate({ quiet = false } = {}) {
  if (validating) return validating;
  validating = runValidate(quiet).finally(() => { validating = null; });
  return validating;
}

async function runValidate(quiet) {
  try {
    if (!quiet) status('Validating…');
    const t0 = performance.now();
    const revision = state.revision;
    setReport(await api.validate(state.fc), revision);
    const errors = state.issues.filter((i) => i.level === 'error').length;
    status(`${errors} errors, ${state.issues.length - errors} warnings (${Math.round(performance.now() - t0)} ms)`,
      errors ? 'warn' : 'ok');
  } catch (e) {
    status(`Validation failed: ${e.message}`, 'error');
  }
}

function dumps(fc) {  // one feature per line, like farm_geojson.dumps
  const { features, ...head } = fc;
  const lines = Object.entries(head).map(([k, v]) => `  ${JSON.stringify(k)}: ${JSON.stringify(v)},`);
  return `{\n${lines.join('\n')}\n  "features": [\n${features.map((f) => `    ${JSON.stringify(f)}`).join(',\n')}\n  ]\n}\n`;
}

function exportFile() {
  const blob = new Blob([dumps(state.fc)], { type: 'application/geo+json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `${state.fc.site || 'farm'}.geojson`;
  a.click();
  URL.revokeObjectURL(a.href);
}

const ACTIONS = {
  validate, export: exportFile, save,
  tree: async () => { if (state.derivedStale) await validate(); showTab('tree'); },
  generate_rows: openRows,
  generate_plants: openPlants,
  generate_posts: openPosts,
  offset_row: openOffset,
  cut_hole: cutHole,
};
on('action', (name) => ACTIONS[name]?.());

// --- open / import ----------------------------------------------------------------------

function explode(features) {  // the profile allows no Multi* geometries: one part, one feature
  const out = [];
  for (const f of features) {
    const g = f.geometry;
    if (!g || !g.coordinates) continue;
    if (g.type.startsWith('Multi') && g.coordinates.length > 1) {
      for (const coordinates of g.coordinates) out.push({ ...f, id: undefined, geometry: { type: g.type.slice(5), coordinates } });
    } else if (g.type.startsWith('Multi')) {
      out.push({ ...f, geometry: { type: g.type.slice(5), coordinates: g.coordinates[0] } });
    } else {
      out.push(f);
    }
  }
  return out;
}

function importFeatures(fc) {
  const incoming = explode(fc.features || []).map((f) => {
    const properties = { ...(f.properties || {}) };
    SIMPLESTYLE.forEach((k) => delete properties[k]);
    return { type: 'Feature', id: f.id == null ? null : String(f.id), properties, geometry: roundGeometry(f.geometry) };
  });
  const taken = new Set(state.fc.features.map((f) => f.id));
  const keep = (f) => {  // a valid id of the right prefix that is still free
    const t = typeOf(f.properties);
    const m = ID_RE.exec(f.id || '');
    return t && m && m[1] === t.prefix && !taken.has(f.id);
  };
  for (const f of incoming) {
    if (!keep(f)) { f.id = null; continue; }
    taken.add(f.id);
    const [, prefix, n] = ID_RE.exec(f.id);
    state.fc.next_id[prefix] = Math.max(state.fc.next_id[prefix] || 0, Number(n) + 1);
  }
  for (const f of incoming) if (f.id == null) f.id = newId(typeOf(f.properties)?.prefix || 'x');
  if (!state.fc.origin && fc.origin) state.fc.origin = fc.origin;
  addFeatures(incoming);
  const unclassified = incoming.filter((f) => !typeOf(f.properties)).length;
  status(`Imported ${incoming.length} features${unclassified ? `, ${unclassified} unclassified (select them to assign a type)` : ''}`,
    unclassified ? 'warn' : 'ok');
  fitAll();
}

function readFile(onLoad) {
  const input = $('file-input');
  input.value = '';
  input.onchange = async () => {
    const file = input.files[0];
    if (!file) return;
    try {
      onLoad(JSON.parse(await file.text()), file.name);
    } catch (e) {
      status(`Could not read ${file.name}: ${e.message}`, 'error');
    }
  };
  input.click();
}

function openFile() {
  if (state.dirty && !window.confirm('Discard unsaved changes and open another file?')) return;
  readFile((fc, name) => {
    if (fc.type !== 'FeatureCollection') throw new Error('not a FeatureCollection');
    setCollection(fc, { dirty: true });
    fitAll();
    status(`Opened ${name}. Save writes it to ${state.path}`, 'warn');
  });
}

// --- keyboard, autosave ---------------------------------------------------------------------

function onKey(e) {
  const typing = ['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement?.tagName);
  if ((e.ctrlKey || e.metaKey) && e.key === 's') { e.preventDefault(); save(); return; }
  if (e.key === 'Escape' && typing) document.activeElement.blur();  // Esc always reaches the tools
  else if (typing) return;
  if ((e.ctrlKey || e.metaKey) && (e.key === 'z' || e.key === 'Z' || e.key === 'y')) {
    e.preventDefault();
    if (e.key === 'y' || e.shiftKey) redo(); else undo();
    return;
  }
  if (e.key === 'Escape') {
    if (isMeasuring()) stopMeasure();
    else if (isToolOpen()) closeTool();
    else if (isBusy()) { stopDraw(); setPick(null); } else select([]);
  } else if (e.key === 'Delete' || e.key === 'Backspace') {
    e.preventDefault();
    deleteSelection();
  }
}

let autosavedRevision = 0;
async function autosave() {
  if (!state.dirty || state.revision === autosavedRevision) return;
  try {
    await api.autosave(state.fc);
    autosavedRevision = state.revision;
  } catch { /* the next tick tries again */ }
}

function updateDirty() {
  const el = $('dirty');
  el.textContent = state.dirty ? '● unsaved' : '✓ saved';
  el.className = state.dirty ? 'warn' : 'ok';
}

// --- boot -----------------------------------------------------------------------------------

async function boot() {
  state.schema = await api.schema();
  initMap($('map'));
  initWorkflow();
  initPanels();

  on('dirty', updateDirty);
  try { state.live = localStorage.getItem('farmEditor.live') === '1'; } catch { /* storage blocked */ }
  let liveTimer = null;
  on('live', (enabled) => {
    state.live = enabled;
    try { localStorage.setItem('farmEditor.live', enabled ? '1' : '0'); } catch { /* storage blocked */ }
    if (enabled) validate({ quiet: true });
  });
  on('changed', () => {
    if (!state.live) return;
    clearTimeout(liveTimer);
    liveTimer = setTimeout(() => validate({ quiet: true }), 1000);
  });
  on('mode', (msg) => { $('mode').textContent = msg; });
  on('status', ({ msg, level }) => status(msg, level));
  on('featureClick', ({ id, shift }) => select([id], { add: shift }));
  on('mapClick', () => select([]));
  on('cursor', (ll) => {
    const frame = frameOf(state.fc);
    const xy = frame ? frame.enu([ll.lng, ll.lat]) : null;
    $('cursor').textContent = `${xy ? `x ${xy[0].toFixed(2)}  y ${xy[1].toFixed(2)} m   ` : ''}${ll.lat.toFixed(7)}, ${ll.lng.toFixed(7)}`;
  });
  on('history', () => { $('btn-undo').disabled = !canUndo(); $('btn-redo').disabled = !canRedo(); });
  on('changed', () => {
    const n = state.fc.features.length;
    $('counts').textContent = `${n} feature${n === 1 ? '' : 's'}`;
  });
  on('loaded', () => { $('counts').textContent = `${state.fc.features.length} features`; });

  $('btn-save').onclick = save;
  $('btn-undo').onclick = () => undo();
  $('btn-redo').onclick = () => redo();
  $('btn-measure').onclick = () => {
    if (isMeasuring()) { stopMeasure(); return; }
    closeTool();
    startMeasure((a, b) => {  // in a frame at the first point: exact for any distance on a farm
      const frame = new Frame(a.lat, a.lng);
      const [x, y] = frame.enu([b.lng, b.lat]);
      return `${Math.hypot(x, y).toFixed(2)} m, heading ${((Math.round(headingDeg(x, y) * 10) / 10) % 360).toFixed(1)}°`;
    });
  };
  $('btn-validate').onclick = () => validate();
  $('btn-export').onclick = exportFile;
  $('btn-open').onclick = openFile;
  $('btn-import').onclick = () => readFile((fc) => importFeatures(fc));
  $('goto').onkeydown = (e) => {
    if (e.key !== 'Enter') return;
    const m = /(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)/.exec(e.target.value);
    if (m) goTo(Number(m[1]), Number(m[2])); else status('Type "lat, lon", e.g. 46.3374, 3.434', 'warn');
  };
  document.addEventListener('keydown', onKey);
  window.addEventListener('beforeunload', (e) => { if (state.dirty) e.preventDefault(); });
  setInterval(autosave, 30000);

  let imageryWarning = '';
  try {
    const errors = addImagery(await api.imagery());
    if (errors.length) imageryWarning = `Imagery not loaded: ${errors.join('; ')}`;
  } catch (e) {
    imageryWarning = `Imagery list failed: ${e.message}`;
  }

  const farm = await api.farm();
  state.path = farm.path;
  $('filepath').textContent = farm.path;
  $('filepath').title = farm.exists ? 'working file' : 'new file, created on first save';
  setCollection(farm.fc);
  fitAll();
  if (!state.fc.features.length && imageryBounds().length) getMap().fitBounds(imageryBounds()[0]);
  if (farm.autosave_newer && window.confirm('An autosave newer than the file exists. Restore it?')) {
    setCollection((await api.autosaved()).fc, { dirty: true });
    fitAll();
  }
  if (imageryWarning) status(imageryWarning, 'warn');
  else status(farm.exists ? 'Loaded' : 'New farm: start with step 1', 'ok');
  if (!state.fc.features.some((f) => f.properties?.layer === 'farm')) {
    document.querySelector('#workflow details')?.setAttribute('open', '');
  }
  window.farmEditor = { state, byId, entryFor, select, map: getMap() };  // for debugging in the console
}

boot().catch((e) => status(`Startup failed: ${e.message}`, 'error'));
