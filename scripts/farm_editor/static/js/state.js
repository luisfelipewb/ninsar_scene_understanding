// Editor state and the feature model. Every change goes through addFeatures /
// updateFeature / removeFeatures, which emit "changed" so the map and panels follow.

export const state = {
  schema: null,
  fc: null,
  path: '',
  derived: {},
  issues: [],
  issuesById: new Map(),  // feature id -> {level: worst level, list}
  derivedStale: true,
  selection: new Set(),
  dirty: false,
  live: false,     // validate automatically after changes
  revision: 0,  // bumped on every change; autosave compares it
};

const listeners = {};
export function on(evt, fn) { (listeners[evt] ||= []).push(fn); }
export function emit(evt, arg) { (listeners[evt] || []).forEach((fn) => fn(arg)); }

export const LAYER_ORDER = ['farm', 'area', 'overlay', 'row', 'navigation', 'object'];  // draw order
export const ID_RE = /^([A-Za-z])(\d+)$/;
export const COMMON_KEYS = new Set(['layer', 'category', 'kind', 'name', 'description']);

// --- schema helpers -----------------------------------------------------------------

export function typeOf(props) {
  if (!props) return null;
  return state.schema.types.find((t) => t.layer === props.layer && t.category === (props.category ?? null)
    && t.kinds.includes(props.kind)) || null;
}

export function entryFor(layer, category = null, kind = null) {
  return state.schema.types.find((t) => t.layer === layer && t.category === (category ?? null)
    && (kind == null || t.kinds.includes(kind))) || null;
}

export function colorOf(props) {
  const crop = state.schema.crops[props?.kind];
  if (crop && (props.layer === 'row' || props.category === 'crop')) return crop.color;
  return typeOf(props)?.color || '#888888';
}

export function labelOf(f) {
  const p = f.properties || {};
  return `${f.id} ${p.kind || 'unclassified'}${p.name ? ` "${p.name}"` : ''}`;
}

// --- ids (same rule as farm_geojson.new_id) --------------------------------------------

export function newId(prefix) { return newIds(prefix, 1)[0]; }

export function newIds(prefix, count) {
  let used = 0;
  for (const f of state.fc.features) {
    const m = ID_RE.exec(f.id || '');
    if (m && m[1] === prefix) used = Math.max(used, Number(m[2]) + 1);
  }
  const n = Math.max(state.fc.next_id[prefix] || 0, used);
  state.fc.next_id[prefix] = n + count;
  return Array.from({ length: count }, (_, i) => `${prefix}${n + i}`);
}

// --- collection -----------------------------------------------------------------------

let index = new Map();

export function byId(id) { return index.get(id); }

export function normalize(fc) {
  fc.type = 'FeatureCollection';
  fc.farm_geojson ??= state.schema.version;
  fc.site ??= '';
  fc.features ??= [];
  if (!fc.next_id) {
    fc.next_id = {};
    for (const f of fc.features) {
      const m = ID_RE.exec(f.id || '');
      if (m) fc.next_id[m[1]] = Math.max(fc.next_id[m[1]] || 0, Number(m[2]) + 1);
    }
  }
  return fc;
}

export function setCollection(fc, { dirty = false } = {}) {
  state.fc = normalize(fc);
  index = new Map(state.fc.features.map((f) => [f.id, f]));
  state.selection.clear();
  state.derived = {};
  state.issues = [];
  state.issuesById = new Map();
  state.derivedStale = true;
  state.dirty = dirty;
  state.revision += 1;
  history.undo = [];
  history.redo = [];
  emit('loaded');
  emit('history');
  emit('selection');
  emit('dirty');
}

function touched() {
  state.dirty = true;
  state.derivedStale = true;
  state.revision += 1;
  emit('dirty');
}

export function addFeatures(features) {
  record({ type: 'add', features: clone(features) });
  for (const f of features) {
    state.fc.features.push(f);
    index.set(f.id, f);
  }
  touched();
  emit('changed', { added: features.map((f) => f.id) });
}

// changes: {properties?: {...replace}, geometry?, id?}. `source` tells the map what to redraw:
// "map" (it already shows the new geometry), "props" (restyle only), or null (rebuild the layer).
export function updateFeature(id, changes, source = null) {
  updateFeatures([[id, changes]], source);
}

export function updateFeatures(list, source = null) {
  const updated = [];
  const renamed = {};
  const before = [];
  const after = [];
  for (const [id, changes] of list) {
    const f = index.get(id);
    if (!f) continue;
    before.push(snapshot(f));
    if (changes.properties) f.properties = changes.properties;
    if (changes.geometry) f.geometry = roundGeometry(changes.geometry);
    if (changes.id && changes.id !== id) {
      index.delete(id);
      f.id = changes.id;
      index.set(f.id, f);
      renamed[id] = f.id;
      if (state.selection.delete(id)) state.selection.add(f.id);
    }
    updated.push(f.id);
    after.push(snapshot(f));
  }
  if (!updated.length) return;
  record({ type: 'update', before, after });
  touched();
  emit('changed', { updated, renamed, source });
}

export function removeFeatures(ids) {
  record({ type: 'remove', features: clone(ids.map((id) => index.get(id)).filter(Boolean)) });
  const gone = new Set(ids);
  state.fc.features = state.fc.features.filter((f) => !gone.has(f.id));
  ids.forEach((id) => { index.delete(id); state.selection.delete(id); });
  touched();
  emit('changed', { removed: ids });
  emit('selection');
}

export function select(ids, { add = false } = {}) {
  if (!add) state.selection.clear();
  for (const id of ids) {
    if (add && state.selection.has(id)) state.selection.delete(id);
    else state.selection.add(id);
  }
  emit('selection');
}

export function markSaved() {
  state.dirty = false;
  emit('dirty');
}

// `revision`: the state.revision the report was computed for; a newer edit keeps it stale
export function setReport({ issues, derived }, revision = state.revision) {
  const before = new Set(state.issuesById.keys());
  state.issues = issues;
  state.issuesById = new Map();
  for (const i of issues) {
    if (!i.id) continue;
    const entry = state.issuesById.get(i.id) || { level: 'warning', list: [] };
    entry.list.push(i);
    if (i.level === 'error') entry.level = 'error';
    state.issuesById.set(i.id, entry);
  }
  state.derived = derived;
  state.derivedStale = revision !== state.revision;
  emit('report', { touched: [...new Set([...before, ...state.issuesById.keys()])] });
}

// --- geometry -------------------------------------------------------------------------

const round8 = (v) => Math.round(v * 1e8) / 1e8;
const roundCoords = (c) => (typeof c === 'number' ? round8(c) : c.map(roundCoords));

export function roundGeometry(g) {
  return { type: g.type, coordinates: roundCoords(g.coordinates) };
}

// --- undo / redo ----------------------------------------------------------------------
// Each step is a list of operations with what is needed to invert them (not snapshots of
// the whole collection, which would cost megabytes per step with thousands of plants).
// Ids are never handed back on undo: next_id only grows (GeoJSON spec §4.1).

const history = { undo: [], redo: [], group: null, replaying: false, limit: 200 };

function clone(x) { return JSON.parse(JSON.stringify(x)); }
function snapshot(f) { return { id: f.id, properties: clone(f.properties), geometry: clone(f.geometry) }; }

function record(op) {
  if (history.replaying) return;
  if (history.group) { history.group.push(op); return; }
  pushStep([op]);
}

function pushStep(ops) {
  if (!ops.length) return;
  history.undo.push(ops);
  if (history.undo.length > history.limit) history.undo.shift();
  history.redo = [];
  emit('history');
}

// run `fn` as one undo step
export function group(fn) {
  if (history.group) return fn();
  history.group = [];
  try {
    return fn();
  } finally {
    const ops = history.group;
    history.group = null;
    pushStep(ops);
  }
}

function replay(op, forward) {
  const ids = (features) => features.map((f) => f.id);
  if (op.type === 'update') {
    const [from, to] = forward ? [op.before, op.after] : [op.after, op.before];
    updateFeatures(from.map((f, i) => [f.id, { properties: clone(to[i].properties), geometry: to[i].geometry, id: to[i].id }]));
  } else if ((op.type === 'add') === forward) {
    addFeatures(clone(op.features));
  } else {
    removeFeatures(ids(op.features));
  }
}

function step(fromStack, toStack, forward) {
  const ops = fromStack.pop();
  if (!ops) return false;
  history.replaying = true;
  try {
    (forward ? ops : [...ops].reverse()).forEach((op) => replay(op, forward));
  } finally {
    history.replaying = false;
  }
  toStack.push(ops);
  emit('history');
  return true;
}

export const undo = () => step(history.undo, history.redo, false);
export const redo = () => step(history.redo, history.undo, true);
export const canUndo = () => history.undo.length > 0;
export const canRedo = () => history.redo.length > 0;
