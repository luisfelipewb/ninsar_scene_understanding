// Derived tree (farm → areas → rows → objects), overlays and loose ends, from the last
// validation. Children render when a node is first opened, so 10k plants stay cheap.
import { zoomTo } from './map.js';
import { h } from './dom.js';
import { byId, labelOf, select, state, typeOf } from './state.js';

const PAGE = 100;

function goto(id) {
  return (e) => { e.preventDefault(); e.stopPropagation(); select([id]); zoomTo([id]); };
}

function link(id) {
  const f = byId(id);
  const issue = state.issuesById.get(id);
  return h('a', { href: '#', class: `tree-link${issue ? ` has-${issue.level}` : ''}`, onclick: goto(id),
    title: issue ? issue.list.map((i) => `${i.rule} ${i.message}`).join('\n') : null },
  f ? labelOf(f) : id);
}

function area(m2) { return m2 >= 1e4 ? `${(m2 / 1e4).toFixed(2)} ha` : `${Math.round(m2)} m²`; }

function summary(id) {
  const f = byId(id);
  const d = state.derived[id] || {};
  const bits = [];
  if (d.area_m2 != null) bits.push(area(d.area_m2));
  if (f?.properties.kind === 'parcel' && d.n_rows) bits.push(`${d.n_rows} rows (${d.crops.join(', ')})`);
  if (f?.properties.layer === 'row') bits.push(`${d.n_plants} plants, ${d.length_m.toFixed(1)} m`);
  if (d.overlays?.length) bits.push(`in ${d.overlays.join(', ')}`);
  return bits.length ? h('span', { class: 'tree-sum' }, bits.join(' · ')) : null;
}

function idList(ids) {
  const out = h('span', { class: 'tree-ids' });
  ids.slice(0, 50).forEach((id, i) => { if (i) out.append(', '); out.append(h('a', { href: '#', onclick: goto(id) }, id)); });
  if (ids.length > 50) out.append(` … ${ids.length - 50} more`);
  return ids.length ? out : h('span', { class: 'muted' }, 'none');
}

function fillChildren(container, kids, from = 0) {
  const shown = kids.slice(from, from + PAGE);
  shown.forEach((k) => container.append(node(k)));
  if (from + PAGE < kids.length) {
    const more = h('button', { type: 'button', class: 'small', onclick: () => { more.remove(); fillChildren(container, kids, from + PAGE); } },
      `Show ${Math.min(PAGE, kids.length - from - PAGE)} more of ${kids.length - from - PAGE}`);
    container.append(more);
  }
}

function node(id, open = false) {
  const kids = state.derived[id]?.children || [];
  if (!kids.length) return h('div', { class: 'tree-leaf' }, link(id), summary(id));
  const counts = {};
  for (const k of kids) { const kind = byId(k)?.properties.kind; counts[kind] = (counts[kind] || 0) + 1; }
  const tally = Object.entries(counts).map(([k, n]) => `${n} ${k}`).join(', ');
  const body = h('div', { class: 'tree-children' });
  const det = h('details', { class: 'tree-node', open },
    h('summary', {}, link(id), summary(id), h('span', { class: 'tree-tally' }, tally)), body);
  const fill = () => { if (det.open && !det.filled) { det.filled = true; fillChildren(body, kids); } };
  det.addEventListener('toggle', fill);
  fill();
  return det;
}

function section(title, count, ...content) {
  return h('details', { class: 'tree-section', open: count > 0 && count <= 20 },
    h('summary', {}, `${title} `, h('span', { class: 'muted' }, String(count))), ...content);
}

export function renderTree(el) {
  const d = state.derived;
  if (!Object.keys(d).length) {
    el.append(h('p', { class: 'muted' }, 'Validate to build the tree.'));
    return;
  }
  const features = state.fc.features.filter((f) => d[f.id]);
  const farms = features.filter((f) => f.properties.layer === 'farm');
  farms.forEach((f) => el.append(node(f.id, true)));
  if (!farms.length) el.append(h('p', { class: 'error' }, 'No farm boundary: nothing has a parent.'));

  const overlays = features.filter((f) => f.properties.layer === 'overlay');
  el.append(section('Overlays', overlays.length, overlays.map((z) => h('div', { class: 'tree-overlay' },
    link(z.id), h('div', { class: 'tree-members' }, 'members: ', idList(d[z.id].members || []),
      d[z.id].roads?.length ? h('span', {}, '; roads: ', idList(d[z.id].roads)) : null)))));

  const roads = features.filter((f) => f.properties.layer === 'navigation');
  el.append(section('Roads', roads.length, roads.map((r) => h('div', { class: 'tree-leaf' }, link(r.id)))));

  const loose = features.filter((f) => ['area', 'row', 'object'].includes(f.properties.layer) && !d[f.id].parent);
  el.append(section('Outside the farm', loose.length, h('div', {}, idList(loose.map((f) => f.id)))));

  const skipped = state.fc.features.filter((f) => !d[f.id]);
  const unclassified = skipped.filter((f) => !typeOf(f.properties));
  el.append(section('Not in the tree (invalid or unclassified)', skipped.length,
    h('div', {}, idList(skipped.map((f) => f.id))),
    unclassified.length ? h('p', { class: 'muted' }, `${unclassified.length} unclassified: select them to assign a type.`) : null));
}
