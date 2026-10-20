// Fetch wrappers for the server API (docs/guides/FARM_EDITOR.md §6).

async function call(method, url, body) {
  const r = await fetch(url, {
    method,
    headers: body ? { 'Content-Type': 'application/json' } : {},
    body: body ? JSON.stringify(body) : undefined,
  });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.error || `${method} ${url}: HTTP ${r.status}`);
  return data;
}

export const api = {
  schema: () => call('GET', '/api/schema'),
  farm: () => call('GET', '/api/farm'),
  imagery: () => call('GET', '/api/imagery'),
  autosaved: () => call('GET', '/api/autosave'),
  save: (fc) => call('PUT', '/api/farm', fc),
  autosave: (fc) => call('POST', '/api/autosave', fc),
  validate: (fc) => call('POST', '/api/validate', fc),
  generate: (what, body) => call('POST', `/api/generate/${what}`, body),
};
