// Service worker for the offline reader.
//
// Same-origin files (index.html, book.json, chapters, figures) are
// network-first: the book is still being edited, so an online reader must
// always see the latest deploy, with the cached copy as the offline fallback.
//
// Cross-origin files (marked, KaTeX and its fonts, Google Fonts) are
// cache-first: their URLs are version-pinned, so once cached they never
// need refetching. Bump CACHE only when those versions change.
const CACHE = 'mppi-book-v1';

const SHELL = [
  './',
  'index.html',
  'book.json',
];

const CDN = [
  'https://fonts.googleapis.com/css2?family=STIX+Two+Text:ital,wght@0,400;0,500;0,600;0,700;1,400;1,600&family=JetBrains+Mono:wght@400;500&display=swap',
  'https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css',
  'https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.js',
  'https://cdn.jsdelivr.net/npm/marked@12.0.2/marked.min.js',
];

self.addEventListener('install', (e) => {
  e.waitUntil((async () => {
    const c = await caches.open(CACHE);
    await c.addAll(SHELL);
    await Promise.all(CDN.map(async (u) => {
      try {
        const res = await fetch(u);
        if (res.ok) await c.put(u, res);
      } catch {}
    }));
    await self.skipWaiting();
  })());
});

self.addEventListener('activate', (e) => {
  e.waitUntil(
    caches.keys()
      .then(keys => Promise.all(keys.filter(k => k !== CACHE).map(k => caches.delete(k))))
      .then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', (e) => {
  const req = e.request;
  if (req.method !== 'GET') return;
  const sameOrigin = new URL(req.url).origin === self.location.origin;
  e.respondWith(sameOrigin ? networkFirst(req) : cacheFirst(req));
});

// The cache write is awaited so that once the page has a response, the file
// is guaranteed to be in Cache Storage — the "Save for offline" button relies
// on that to verify each download.
async function store(req, res) {
  try { await (await caches.open(CACHE)).put(req, res); } catch {}
}

async function networkFirst(req) {
  try {
    // {cache: 'reload'} skips the HTTP cache, which on GitHub Pages would
    // otherwise serve a copy up to ten minutes old.
    const res = await fetch(req, { cache: 'reload' });
    if (res.ok) await store(req, res.clone());
    return res;
  } catch (err) {
    const cached = await caches.match(req, { ignoreSearch: true });
    if (cached) return cached;
    if (req.mode === 'navigate') {
      const fallback = await caches.match('index.html');
      if (fallback) return fallback;
    }
    throw err;
  }
}

async function cacheFirst(req) {
  const cached = await caches.match(req);
  if (cached) return cached;
  const res = await fetch(req);
  if (res.ok || res.type === 'opaque') await store(req, res.clone());
  return res;
}
