/* Runs synchronously from <head>, before the page is painted.
   Resolves the colour theme and the language redirect up front, so a page
   never flashes in the wrong theme or the wrong language. */
(function () {
  var root = document.documentElement;
  root.classList.add('js');

  function read(key) {
    try { return window.localStorage.getItem(key); } catch (e) { return null; }
  }

  // Theme: an explicit choice wins, otherwise follow the system.
  var theme = read('theme');
  if (theme !== 'light' && theme !== 'dark') {
    theme = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
  }
  root.setAttribute('data-theme', theme);

  // Language: a stored choice wins; with none, Russian browsers get the
  // Russian pages. The query string and #anchor survive the redirect.
  var file = window.location.pathname.split('/').pop() || 'index.html';
  if (!/\.html$/.test(file)) return;

  var isRu = /_ru\.html$/.test(file);
  var stored = read('lang');
  var browserRu = (navigator.language || '').toLowerCase().indexOf('ru') === 0;
  var want = stored === 'ru' || stored === 'en' ? stored : (browserRu ? 'ru' : null);

  var target = null;
  if (want === 'ru' && !isRu) target = file.replace(/\.html$/, '_ru.html');
  if (want === 'en' && isRu) target = file.replace(/_ru\.html$/, '.html');
  if (target) window.location.replace(target + window.location.search + window.location.hash);
})();
