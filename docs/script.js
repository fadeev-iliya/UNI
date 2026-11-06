'use strict';

/* ==========================================================================
   UNI Documentation — page shell
   Builds the sidebar, header, table of contents, code frames and search
   dialog around the static content in each HTML file. boot.js, loaded from
   <head>, has already settled the theme and the language by this point.
   ========================================================================== */

const currentPath = window.location.pathname;
const currentFile = (currentPath.split('/').pop() || 'index.html').split('#')[0] || 'index.html';
const isRuPage = /_ru\.html$/.test(currentFile);
const lang = isRuPage ? 'ru' : 'en';

const REPO_URL = 'https://github.com/fadeev-iliya/UNI';

const T = {
  en: {
    search: 'Search...',
    searchPlaceholder: 'Search documentation',
    noResults: 'No results found',
    searchHint: 'Search every page: methods, examples, sections',
    onThisPage: 'On this page',
    copyPage: 'Copy page',
    copied: 'Copied',
    getStarted: 'Get started',
    previous: 'Previous',
    next: 'Next',
    github: 'GitHub',
    otherLang: 'Русский',
    system: 'System theme',
    light: 'Light theme',
    dark: 'Dark theme',
    menu: 'Open navigation',
    copy: 'Copy',
    expandAll: 'Expand all',
    collapseAll: 'Collapse all',
    linkHere: 'Link to this section',
    soon: 'soon',
    searching: 'Searching the other pages…',
    types: { method: 'Method', section: 'Section', page: 'Page', example: 'Example', topic: 'Topic' },
  },
  ru: {
    search: 'Поиск...',
    searchPlaceholder: 'Поиск по документации',
    noResults: 'Ничего не найдено',
    searchHint: 'Поиск по всем страницам: методы, примеры, разделы',
    onThisPage: 'На этой странице',
    copyPage: 'Копировать',
    copied: 'Скопировано',
    getStarted: 'Начало',
    previous: 'Назад',
    next: 'Далее',
    github: 'GitHub',
    otherLang: 'English',
    system: 'Системная тема',
    light: 'Светлая тема',
    dark: 'Тёмная тема',
    menu: 'Открыть навигацию',
    copy: 'Копировать',
    expandAll: 'Развернуть все',
    collapseAll: 'Свернуть все',
    linkHere: 'Ссылка на этот раздел',
    soon: 'скоро',
    searching: 'Ищу на остальных страницах…',
    types: { method: 'Метод', section: 'Раздел', page: 'Страница', example: 'Пример', topic: 'Тема' },
  },
}[lang];

/* --- Icons ---------------------------------------------------------------- */

const svg = (paths, size = 16, extra = '') =>
  `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" ` +
  `stroke-width="2" stroke-linecap="round" stroke-linejoin="round" ${extra}>${paths}</svg>`;

const ICONS = {
  search: svg('<circle cx="11" cy="11" r="7"/><line x1="21" y1="21" x2="16.65" y2="16.65"/>'),
  menu: svg('<line x1="3" y1="6" x2="21" y2="6"/><line x1="3" y1="12" x2="21" y2="12"/><line x1="3" y1="18" x2="21" y2="18"/>', 18),
  chevron: svg('<polyline points="9 18 15 12 9 6"/>', 14),
  monitor: svg('<rect x="2" y="3" width="20" height="14" rx="2"/><line x1="8" y1="21" x2="16" y2="21"/><line x1="12" y1="17" x2="12" y2="21"/>', 14),
  sun: svg('<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.93 4.93l1.41 1.41M17.66 17.66l1.41 1.41M2 12h2M20 12h2M4.93 19.07l1.41-1.41M17.66 6.34l1.41-1.41"/>', 14),
  moon: svg('<path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"/>', 14),
  copy: svg('<rect x="9" y="9" width="13" height="13" rx="2"/><path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"/>', 14),
  check: svg('<polyline points="20 6 9 17 4 12"/>', 14),
  list: svg('<line x1="8" y1="6" x2="21" y2="6"/><line x1="8" y1="12" x2="21" y2="12"/><line x1="8" y1="18" x2="21" y2="18"/><line x1="3" y1="6" x2="3.01" y2="6"/><line x1="3" y1="12" x2="3.01" y2="12"/><line x1="3" y1="18" x2="3.01" y2="18"/>', 14),
  external: svg('<path d="M18 13v6a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h6"/><polyline points="15 3 21 3 21 9"/><line x1="10" y1="14" x2="21" y2="3"/>', 13),
  globe: svg('<circle cx="12" cy="12" r="10"/><line x1="2" y1="12" x2="22" y2="12"/><path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z"/>', 14),
  github: svg('<path d="M9 19c-5 1.5-5-2.5-7-3m14 6v-3.87a3.37 3.37 0 0 0-.94-2.61c3.14-.35 6.44-1.54 6.44-7A5.44 5.44 0 0 0 20 4.77 5.07 5.07 0 0 0 19.91 1S18.73.65 16 2.48a13.38 13.38 0 0 0-7 0C6.27.65 5.09 1 5.09 1A5.07 5.07 0 0 0 5 4.77a5.44 5.44 0 0 0-1.5 3.78c0 5.42 3.3 6.61 6.44 7A3.37 3.37 0 0 0 9 18.13V22"/>', 14),
  link: svg('<path d="M10 13a5 5 0 0 0 7.54.54l3-3a5 5 0 0 0-7.07-7.07l-1.72 1.71"/><path d="M14 11a5 5 0 0 0-7.54-.54l-3 3a5 5 0 0 0 7.07 7.07l1.71-1.71"/>', 14),
  unfold: svg('<polyline points="7 13 12 18 17 13"/><polyline points="7 6 12 11 17 6"/>', 14),
  fold: svg('<polyline points="17 11 12 6 7 11"/><polyline points="17 18 12 13 7 18"/>', 14),
  note: svg('<circle cx="12" cy="12" r="10"/><line x1="12" y1="16" x2="12" y2="12"/><line x1="12" y1="8" x2="12.01" y2="8"/>', 16),
  tip: svg('<path d="M9 18h6M10 22h4"/><path d="M12 2a7 7 0 0 0-4 12.74V16a1 1 0 0 0 1 1h6a1 1 0 0 0 1-1v-1.26A7 7 0 0 0 12 2z"/>', 16),
  warning: svg('<path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/>', 16),
};

const NAV_ICONS = {
  home: svg('<path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><polyline points="9 22 9 12 15 12 15 22"/>', 15),
  start: svg('<polygon points="5 3 19 12 5 21 5 3"/>', 15),
  unibase: svg('<rect x="2" y="7" width="20" height="14" rx="2"/><path d="M16 7V5a2 2 0 0 0-2-2h-4a2 2 0 0 0-2 2v2"/><line x1="12" y1="12" x2="12" y2="16"/><line x1="10" y1="14" x2="14" y2="14"/>', 15),
  unidev: svg('<circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M4.22 4.22l2.12 2.12M17.66 17.66l2.12 2.12M2 12h3M19 12h3M4.22 19.78l2.12-2.12M17.66 6.34l2.12-2.12"/>', 15),
  overview: svg('<path d="M2 3h6a4 4 0 0 1 4 4v14a3 3 0 0 0-3-3H2z"/><path d="M22 3h-6a4 4 0 0 0-4 4v14a3 3 0 0 1 3-3h7z"/>', 15),
  examples: svg('<polyline points="16 18 22 12 16 6"/><polyline points="8 6 2 12 8 18"/>', 15),
  limits: svg('<path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/>', 15),
  faq: svg('<circle cx="12" cy="12" r="10"/><path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"/><line x1="12" y1="17" x2="12.01" y2="17"/>', 15),
  hardware: svg('<rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/><path d="M9 1v3M15 1v3M9 20v3M15 20v3M20 9h3M20 14h3M1 9h3M1 14h3"/>', 15),
  lesson: svg('<path d="M4 15s1-1 4-1 5 2 8 2 4-1 4-1V3s-1 1-4 1-5-2-8-2-4 1-4 1z"/><line x1="4" y1="22" x2="4" y2="15"/>', 15),
  ports: svg('<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/>', 15),
  attachment: svg('<path d="M14.7 6.3a1 1 0 0 0 0 1.4l1.6 1.6a1 1 0 0 0 1.4 0l3.77-3.77a6 6 0 0 1-7.94 7.94l-6.91 6.91a2.12 2.12 0 0 1-3-3l6.91-6.91a6 6 0 0 1 7.94-7.94l-3.76 3.76z"/>', 15),
  protocol: svg('<polyline points="17 1 21 5 17 9"/><path d="M3 11V9a4 4 0 0 1 4-4h14"/><polyline points="7 23 3 19 7 15"/><path d="M21 13v2a4 4 0 0 1-4 4H3"/>', 15),
  reference: svg('<path d="M8 3H7a2 2 0 0 0-2 2v5a2 2 0 0 1-2 2 2 2 0 0 1 2 2v5a2 2 0 0 0 2 2h1"/><path d="M16 21h1a2 2 0 0 0 2-2v-5a2 2 0 0 1 2-2 2 2 0 0 1-2-2V5a2 2 0 0 0-2-2h-1"/>', 15),
  library: svg('<path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20"/><path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z"/>', 15),
  attachments: svg('<path d="M12 2 2 7l10 5 10-5-10-5z"/><path d="m2 17 10 5 10-5"/><path d="m2 12 10 5 10-5"/>', 15),
  light: svg('<path d="M9 18h6M10 22h4"/><path d="M12 2a7 7 0 0 0-4 12.74V16a1 1 0 0 0 1 1h6a1 1 0 0 0 1-1v-1.26A7 7 0 0 0 12 2z"/>', 15),
  sensor: svg('<polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/>', 15),
  ide: svg('<polyline points="4 17 10 11 4 5"/><line x1="12" y1="19" x2="20" y2="19"/>', 15),
};

const LOGO_SVG = '<svg class="logo-icon" width="20" height="24" viewBox="0 0 168 200" fill="none" xmlns="http://www.w3.org/2000/svg"><path d="M65.0275 93.6581L54.9206 82.2931C54.1615 81.4395 53.0738 80.9512 51.9316 80.9512H51C48.7909 80.9512 47 79.1604 47 76.9512V75C47 72.7909 48.7909 71 51 71H117C119.209 71 121 72.7909 121 75V76.9512C121 79.1604 119.209 80.9512 117 80.9512H116.068C114.926 80.9512 113.838 81.4395 113.079 82.2931L102.973 93.6581C102.213 94.5117 101.126 95 99.9835 95H68.0165C66.8743 95 65.7865 94.5117 65.0275 93.6581Z" fill="currentColor"/><path d="M47.4365 96H40C38.3431 96 37 97.3431 37 99V115.896C37 117.39 38.099 118.656 39.5779 118.866L54.5926 121H114.407L129.422 118.866C130.901 118.656 132 117.39 132 115.896V99C132 97.3431 130.657 96 129 96H121.015C120.168 96 119.361 96.3579 118.792 96.9853L107.091 109.896C106.522 110.523 105.715 110.881 104.868 110.881H64.0944C63.2697 110.881 62.4814 110.541 61.9148 109.942L49.6161 96.9386C49.0494 96.3395 48.2612 96 47.4365 96Z" fill="currentColor"/><path d="M132 172C132 174.209 130.209 176 128 176H41C38.7909 176 37 174.209 37 172V115C37 112.791 38.7909 111 41 111H128C130.209 111 132 112.791 132 115V172ZM58 128C55.7909 128 54 129.791 54 132V154C54 156.209 55.7909 158 58 158H111C113.209 158 115 156.209 115 154V132C115 129.791 113.209 128 111 128H58Z" fill="currentColor"/><path d="M128 24C130.209 24 132 25.7909 132 28V77C132 79.2091 130.209 81 128 81H41C38.7909 81 37 79.2091 37 77V28C37 25.7909 38.7909 24 41 24H128ZM58 38C55.7909 38 54 39.7909 54 42V64C54 66.2091 55.7909 68 58 68H111C113.209 68 115 66.2091 115 64V42C115 39.7909 113.209 38 111 38H58Z" fill="currentColor"/></svg>';

function navIcon(href) {
  const f = (href || '').split('#')[0];
  if (/^index/.test(f)) return NAV_ICONS.home;
  if (/getting-started/.test(f)) return NAV_ICONS.start;
  if (/^hardware-unidev/.test(f)) return NAV_ICONS.ports;
  if (/^hardware-attachment/.test(f)) return NAV_ICONS.attachment;
  if (/^hardware/.test(f)) return NAV_ICONS.hardware;
  if (/^protocol/.test(f)) return NAV_ICONS.protocol;
  if (/^reference/.test(f)) return NAV_ICONS.reference;
  if (/^tutorial-/.test(f)) return NAV_ICONS.lesson;
  if (/^unibase/.test(f)) return NAV_ICONS.unibase;
  if (/^unidev/.test(f)) return NAV_ICONS.unidev;
  if (/library-overview/.test(f)) return NAV_ICONS.overview;
  if (/^examples/.test(f)) return NAV_ICONS.examples;
  if (/limitations/.test(f)) return NAV_ICONS.limits;
  if (/faq/.test(f)) return NAV_ICONS.faq;
  return '';
}

const navItems = () => (window.NAV && window.NAV[lang]) || [];

// Every page in the navigation, in reading order. A page inside a fold-out
// group takes the group's title as its section, so the breadcrumb reads
// "UNI.h library › UniBase class". Announced items and the highlighted links
// above the sections are not pages of their own.
function navPages() {
  const pages = [];
  // Pages above the first heading belong to "Get started", as the
  // breadcrumb has always called them.
  let section = T.getStarted;
  navItems().forEach((item) => {
    if (item.section) {
      section = item.section;
    } else if (item.group) {
      (item.items || []).forEach((sub) => {
        if (sub.href && !sub.soon) pages.push({ item: sub, section: item.group });
      });
    } else if (item.href && !item.soon && !item.anchor) {
      pages.push({ item, section });
    }
  });
  return pages;
}

/* ==========================================================================
   Small helpers
   ========================================================================== */

// localStorage throws in some privacy modes; a failed read or write is not
// worth breaking the page for.
const store = {
  get(key) {
    try { return localStorage.getItem(key); } catch (e) { return null; }
  },
  set(key, value) {
    try {
      if (value === null) localStorage.removeItem(key);
      else localStorage.setItem(key, value);
    } catch (e) { /* ignore */ }
  },
};

const isMac = /Mac|iPhone|iPad|iPod/.test(navigator.platform || '');
const clean = (s) => (s || '').replace(/\s+/g, ' ').trim();
const escapeHtml = (s) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const escapeRe = (s) => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
const headerHeight = () =>
  parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--header-height')) || 56;

function copyText(text) {
  if (!navigator.clipboard || !navigator.clipboard.writeText) return Promise.resolve();
  return navigator.clipboard.writeText(text).catch(() => {});
}

// What the scroll position currently points at; kept up to date by the
// scroll spy and read by the breadcrumb and the language switch.
const state = { generated: new Set(), groupId: '', entryId: '', pin: null };

/* ==========================================================================
   Theme — system / light / dark. boot.js applies it before first paint;
   this only wires up the switch.
   ========================================================================== */

const mql = window.matchMedia('(prefers-color-scheme: dark)');

function themePref() {
  const v = store.get('theme');
  return v === 'light' || v === 'dark' ? v : 'system';
}

function applyTheme() {
  const pref = themePref();
  const resolved = pref === 'system' ? (mql.matches ? 'dark' : 'light') : pref;
  document.documentElement.setAttribute('data-theme', resolved);
  document.querySelectorAll('.theme-seg button').forEach((b) => {
    b.setAttribute('aria-pressed', String(b.dataset.themeSet === pref));
  });
}

mql.addEventListener('change', () => {
  if (themePref() === 'system') applyTheme();
});

// Paper is white: print in the light palette whatever the screen theme is.
window.addEventListener('beforeprint', () => document.documentElement.setAttribute('data-theme', 'light'));
window.addEventListener('afterprint', applyTheme);

/* ==========================================================================
   Language. The redirect itself lives in boot.js.
   ========================================================================== */

function langHref(target) {
  if (target === 'ru') return isRuPage ? currentFile : currentFile.replace(/\.html$/, '_ru.html');
  return isRuPage ? currentFile.replace(/_ru\.html$/, '.html') : currentFile;
}

// An anchor that exists in both language versions, so switching language
// keeps the reader's place. Authored ids and method ids match across the
// two; ids slugged from heading text do not.
function portableHash() {
  if (window.scrollY < 200) return '';
  const ok = (id) => id && (!state.generated.has(id) || id.startsWith('api-'));
  if (ok(state.entryId)) return '#' + state.entryId;
  if (ok(state.groupId)) return '#' + state.groupId;
  return '';
}

/* ==========================================================================
   Anchors
   ========================================================================== */

// Keeps Cyrillic as well as Latin, so Russian headings get real anchors.
function slugify(text) {
  return text.toLowerCase().trim()
    .replace(/[^\p{L}\p{N}\s-]/gu, '')
    .replace(/\s+/g, '-')
    .replace(/-{2,}/g, '-')
    .replace(/^-+|-+$/g, '');
}

// Gives every section heading, method and topic card an id. It runs on the
// live page and on the pages parsed for the search index, and must arrive at
// the same anchors in both — so it depends only on the authored markup.
function assignIds(root) {
  const used = new Set();
  root.querySelectorAll('[id]').forEach((el) => used.add(el.id));
  const generated = new Set();

  const claim = (el, base) => {
    let id = base;
    for (let n = 2; used.has(id); n += 1) id = `${base}-${n}`;
    used.add(id);
    generated.add(id);
    el.id = id;
  };

  let sections = 0;
  root.querySelectorAll('.page-content h2, .page-content .api-card').forEach((el) => {
    if (el.tagName === 'H2') sections += 1;
    if (el.id) return;
    if (el.tagName === 'H2') {
      claim(el, slugify(el.textContent) || `section-${sections}`);
      return;
    }
    const name = el.querySelector('.api-name');
    if (name) {
      // "getLeftTicks / getRightTicks" and "displayPrint (Named)" both key
      // on their first identifier, which is the same in every language.
      const ident = name.textContent.match(/[A-Za-z_]\w*/);
      claim(el, 'api-' + (ident ? ident[0] : 'method'));
      return;
    }
    const h3 = el.querySelector('h3');
    if (h3) claim(el, slugify(h3.textContent) || 'topic');
  });

  return generated;
}

function revealId(id, instant) {
  const el = id && document.getElementById(id);
  if (!el) return false;
  const card = el.closest('.api-card:not(.static)');
  if (card) setCardOpen(card, true);
  const top = el.getBoundingClientRect().top + window.scrollY - headerHeight() - 20;
  window.scrollTo({ top, behavior: instant ? 'instant' : 'smooth' });
  return true;
}

function hashId() {
  try { return decodeURIComponent(window.location.hash.slice(1)); } catch (e) { return ''; }
}

/* ==========================================================================
   Sidebar
   ========================================================================== */

function renderSidebar() {
  const aside = document.querySelector('.sidebar');
  if (!aside) return;

  const currentHash = hashId();
  const homeHref = isRuPage ? 'index_ru.html' : 'index.html';

  // What sits under a page: the list from nav.js or, for the page being
  // read, its own section headings. Where the table of contents is hidden
  // this is the only way around the page, so no page should go without.
  const subItems = (item, isActive) => {
    if (item.children && item.children.length) {
      return item.children.map((c) => ({ label: c.label, href: c.href, anchor: c.href.split('#')[1] || '' }));
    }
    const heads = isActive ? [...document.querySelectorAll('.page-content h2[id]')] : [];
    if (heads.length < 3) return [];
    return heads.map((h) => ({
      label: escapeHtml(clean(h.textContent)),
      href: `${item.href}#${encodeURIComponent(h.id)}`,
      anchor: escapeHtml(h.id),
    }));
  };

  let nav = '';
  let openList = false;
  const closeList = () => {
    if (openList) {
      nav += '</ul>';
      openList = false;
    }
  };

  const iconSpan = (icon) => (icon ? `<span class="nav-icon" aria-hidden="true">${icon}</span>` : '');

  // Pages inside a group go without icons: the group's own icon stands for them.
  const pageLi = (item, withIcon) => {
    const isActive = currentFile === item.href.split('#')[0];
    const link = `<a href="${item.href}"${isActive ? ' class="active" aria-current="page"' : ''}>` +
      `${withIcon ? iconSpan(navIcon(item.href)) : ''}<span class="nav-label">${item.label}</span></a>`;

    const sub = subItems(item, isActive);
    if (!sub.length) return `<li>${link}</li>`;
    const kids = sub.map((c) => {
      const on = isActive && c.anchor === currentHash;
      return `<li><a href="${c.href}" data-anchor="${c.anchor}"${on ? ' class="active"' : ''}>${c.label}</a></li>`;
    }).join('');
    return `<li>${link}<ul class="sidebar-subnav">${kids}</ul></li>`;
  };

  // Announced but not written yet: named in the list, not a link.
  const soonLi = (item, withIcon) =>
    `<li><span class="nav-soon" aria-disabled="true">${withIcon ? iconSpan(NAV_ICONS[item.icon]) : ''}` +
    `<span class="nav-label">${item.label}</span><span class="soon-badge">${T.soon}</span></span></li>`;

  // On arriving at the site every fold-out group is folded, the one holding
  // the page being read included (its title is highlighted instead). From
  // then on each stays as the reader last left it, until the tab is closed.
  let folds = {};
  try { folds = JSON.parse(sessionStorage.getItem('navGroups') || '{}') || {}; } catch (e) { /* ignore */ }
  const groupLi = (item, index) => {
    const items = item.items || [];
    const holdsCurrent = items.some((i) => i.href && i.href.split('#')[0] === currentFile);
    const open = Boolean(folds[item.group]);
    const id = `nav-group-${index}`;
    const kids = items.map((i) => (i.soon ? soonLi(i, false) : pageLi(i, false))).join('');
    return `<li class="nav-group${open ? ' open' : ''}${holdsCurrent ? ' holds-current' : ''}">` +
      `<button type="button" class="nav-group-toggle" aria-expanded="${open}" aria-controls="${id}" data-group="${escapeHtml(item.group)}">` +
      `${iconSpan(NAV_ICONS[item.icon])}<span class="nav-label">${item.group}</span>` +
      `<span class="nav-chevron" aria-hidden="true">${ICONS.chevron}</span></button>` +
      `<div class="nav-group-fold" id="${id}"><ul class="nav-group-items">${kids}</ul></div></li>`;
  };

  navItems().forEach((item, index) => {
    if (item.anchor) return;
    if (item.section) {
      closeList();
      nav += `<div class="sidebar-section-title">${item.section}</div>`;
      return;
    }
    if (!openList) {
      nav += '<ul>';
      openList = true;
    }
    if (item.group) nav += groupLi(item, index);
    else if (item.soon) nav += soonLi(item, true);
    else nav += pageLi(item, true);
  });
  closeList();

  // The highlighted links above the sections, for the pages readers come for.
  const anchors = navItems().filter((i) => i.anchor).map((a) => {
    const on = currentFile === a.href.split('#')[0];
    return `<a href="${a.href}" class="sidebar-anchor${on ? ' active' : ''}">` +
      `<span class="anchor-icon" aria-hidden="true">${NAV_ICONS[a.icon] || ''}</span><span>${a.anchor}</span></a>`;
  }).join('');

  aside.innerHTML = `
    <div class="sidebar-top">
      <a href="${homeHref}" class="logo">${LOGO_SVG}<span>UNI Docs</span><span class="version-badge">v1.2</span></a>
      <div class="theme-seg" role="group">
        <button type="button" data-theme-set="system" title="${T.system}" aria-label="${T.system}">${ICONS.monitor}</button>
        <button type="button" data-theme-set="light" title="${T.light}" aria-label="${T.light}">${ICONS.sun}</button>
        <button type="button" data-theme-set="dark" title="${T.dark}" aria-label="${T.dark}">${ICONS.moon}</button>
      </div>
    </div>

    <button type="button" class="search-trigger" data-open-search>
      ${ICONS.search}<span class="st-label">${T.search}</span><kbd>${isMac ? '⌘K' : 'Ctrl K'}</kbd>
    </button>

    ${anchors ? `<div class="sidebar-anchors">${anchors}</div>` : ''}

    <nav class="sidebar-nav" aria-label="Documentation">${nav}</nav>

    <div class="sidebar-footer">
      <a href="${langHref(isRuPage ? 'en' : 'ru')}" id="lang-switch">
        ${ICONS.globe}<span>${T.otherLang}</span>
      </a>
      <a href="${REPO_URL}" target="_blank" rel="noopener">
        ${ICONS.github}<span>${T.github}</span><span class="ext">${ICONS.external}</span>
      </a>
    </div>`;

  aside.querySelectorAll('.nav-group-toggle').forEach((b) => {
    b.addEventListener('click', () => {
      const group = b.closest('.nav-group');
      const open = !group.classList.contains('open');
      group.classList.toggle('open', open);
      b.setAttribute('aria-expanded', String(open));
      folds[b.dataset.group] = open;
      try { sessionStorage.setItem('navGroups', JSON.stringify(folds)); } catch (e) { /* ignore */ }
    });
  });

  aside.querySelectorAll('.theme-seg button').forEach((b) => {
    b.addEventListener('click', () => {
      store.set('theme', b.dataset.themeSet === 'system' ? null : b.dataset.themeSet);
      applyTheme();
    });
  });

  // The link is completed on click so it can carry the reader's position.
  const langLink = aside.querySelector('#lang-switch');
  langLink.addEventListener('click', () => {
    const target = isRuPage ? 'en' : 'ru';
    store.set('lang', target);
    langLink.href = langHref(target) + portableHash();
  });

  // The list stays where the reader left it: if it sprang back to the top on
  // every page, the entry under the pointer would be a different one each
  // time. Only when the current page is out of sight is it brought into view.
  let saved = 0;
  try { saved = Number(sessionStorage.getItem('navScroll')) || 0; } catch (e) { /* ignore */ }
  aside.scrollTop = saved;
  let active = aside.querySelector('.sidebar-nav a.active');
  const folded = active && active.closest('.nav-group:not(.open)');
  if (folded) active = folded.querySelector('.nav-group-toggle');
  if (active) {
    const top = active.getBoundingClientRect().top - aside.getBoundingClientRect().top;
    if (top < 8 || top > aside.clientHeight - 80) aside.scrollTop += top - aside.clientHeight / 2;
  }
  window.addEventListener('pagehide', () => {
    try { sessionStorage.setItem('navScroll', String(Math.round(aside.scrollTop))); } catch (e) { /* ignore */ }
  });
}

// Marks where the reader is in the list under the current page. That list
// can name sections (most pages) or single entries (the examples), so the
// entry is tried first and its section second.
function syncSidebarSection() {
  const links = [...document.querySelectorAll('.sidebar-nav a.active + .sidebar-subnav a[data-anchor]')];
  const here = links.some((a) => a.dataset.anchor === state.entryId) ? state.entryId : state.groupId;
  links.forEach((a) => a.classList.toggle('active', a.dataset.anchor === here));
}

/* ==========================================================================
   Header — breadcrumb and page actions
   ========================================================================== */

function currentNavEntry() {
  return navPages().find((p) => p.item.href.split('#')[0] === currentFile) || null;
}

function renderHeader(meta) {
  const header = document.querySelector('.top-header');
  if (!header) return;

  const title = meta ? meta.item.label : document.title;
  const section = (meta && meta.section) || T.getStarted;
  const collapsible = document.querySelectorAll('.api-card:not(.static)').length > 1;

  header.innerHTML = `
    <button type="button" class="mobile-menu-toggle" id="menu-toggle" aria-label="${T.menu}">${ICONS.menu}</button>
    <nav class="breadcrumb" aria-label="Breadcrumb">
      <span class="crumb-section">${section}</span>${ICONS.chevron}<span class="crumb-page">${title}</span>
      <span class="crumb-tail">${ICONS.chevron}<span class="crumb-here"></span></span>
    </nav>
    <div class="header-actions">
      <button type="button" class="icon-btn header-search" data-open-search aria-label="${T.searchPlaceholder}">${ICONS.search}</button>
      ${collapsible ? `<button type="button" class="ghost-btn" id="toggle-all"><span class="btn-icon"></span><span class="btn-label"></span></button>` : ''}
      <button type="button" class="ghost-btn" id="copy-page" aria-label="${T.copyPage}"><span class="btn-icon">${ICONS.copy}</span><span class="btn-label">${T.copyPage}</span></button>
    </div>`;

  // The breadcrumb repeats the page title, so on a wide screen it only
  // appears once the title itself has scrolled away.
  const h1 = document.querySelector('.page-content h1');
  const onScroll = () => {
    header.classList.toggle('scrolled', window.scrollY > 8);
    header.classList.toggle('past-title', !h1 || h1.getBoundingClientRect().bottom < headerHeight());
  };
  onScroll();
  window.addEventListener('scroll', onScroll, { passive: true });

  header.querySelector('#copy-page').addEventListener('click', (e) => copyPage(e.currentTarget));

  const toggleAll = header.querySelector('#toggle-all');
  if (toggleAll) {
    const paint = () => {
      const open = store.get('apiOpen') === '1';
      toggleAll.querySelector('.btn-icon').innerHTML = open ? ICONS.fold : ICONS.unfold;
      toggleAll.querySelector('.btn-label').textContent = open ? T.collapseAll : T.expandAll;
      toggleAll.setAttribute('aria-label', open ? T.collapseAll : T.expandAll);
    };
    paint();
    toggleAll.addEventListener('click', () => {
      const open = store.get('apiOpen') !== '1';
      store.set('apiOpen', open ? '1' : null);
      document.querySelectorAll('.api-card:not(.static)').forEach((card) => setCardOpen(card, open));
      paint();
    });
  }
}

function setCrumbSection(text) {
  const here = document.querySelector('.crumb-here');
  if (!here) return;
  here.textContent = text || '';
  here.parentElement.classList.toggle('on', Boolean(text));
}

/* Serialise the page to Markdown so it can be pasted elsewhere. */
function copyPage(btn) {
  const root = document.querySelector('.page-content');
  if (!root) return;

  const lines = [];
  const walk = (node) => {
    Array.from(node.children).forEach((el) => {
      const tag = el.tagName;
      if (/^H[1-4]$/.test(tag)) {
        lines.push('\n' + '#'.repeat(+tag[1]) + ' ' + clean(el.textContent) + '\n');
      } else if (tag === 'P') {
        lines.push(clean(el.textContent) + '\n');
      } else if (tag === 'UL' || tag === 'OL') {
        Array.from(el.children).forEach((li) => lines.push('- ' + clean(li.textContent)));
        lines.push('');
      } else if (tag === 'PRE') {
        lines.push('```cpp\n' + el.textContent.replace(/\n+$/, '') + '\n```\n');
      } else if (tag === 'TABLE') {
        Array.from(el.querySelectorAll('tr')).forEach((tr) => {
          lines.push('| ' + Array.from(tr.children).map((td) => clean(td.textContent)).join(' | ') + ' |');
        });
        lines.push('');
      } else if (el.classList.contains('api-header')) {
        const name = el.querySelector('.api-name');
        const sig = el.querySelector('.api-signature');
        lines.push('\n### ' + (name ? clean(name.textContent) : ''));
        if (sig) lines.push('`' + clean(sig.textContent) + '`\n');
      } else if (el.classList.contains('page-nav')) {
        // navigation, not content
      } else if (el.children.length) {
        walk(el);
      }
    });
  };
  walk(root);

  copyText(lines.join('\n').replace(/\n{3,}/g, '\n\n').trim()).then(() => {
    const icon = btn.querySelector('.btn-icon');
    const label = btn.querySelector('.btn-label');
    btn.classList.add('done');
    icon.innerHTML = ICONS.check;
    label.textContent = T.copied;
    setTimeout(() => {
      btn.classList.remove('done');
      icon.innerHTML = ICONS.copy;
      label.textContent = T.copyPage;
    }, 1800);
  });
}

/* ==========================================================================
   Layout — wrap the static content and add the table of contents
   ========================================================================== */

// The page outline: every h2, with the cards below it (methods, examples,
// topics) as its children.
function pageOutline(content) {
  const groups = [];
  let current = null;
  content.querySelectorAll('h2, .api-card[id]').forEach((el) => {
    if (el.tagName === 'H2') {
      current = { el, id: el.id, text: clean(el.textContent), children: [] };
      groups.push(current);
      return;
    }
    const label = el.querySelector('.api-name, h3');
    if (!current || !label) return;
    current.children.push({
      el, id: el.id, text: clean(label.textContent), code: label.classList.contains('api-name'),
    });
  });
  return groups;
}

function buildLayout() {
  const content = document.querySelector('.page-content');
  if (!content || content.parentElement.classList.contains('content-columns')) return;

  const columns = document.createElement('div');
  columns.className = 'content-columns';
  content.parentElement.insertBefore(columns, content);
  columns.appendChild(content);

  const toc = document.createElement('aside');
  toc.className = 'toc';
  columns.appendChild(toc);

  const groups = pageOutline(content);
  const childCount = groups.reduce((n, g) => n + g.children.length, 0);
  if (groups.length + childCount < 2) return;

  // A long reference page would bury the section list under its methods, so
  // there the methods show for the section being read only.
  if (childCount > 14) toc.classList.add('toc-collapsible');

  const link = (item, cls) =>
    `<a href="#${item.id}" data-id="${item.id}"${cls ? ` class="${cls}"` : ''}></a>`;

  toc.innerHTML = `<div class="toc-title">${ICONS.list}<span>${T.onThisPage}</span></div><ul class="toc-list">` +
    groups.map((g) => `<li class="toc-group">${link(g)}` +
      (g.children.length
        ? `<div class="toc-fold"><div><ul class="toc-sub">${g.children.map((c) => `<li>${link(c, c.code ? 'toc-code' : '')}</li>`).join('')}</ul></div></div>`
        : '') + '</li>').join('') + '</ul>';

  // Labels go in as text, never as markup.
  const labels = new Map();
  groups.forEach((g) => {
    labels.set(g.id, g.text);
    g.children.forEach((c) => labels.set(c.id, c.text));
  });
  toc.querySelectorAll('a[data-id]').forEach((a) => { a.textContent = labels.get(a.dataset.id); });

  initScrollSpy(groups, toc);
}

function initScrollSpy(groups, toc) {
  const items = [];
  groups.forEach((g) => {
    items.push({ id: g.id, el: g.el, group: g });
    g.children.forEach((c) => items.push({ id: c.id, el: c.el, group: g }));
  });

  const byId = new Map(items.map((it) => [it.id, it]));
  const links = new Map();
  toc.querySelectorAll('a[data-id]').forEach((a) => links.set(a.dataset.id, a));
  const h1 = document.querySelector('.page-content h1');
  let queued = false;

  // An entry the reader picked — here, in the sidebar, in search or through
  // a link — stays the current one until they scroll on. Otherwise a section
  // near the end of the page, which can never reach the top of the window,
  // would hand the marker to a neighbour the moment it was chosen.
  let pinned = null;            // { item, y }; y is where the page came to rest
  let settle = null;
  const rest = () => {
    if (pinned && pinned.y === null) pinned.y = window.scrollY;
  };

  const update = () => {
    queued = false;
    const line = headerHeight() + 40;
    let found = items[0];
    for (const it of items) {
      if (it.el.getBoundingClientRect().top <= line) found = it;
      else break;
    }
    // At the very bottom the last entry is the one being read, even if it
    // is too short to reach the top of the window.
    if (window.innerHeight + window.scrollY >= document.documentElement.scrollHeight - 4) {
      found = items[items.length - 1];
    }
    if (pinned) {
      if (pinned.y === null || Math.abs(window.scrollY - pinned.y) < 4) found = pinned.item;
      else pinned = null;
    }

    let moved = false;
    if (found.id !== state.entryId) {
      const previous = links.get(state.entryId);
      if (previous) previous.classList.remove('active');
      state.entryId = found.id;
      const active = links.get(found.id);
      if (active) {
        active.classList.add('active');
        // Keep the marker in view inside the table of contents itself.
        const top = active.getBoundingClientRect().top - toc.getBoundingClientRect().top + toc.scrollTop;
        if (top < toc.scrollTop + 32) toc.scrollTop = Math.max(0, top - 32);
        else if (top > toc.scrollTop + toc.clientHeight - 40) toc.scrollTop = top - toc.clientHeight + 40;
      }
      moved = true;
    }

    if (found.group.id !== state.groupId) {
      state.groupId = found.group.id;
      toc.querySelectorAll('.toc-group').forEach((li) => {
        li.classList.toggle('open', li.firstElementChild.dataset.id === state.groupId);
      });
      moved = true;
    }
    if (moved) syncSidebarSection();

    const pastTitle = !h1 || h1.getBoundingClientRect().bottom < headerHeight();
    setCrumbSection(pastTitle && found.group.el.getBoundingClientRect().top <= line ? found.group.text : '');
  };

  const request = () => {
    if (queued) return;
    queued = true;
    window.requestAnimationFrame(update);
  };

  state.pin = (id) => {
    const item = byId.get(id);
    if (!item) return;
    pinned = { item, y: null };
    clearTimeout(settle);
    settle = setTimeout(rest, 1200);       // nothing may need to scroll at all
    request();
  };
  toc.addEventListener('click', (e) => {
    const a = e.target.closest('a[data-id]');
    if (a) state.pin(a.dataset.id);
  });

  update();
  window.addEventListener('scroll', () => {
    if (pinned && pinned.y === null) {     // still travelling to the chosen entry
      clearTimeout(settle);
      settle = setTimeout(rest, 160);
    }
    request();
  }, { passive: true });
  window.addEventListener('resize', request);
  // Opening a card moves everything below it.
  document.addEventListener('uni:layout', request);
}

/* A link icon next to each section heading, for sharing that section. */
function initHeadingAnchors() {
  document.querySelectorAll('.page-content h2[id]').forEach((h2) => {
    const a = document.createElement('a');
    a.className = 'heading-anchor';
    a.href = '#' + h2.id;
    a.setAttribute('aria-label', T.linkHere);
    a.title = T.linkHere;
    a.innerHTML = ICONS.link;
    a.addEventListener('click', () => copyText(window.location.href.split('#')[0] + '#' + h2.id));
    h2.appendChild(a);
  });
}

/* Fill in the icons declared on home-page navigation cards. */
function initCardIcons() {
  document.querySelectorAll('.nav-card-icon[data-icon]').forEach((el) => {
    const icon = NAV_ICONS[el.dataset.icon];
    if (icon) el.innerHTML = icon.replace('width="15" height="15"', 'width="20" height="20"');
  });
}

/* Callouts are written as bare boxes; the icon comes from their kind. */
function initCallouts() {
  document.querySelectorAll('.callout').forEach((box) => {
    if (box.querySelector('.callout-icon')) return;
    const kind = ['tip', 'warning'].find((k) => box.classList.contains(k)) || 'note';
    const icon = document.createElement('span');
    icon.className = 'callout-icon';
    icon.setAttribute('aria-hidden', 'true');
    icon.innerHTML = ICONS[kind];
    box.insertBefore(icon, box.firstChild);
  });
}

/* Section label above the page title, matching the sidebar grouping. */
function renderEyebrow(meta) {
  const h1 = document.querySelector('.page-content h1');
  if (!h1 || (h1.previousElementSibling && h1.previousElementSibling.classList.contains('eyebrow'))) return;
  const el = document.createElement('div');
  el.className = 'eyebrow';
  el.textContent = (meta && meta.section) || T.getStarted;
  h1.parentElement.insertBefore(el, h1);
}

/* Previous / next links derived from the navigation order. */
function renderPageNav() {
  const flat = navPages().map((p) => ({ href: p.item.href, label: p.item.label, section: p.section }));
  const idx = flat.findIndex((i) => i.href.split('#')[0] === currentFile);
  if (idx === -1) return;

  const prev = flat[idx - 1];
  const next = flat[idx + 1];
  if (!prev && !next) return;

  // A step into another part of the docs names that part: "UniBase" alone
  // could be the platform or its reference.
  const dir = (word, to) => (to.section !== flat[idx].section ? `${word} · ${to.section}` : word);

  const content = document.querySelector('.page-content');
  const nav = document.createElement('nav');
  nav.className = 'page-nav';
  nav.innerHTML =
    (prev ? `<a href="${prev.href}" class="pn-prev"><span class="pn-dir">${dir(T.previous, prev)}</span><span class="pn-title">${prev.label}</span></a>` : '') +
    (next ? `<a href="${next.href}" class="pn-next"><span class="pn-dir">${dir(T.next, next)}</span><span class="pn-title">${next.label}</span></a>` : '');
  content.appendChild(nav);
}

/* ==========================================================================
   Code blocks
   ========================================================================== */

function initCodeBlocks() {
  document.querySelectorAll('pre').forEach((pre) => {
    if (pre.closest('.code-frame')) return;

    const frame = document.createElement('div');
    frame.className = 'code-frame';
    pre.parentNode.insertBefore(frame, pre);

    const head = document.createElement('div');
    head.className = 'code-frame-head';
    head.innerHTML = '<span class="code-lang">arduino</span>';

    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'copy-btn';
    btn.title = T.copy;
    btn.setAttribute('aria-label', T.copy);
    btn.innerHTML = ICONS.copy;
    head.appendChild(btn);

    const bodyEl = document.createElement('div');
    bodyEl.className = 'code-frame-body';
    bodyEl.appendChild(pre);

    frame.appendChild(head);
    frame.appendChild(bodyEl);

    btn.addEventListener('click', (e) => {
      e.stopPropagation();
      copyText((pre.querySelector('code') || pre).innerText).then(() => {
        btn.innerHTML = ICONS.check;
        btn.classList.add('copied');
        setTimeout(() => {
          btn.innerHTML = ICONS.copy;
          btn.classList.remove('copied');
        }, 1800);
      });
    });

    const code = pre.querySelector('code');
    if (code) highlightCode(code);
  });
}

function highlightCode(element) {
  let html = (element.textContent || '').trim()
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;');

  // Comments, strings and include targets are lifted out before the token
  // passes run. A marker encodes its index as letters wrapped in a control
  // character: digits would be captured by the number pass below, and word
  // characters by the keyword pass.
  const stash = [];
  const SEP = String.fromCharCode(1);
  const DIGITS = 'ABCDEFGHIJ';
  const keep = (m) => SEP + String(stash.push(m) - 1).replace(/\d/g, (d) => DIGITS[+d]) + SEP;

  html = html.replace(/(\/\/[^\n]*)/g, (m) => keep(`<span class="token comment">${m}</span>`));
  html = html.replace(/("[^"\n]*")/g, (m) => keep(`<span class="token string">${m}</span>`));
  html = html.replace(/(&lt;[\w./]+&gt;)/g, (m) => keep(`<span class="token string">${m}</span>`));
  html = html.replace(/^(\s*)(#\w+)/gm, (m, ws, d) => ws + keep(`<span class="token keyword">${d}</span>`));

  html = html
    .replace(/\b(void|int|float|double|bool|char|long|short|unsigned|signed|const|static|return|if|else|for|while|do|struct|class|public|private|protected|new|delete|break|continue|switch|case|default|true|false|nullptr|sizeof)\b/g,
      '<span class="token keyword">$1</span>')
    .replace(/\b(String|UniBase|UniBaseControl|UniDev|Adafruit_SSD1306|OdometryData|Serial)\b/g,
      '<span class="token type">$1</span>')
    .replace(/\b([a-zA-Z_]\w*)(?=\s*\()/g, '<span class="token function">$1</span>')
    .replace(/\b(\d+(?:\.\d+)?)\b/g, '<span class="token number">$1</span>');

  // Keywords and types are tagged first, so the function pass can wrap a name
  // that already carries a token span; unwrap those.
  html = html.replace(
    /<span class="token function">(<span class="token (?:keyword|type)">[^<]*<\/span>)<\/span>/g, '$1');

  html = html.replace(new RegExp(SEP + '([A-J]+)' + SEP, 'g'), (_, letters) =>
    stash[+letters.replace(/[A-J]/g, (c) => DIGITS.indexOf(c))]);

  element.innerHTML = html;
}

/* ==========================================================================
   Collapsible API cards
   ========================================================================== */

// The first sentence of a description, as a one-line summary.
function firstSentence(text) {
  const t = clean(text);
  const m = t.match(/^.*?[.!?](?=\s+[A-ZА-ЯЁ(`]|$)/);
  const s = m ? m[0] : t;
  return s.length > 140 ? s.slice(0, 137).replace(/\s+\S*$/, '') + '…' : s;
}

// A closed card is hidden="until-found" rather than display:none, so the
// browser's own find-in-page still sees the text inside and can open it.
function setCardOpen(card, open) {
  const header = card.querySelector('.api-header');
  const details = card.querySelector('.api-details');
  if (!header || !details) return;
  card.classList.toggle('expanded', open);
  header.setAttribute('aria-expanded', String(open));
  if (open) details.removeAttribute('hidden');
  else details.setAttribute('hidden', 'until-found');
  document.dispatchEvent(new Event('uni:layout'));
}

function initApiCards() {
  const openAll = store.get('apiOpen') === '1';

  document.querySelectorAll('.api-card:not(.static)').forEach((card) => {
    const header = card.querySelector('.api-header');
    const details = card.querySelector('.api-details');
    if (!header || !details) return;

    // What the method does, readable without opening the card.
    const lead = details.querySelector(':scope > p');
    const titles = header.querySelector('.api-title-container');
    if (lead && titles) {
      const summary = document.createElement('span');
      summary.className = 'api-summary';
      summary.textContent = firstSentence(lead.textContent);
      titles.appendChild(summary);
    }

    const anchor = document.createElement('a');
    anchor.className = 'api-anchor';
    anchor.href = '#' + card.id;
    anchor.title = T.linkHere;
    anchor.setAttribute('aria-label', T.linkHere);
    anchor.innerHTML = ICONS.link;
    anchor.addEventListener('click', (e) => {
      e.stopPropagation();
      copyText(window.location.href.split('#')[0] + '#' + card.id);
    });
    header.insertBefore(anchor, header.querySelector('.expand-icon'));

    header.setAttribute('role', 'button');
    header.setAttribute('tabindex', '0');
    setCardOpen(card, openAll);
    card.classList.add('ready');

    header.addEventListener('click', () => setCardOpen(card, !card.classList.contains('expanded')));
    header.addEventListener('keydown', (e) => {
      if (e.target !== header || (e.key !== 'Enter' && e.key !== ' ')) return;
      e.preventDefault();
      setCardOpen(card, !card.classList.contains('expanded'));
    });
    details.addEventListener('beforematch', () => setCardOpen(card, true));
  });
}

/* ==========================================================================
   Site index — every page of the current language, read once and shared by
   search and by the automatic links to the reference.
   ========================================================================== */

const siteIndex = { entries: [], complete: false, promise: null };

function sitePages() {
  return navPages().map(({ item }) => ({
    file: item.href.split('#')[0], label: item.label, children: item.children || [],
  }));
}

// Works on the live document and on a fetched, parsed one alike.
function indexDocument(root, file, pageLabel) {
  const content = root.querySelector('.page-content');
  if (!content) return [];
  if (root !== document) assignIds(root);

  const entries = [];
  const add = (type, title, anchor, crumb, sig, text) => {
    entries.push({ type, title, file, anchor, crumb, sig, text: clean(text).slice(0, 600) });
  };

  const lead = content.querySelector('.lead') || content.querySelector('h1 + p');
  add('page', pageLabel, '', '', '', lead ? lead.textContent : '');

  let section = '';
  content.querySelectorAll('h2, .api-card').forEach((el) => {
    if (el.tagName === 'H2') {
      section = clean(el.textContent);
      // The prose that belongs to the heading itself, up to the first card.
      let text = '';
      for (let n = el.nextElementSibling; n && n.tagName !== 'H2'; n = n.nextElementSibling) {
        if (!n.classList.contains('api-card') && !n.classList.contains('page-nav')) text += ' ' + n.textContent;
      }
      add('section', section, el.id, pageLabel, '', text);
      return;
    }
    const crumb = section ? `${pageLabel} › ${section}` : pageLabel;
    const name = el.querySelector('.api-name');
    if (name) {
      const sig = el.querySelector('.api-signature');
      const details = el.querySelector('.api-details');
      add('method', clean(name.textContent), el.id, crumb, sig ? clean(sig.textContent) : '',
        details ? details.textContent : '');
      return;
    }
    const h3 = el.querySelector('h3');
    if (!h3 || !el.id || el.classList.contains('fn-group')) return;
    const title = clean(h3.textContent);
    add(/^examples/.test(file) ? 'example' : 'topic', title, el.id, crumb, '',
      clean(el.textContent).slice(title.length));
  });

  return entries;
}

// Without the page itself (file:// or a failed request) the sidebar still
// tells us its title and sections.
function fallbackEntries(page) {
  const entry = (title, anchor, crumb) => ({
    type: anchor ? 'section' : 'page', title, file: page.file, anchor, crumb, sig: '', text: '',
  });
  return [entry(page.label, '', '')].concat(
    page.children.map((c) => entry(c.label, c.href.split('#')[1] || '', page.label)));
}

function loadSiteIndex() {
  if (siteIndex.promise) return siteIndex.promise;

  const pages = sitePages();
  const here = pages.find((p) => p.file === currentFile);
  const local = indexDocument(document, currentFile, here ? here.label : document.title);
  const others = pages.filter((p) => p.file !== currentFile);
  siteIndex.entries = local.concat(...others.map(fallbackEntries));

  siteIndex.promise = Promise.all(others.map((page) =>
    fetch(page.file)
      .then((res) => (res.ok ? res.text() : Promise.reject(new Error(String(res.status)))))
      .then((html) => indexDocument(new DOMParser().parseFromString(html, 'text/html'), page.file, page.label))
      .then((list) => (list.length ? list : fallbackEntries(page)))
      .catch(() => fallbackEntries(page))
  )).then((lists) => {
    siteIndex.entries = local.concat(...lists);
    siteIndex.complete = true;
    return siteIndex.entries;
  });

  return siteIndex.promise;
}

// Titles and signatures match anywhere, so "Dist" finds moveDist. Body text
// matches at the start of a word only, so "uart" does not find "quarter".
const wordStart = (term) => new RegExp('(?:^|[^\\p{L}\\p{N}_])' + escapeRe(term), 'u');

function searchEntries(entries, terms) {
  const bonus = { method: 6, page: 4, example: 3, topic: 3, section: 2 };
  const inText = terms.map(wordStart);
  const hits = [];
  entries.forEach((e) => {
    const title = e.title.toLowerCase();
    const sig = e.sig.toLowerCase();
    const text = e.text.toLowerCase();
    let score = 0;
    for (let i = 0; i < terms.length; i += 1) {
      const t = terms[i];
      let s = 0;
      if (title === t) s = 100;
      else if (title.startsWith(t)) s = 70;
      else if (title.includes(t)) s = 50;
      else if (sig.includes(t)) s = 25;
      else if (inText[i].test(text)) s = 10;
      if (!s) return;
      score += s;
    }
    hits.push({ e, score: score + (bonus[e.type] || 0) + (e.file === currentFile ? 1 : 0) });
  });
  hits.sort((a, b) => b.score - a.score);
  return hits.slice(0, 20).map((h) => h.e);
}

/* Links inline mentions such as `waitMove()` to their reference entry. */
function autoLinkCode(entries) {
  const byName = new Map();
  entries.forEach((e) => {
    if (e.type !== 'method') return;
    (e.title.split('(')[0].match(/[A-Za-z_]\w*/g) || []).forEach((name) => {
      if (!byName.has(name)) byName.set(name, []);
      byName.get(name).push(e);
    });
  });

  document.querySelectorAll('.page-content code').forEach((code) => {
    if (code.closest('pre, a, .api-header, h1, h2, h3, h4')) return;
    const m = clean(code.textContent).match(/^(?:(robot|module)\.)?([A-Za-z_]\w*)\s*(?:\([^;]*\))?;?$/);
    if (!m || !byName.has(m[2])) return;

    let found = byName.get(m[2]);
    if (m[1] === 'robot') found = found.filter((e) => /^unibase/.test(e.file));
    if (m[1] === 'module') found = found.filter((e) => /^unidev/.test(e.file));

    // A name documented on two pages is only linked when it is unambiguous.
    let target = found.find((e) => e.file === currentFile);
    if (!target && new Set(found.map((e) => e.file)).size === 1) target = found[0];
    if (!target) return;

    const own = code.closest('.api-card');
    if (own && target.file === currentFile && own.id === target.anchor) return;

    const a = document.createElement('a');
    a.className = 'code-link';
    a.href = (target.file === currentFile ? '' : target.file) + '#' + target.anchor;
    code.parentNode.insertBefore(a, code);
    a.appendChild(code);
  });
}

/* ==========================================================================
   Search dialog
   ========================================================================== */

function highlightInto(el, text, terms) {
  const re = new RegExp('(' + terms.map(escapeRe).join('|') + ')', 'ig');
  text.split(re).forEach((part, i) => {
    if (!part) return;
    if (i % 2) {
      const mark = document.createElement('mark');
      mark.textContent = part;
      el.appendChild(mark);
    } else {
      el.appendChild(document.createTextNode(part));
    }
  });
}

// Second line of a result: where it lives, then its signature or the text
// around the match.
function contextLine(entry, terms) {
  const head = (entry.title + ' ' + entry.sig).toLowerCase();
  let detail = entry.sig;
  if (!terms.every((t) => head.includes(t))) {
    const hay = entry.text.toLowerCase();
    const term = terms.find((t) => !head.includes(t) && wordStart(t).test(hay));
    if (term) {
      const hit = wordStart(term).exec(hay);
      const at = hit.index + hit[0].length - term.length;
      const from = Math.max(0, at - 40);
      detail = (from ? '…' : '') + entry.text.slice(from, at + term.length + 70).trim() + '…';
    }
  } else if (!detail && entry.text) {
    detail = entry.text.length > 100 ? entry.text.slice(0, 100).trim() + '…' : entry.text;
  }
  return [entry.crumb, detail].filter(Boolean).join(' — ');
}

function initSearch() {
  const dialog = document.createElement('div');
  dialog.className = 'search-dialog';
  dialog.innerHTML = `
    <div class="search-panel" role="dialog" aria-modal="true" aria-label="${T.searchPlaceholder}">
      <div class="search-field">
        ${ICONS.search}
        <input type="text" class="search-input" placeholder="${T.searchPlaceholder}" autocomplete="off" spellcheck="false">
        <kbd>ESC</kbd>
      </div>
      <div class="search-results"></div>
      <div class="search-foot" hidden>${T.searching}</div>
    </div>`;
  document.body.appendChild(dialog);

  const input = dialog.querySelector('.search-input');
  const results = dialog.querySelector('.search-results');
  const foot = dialog.querySelector('.search-foot');
  let shown = [];
  let cursor = -1;
  let lastFocus = null;

  const message = (cls, text) => {
    results.innerHTML = '';
    const el = document.createElement('div');
    el.className = cls;
    el.textContent = text;
    results.appendChild(el);
  };

  const isOpen = () => dialog.classList.contains('open');

  const close = () => {
    if (!isOpen()) return;
    dialog.classList.remove('open');
    if (lastFocus && document.contains(lastFocus)) lastFocus.focus({ preventScroll: true });
  };

  const go = (entry) => {
    close();
    if (entry.file !== currentFile) {
      window.location.href = entry.file + (entry.anchor ? '#' + entry.anchor : '');
    } else if (!entry.anchor) {
      window.scrollTo({ top: 0, behavior: 'smooth' });
    } else {
      window.history.pushState(null, '', '#' + entry.anchor);
      if (state.pin) state.pin(entry.anchor);
      revealId(entry.anchor, false);
    }
  };

  const setCursor = (n) => {
    const items = results.querySelectorAll('.search-result-item');
    if (!items.length) return;
    if (items[cursor]) items[cursor].classList.remove('cursor');
    cursor = (n + items.length) % items.length;
    items[cursor].classList.add('cursor');
    items[cursor].scrollIntoView({ block: 'nearest' });
  };

  const render = () => {
    const terms = input.value.trim().toLowerCase().split(/\s+/).filter(Boolean);
    cursor = -1;
    shown = [];
    if (!terms.length || terms.join('').length < 2) {
      foot.hidden = true;
      message('search-hint', T.searchHint);
      return;
    }

    shown = searchEntries(siteIndex.entries, terms);
    foot.hidden = siteIndex.complete;
    if (!shown.length) {
      message('search-no-results', siteIndex.complete ? T.noResults : T.searching);
      return;
    }

    results.innerHTML = '';
    shown.forEach((entry) => {
      const item = document.createElement('button');
      item.type = 'button';
      item.className = 'search-result-item';

      const type = document.createElement('span');
      type.className = 'result-type';
      type.textContent = T.types[entry.type] || '';

      const body = document.createElement('span');
      body.className = 'result-body';
      const title = document.createElement('span');
      title.className = 'result-title' + (entry.type === 'method' ? ' is-code' : '');
      highlightInto(title, entry.title, terms);
      const context = document.createElement('span');
      context.className = 'result-context';
      context.textContent = contextLine(entry, terms);
      body.appendChild(title);
      body.appendChild(context);

      item.appendChild(type);
      item.appendChild(body);
      item.addEventListener('click', () => go(entry));
      results.appendChild(item);
    });
    setCursor(0);
  };

  const open = () => {
    if (isOpen()) return;
    lastFocus = document.activeElement;
    dialog.classList.add('open');
    input.value = '';
    render();
    input.focus();
    // Results from this page are available at once; the rest of the site
    // follows as soon as it has been read.
    loadSiteIndex().then(() => {
      if (isOpen()) render();
    });
  };

  document.addEventListener('click', (e) => {
    if (e.target.closest('[data-open-search]')) open();
  });
  dialog.addEventListener('mousedown', (e) => {
    if (e.target === dialog) close();
  });
  input.addEventListener('input', render);

  document.addEventListener('keydown', (e) => {
    const typing = /^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName) || e.target.isContentEditable;
    if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'k') {
      e.preventDefault();
      if (isOpen()) close();
      else open();
    } else if (e.key === '/' && !typing && !e.metaKey && !e.ctrlKey && !e.altKey) {
      e.preventDefault();
      open();
    } else if (e.key === 'Escape') {
      close();
    }
  });

  input.addEventListener('keydown', (e) => {
    if (e.key === 'ArrowDown') {
      e.preventDefault();
      setCursor(cursor + 1);
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      setCursor(cursor - 1);
    } else if (e.key === 'Enter' && shown.length) {
      e.preventDefault();
      go(shown[Math.max(cursor, 0)]);
    }
  });
}

/* ==========================================================================
   Mobile navigation
   ========================================================================== */

function initMobileNav() {
  const sidebar = document.querySelector('.sidebar');
  const toggle = document.getElementById('menu-toggle');
  if (!sidebar || !toggle) return;

  const scrim = document.createElement('div');
  scrim.className = 'scrim';
  document.body.appendChild(scrim);

  const setOpen = (open) => {
    sidebar.classList.toggle('open', open);
    scrim.classList.toggle('open', open);
    toggle.setAttribute('aria-expanded', String(open));
  };
  setOpen(false);

  // Past the breakpoint the sidebar is always on screen; a drawer left open
  // would keep its scrim over the page.
  const wide = window.matchMedia('(min-width: 901px)');
  const onWide = () => { if (wide.matches) setOpen(false); };
  if (wide.addEventListener) wide.addEventListener('change', onWide);
  else if (wide.addListener) wide.addListener(onWide);

  toggle.addEventListener('click', () => setOpen(!sidebar.classList.contains('open')));
  scrim.addEventListener('click', () => setOpen(false));
  sidebar.addEventListener('click', (e) => {
    if (e.target.closest('a, [data-open-search]')) setOpen(false);
  });
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') setOpen(false);
  });
}

/* ==========================================================================
   Boot
   ========================================================================== */

document.addEventListener('DOMContentLoaded', () => {
  const meta = currentNavEntry();
  state.generated = assignIds(document);

  renderSidebar();
  renderHeader(meta);
  applyTheme();
  renderEyebrow(meta);
  initCardIcons();
  initCallouts();
  initCodeBlocks();
  initApiCards();
  buildLayout();
  initHeadingAnchors();
  renderPageNav();
  initSearch();
  initMobileNav();

  // A link straight to a method opens its card. The jump is repeated once
  // the web fonts are in, because they change the height of what is above.
  if (hashId()) {
    if (state.pin) state.pin(hashId());
    revealId(hashId(), true);
    if (document.fonts && document.fonts.ready) {
      const before = window.scrollY;
      document.fonts.ready.then(() => {
        if (Math.abs(window.scrollY - before) < 4) revealId(hashId(), true);
      });
    }
  }
  window.addEventListener('hashchange', () => {
    const el = document.getElementById(hashId());
    const card = el && el.closest('.api-card:not(.static)');
    if (card) setCardOpen(card, true);
    if (state.pin) state.pin(hashId());
  });

  // Read the other pages once the browser is idle: it makes search complete
  // before it is first opened and lets inline method names link across pages.
  const saveData = navigator.connection && navigator.connection.saveData;
  const whenIdle = window.requestIdleCallback || ((fn) => setTimeout(fn, 1200));
  if (saveData) autoLinkCode(indexDocument(document, currentFile, document.title));
  else whenIdle(() => loadSiteIndex().then(autoLinkCode));
});
