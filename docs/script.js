// Language Management
// -------------------
const currentPath = window.location.pathname;
const isRuPage = currentPath.includes('_ru.html');

// Sidebar Rendering
// -----------------
const _NAV_ICONS = {
    home:     `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><polyline points="9 22 9 12 15 12 15 22"/></svg>`,
    start:    `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><polygon points="5 3 19 12 5 21 5 3"/></svg>`,
    unibase:  `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><rect x="2" y="7" width="20" height="14" rx="2"/><path d="M16 7V5a2 2 0 0 0-2-2h-4a2 2 0 0 0-2 2v2"/><line x1="12" y1="12" x2="12" y2="16"/><line x1="10" y1="14" x2="14" y2="14"/></svg>`,
    unidev:   `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M4.22 4.22l2.12 2.12M17.66 17.66l2.12 2.12M2 12h3M19 12h3M4.22 19.78l2.12-2.12M17.66 6.34l2.12-2.12"/></svg>`,
    overview: `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M2 3h6a4 4 0 0 1 4 4v14a3 3 0 0 0-3-3H2z"/><path d="M22 3h-6a4 4 0 0 0-4 4v14a3 3 0 0 1 3-3h7z"/></svg>`,
    examples: `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><polyline points="16 18 22 12 16 6"/><polyline points="8 6 2 12 8 18"/></svg>`,
    limits:   `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>`,
    faq:      `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"/><path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>`,
};

function _getNavIcon(href) {
    const f = (href || '').split('#')[0];
    if (/^index/.test(f))            return _NAV_ICONS.home;
    if (/getting-started/.test(f))   return _NAV_ICONS.start;
    if (/^unibase/.test(f))          return _NAV_ICONS.unibase;
    if (/^unidev/.test(f))           return _NAV_ICONS.unidev;
    if (/library-overview/.test(f))  return _NAV_ICONS.overview;
    if (/^examples/.test(f))         return _NAV_ICONS.examples;
    if (/limitations/.test(f))       return _NAV_ICONS.limits;
    if (/faq/.test(f))               return _NAV_ICONS.faq;
    return '';
}

function renderSidebar() {
    const aside = document.querySelector('.sidebar');
    if (!aside || typeof window.NAV === 'undefined') return;

    const lang = isRuPage ? 'ru' : 'en';
    const items = window.NAV[lang];
    if (!items) return;

    const currentFile = (currentPath.split('/').pop() || 'index.html').split('#')[0];
    const currentHash = window.location.hash.slice(1);
    const homeHref = isRuPage ? 'index_ru.html' : 'index.html';

    let html = `
        <div class="sidebar-header">
            <a href="${homeHref}" class="logo">
                <svg class="logo-icon" width="30" height="36" viewBox="0 0 168 200" fill="none" xmlns="http://www.w3.org/2000/svg"><path d="M65.0275 93.6581L54.9206 82.2931C54.1615 81.4395 53.0738 80.9512 51.9316 80.9512H51C48.7909 80.9512 47 79.1604 47 76.9512V75C47 72.7909 48.7909 71 51 71H117C119.209 71 121 72.7909 121 75V76.9512C121 79.1604 119.209 80.9512 117 80.9512H116.068C114.926 80.9512 113.838 81.4395 113.079 82.2931L102.973 93.6581C102.213 94.5117 101.126 95 99.9835 95H68.0165C66.8743 95 65.7865 94.5117 65.0275 93.6581Z" fill="currentColor"/><path d="M47.4365 96H40C38.3431 96 37 97.3431 37 99V115.896C37 117.39 38.099 118.656 39.5779 118.866L54.5926 121H114.407L129.422 118.866C130.901 118.656 132 117.39 132 115.896V99C132 97.3431 130.657 96 129 96H121.015C120.168 96 119.361 96.3579 118.792 96.9853L107.091 109.896C106.522 110.523 105.715 110.881 104.868 110.881H64.0944C63.2697 110.881 62.4814 110.541 61.9148 109.942L49.6161 96.9386C49.0494 96.3395 48.2612 96 47.4365 96Z" fill="currentColor"/><path d="M132 172C132 174.209 130.209 176 128 176H41C38.7909 176 37 174.209 37 172V115C37 112.791 38.7909 111 41 111H128C130.209 111 132 112.791 132 115V172ZM58 128C55.7909 128 54 129.791 54 132V154C54 156.209 55.7909 158 58 158H111C113.209 158 115 156.209 115 154V132C115 129.791 113.209 128 111 128H58Z" fill="currentColor"/><path d="M128 24C130.209 24 132 25.7909 132 28V77C132 79.2091 130.209 81 128 81H41C38.7909 81 37 79.2091 37 77V28C37 25.7909 38.7909 24 41 24H128ZM58 38C55.7909 38 54 39.7909 54 42V64C54 66.2091 55.7909 68 58 68H111C113.209 68 115 66.2091 115 64V42C115 39.7909 113.209 38 111 38H58Z" fill="currentColor"/></svg>
                UNI Docs
            </a>
            <span class="version-badge">v1.1</span>
        </div>
        <nav class="sidebar-nav">`;

    let inUl = false;
    items.forEach(item => {
        if (item.section) {
            if (inUl) { html += '</ul>'; inUl = false; }
            html += `<div class="sidebar-section-title"><span>${item.section}</span></div>`;
        } else {
            if (!inUl) { html += '<ul>'; inUl = true; }
            const itemFile = item.href.split('#')[0];
            const isActive = currentFile === itemFile;
            const activeClass = isActive ? ' class="active"' : '';
            const icon = _getNavIcon(item.href);
            const iconHtml = icon ? `<span class="nav-icon" aria-hidden="true">${icon}</span>` : '';

            if (item.children) {
                html += `<li>
                    <a href="${item.href}"${activeClass}>${iconHtml}<span class="nav-label">${item.label}</span></a>
                    <ul class="sidebar-subnav">`;
                item.children.forEach(child => {
                    const childAnchor = child.href.split('#')[1] || '';
                    const childActive = isActive && childAnchor === currentHash;
                    const childClass = childActive ? ' class="active"' : '';
                    html += `<li><a href="${child.href}"${childClass}>${child.label}</a></li>`;
                });
                html += `</ul></li>`;
            } else {
                html += `<li><a href="${item.href}"${activeClass}>${iconHtml}<span class="nav-label">${item.label}</span></a></li>`;
            }
        }
    });

    if (inUl) html += '</ul>';
    html += '</nav>';
    aside.innerHTML = html;
}
const systemLang = navigator.language || navigator.userLanguage;
const storedLang = localStorage.getItem('lang');

// Determine desired language
// 1. Stored preference
// 2. Browser language (default)
// But we only auto-redirect if there is a MISMATCH and we are on a "content" page.
let desiredLang = storedLang || (systemLang.startsWith('ru') ? 'ru' : 'en');

// Redirect Logic
// We only redirect if we are confident the target exists. For now, we assume symmetry.
// We avoid redirecting if we are just opening the file mostly.
// But for the user request "Default by region", we need this.

if (storedLang || systemLang.startsWith('ru')) {
    // If explicitly stored OR system is RU (and we are on EN), try to switch.
    if (desiredLang === 'ru' && !isRuPage) {
        let target = '';
        if (currentPath.endsWith('/') || currentPath.endsWith('index.html')) {
            target = currentPath.endsWith('/') ? 'index_ru.html' : currentPath.replace('.html', '_ru.html');
        } else if (currentPath.endsWith('.html')) {
            target = currentPath.replace('.html', '_ru.html');
        }
        if (target) window.location.replace(target);
    } else if (desiredLang === 'en' && isRuPage) {
        let target = currentPath.replace('_ru.html', '.html');
        window.location.replace(target);
    }
}


// Theme Toggle
const themeToggleBtn = document.getElementById('theme-toggle');
const body = document.body;
const iconSun = '<svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="5"></circle><line x1="12" y1="1" x2="12" y2="3"></line><line x1="12" y1="21" x2="12" y2="23"></line><line x1="4.22" y1="4.22" x2="5.64" y2="5.64"></line><line x1="18.36" y1="18.36" x2="19.78" y2="19.78"></line><line x1="1" y1="12" x2="3" y2="12"></line><line x1="21" y1="12" x2="23" y2="12"></line><line x1="4.22" y1="19.78" x2="5.64" y2="18.36"></line><line x1="18.36" y1="5.64" x2="19.78" y2="4.22"></line></svg>';
const iconMoon = '<svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"></path></svg>';

// Check local storage or system preference
const currentTheme = localStorage.getItem('theme') || (window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
document.documentElement.setAttribute('data-theme', currentTheme);
if (themeToggleBtn) updateThemeIcon(currentTheme);

if (themeToggleBtn) {
    themeToggleBtn.addEventListener('click', () => {
        let theme = document.documentElement.getAttribute('data-theme');
        let newTheme = theme === 'light' ? 'dark' : 'light';
        document.documentElement.setAttribute('data-theme', newTheme);
        localStorage.setItem('theme', newTheme);
        updateThemeIcon(newTheme);
    });
}

function updateThemeIcon(theme) {
    if (theme === 'dark') {
        themeToggleBtn.innerHTML = iconSun;
    } else {
        themeToggleBtn.innerHTML = iconMoon;
    }
}

// Language Toggle Injector
const header = document.querySelector('.top-header'); // Parent of theme-toggle
if (header) {
    const langBtn = document.createElement('button');
    langBtn.className = 'theme-toggle'; // Reuse style
    langBtn.style.marginLeft = '10px';
    langBtn.style.fontSize = '14px';
    langBtn.style.fontWeight = 'bold';
    langBtn.title = isRuPage ? 'Switch to English' : 'Переключить на Русский';
    langBtn.innerText = isRuPage ? 'RU' : 'EN';

    langBtn.addEventListener('click', () => {
        const newLang = isRuPage ? 'en' : 'ru';
        localStorage.setItem('lang', newLang);

        // Reload/Redirect
        if (newLang === 'ru') {
            if (currentPath.endsWith('/') || currentPath.endsWith('index.html')) {
                window.location.href = currentPath.endsWith('/') ? 'index_ru.html' : currentPath.replace('.html', '_ru.html');
            } else {
                window.location.href = currentPath.replace('.html', '_ru.html');
            }
        } else {
            window.location.href = currentPath.replace('_ru.html', '.html');
        }
    });

    // Insert before theme toggle or append
    const themeBtn = document.getElementById('theme-toggle');
    if (themeBtn) {
        header.insertBefore(langBtn, themeBtn);
    } else {
        header.appendChild(langBtn);
    }
}

// Mobile Menu Toggle
const menuToggle = document.getElementById('mobile-menu-toggle');
const sidebar = document.querySelector('.sidebar');
if (menuToggle && sidebar) {
    menuToggle.addEventListener('click', () => {
        sidebar.classList.toggle('open');
    });

    document.addEventListener('click', (e) => {
        if (window.innerWidth <= 768) {
            if (!sidebar.contains(e.target) && !menuToggle.contains(e.target) && sidebar.classList.contains('open')) {
                sidebar.classList.remove('open');
            }
        }
    });
}

// API Card Expansion Logic
function initApiCards() {
    const headers = document.querySelectorAll('.api-header');
    headers.forEach(header => {
        header.addEventListener('click', () => {
            const card = header.parentElement;

            // Toggle active class
            const isExpanded = card.classList.contains('expanded');

            // Optional: Close others? For now we allow multiple open
            // document.querySelectorAll('.api-card').forEach(c => c.classList.remove('expanded'));

            if (isExpanded) {
                card.classList.remove('expanded');
            } else {
                card.classList.add('expanded');
            }
        });
    });
}
// Run init after DOM load
document.addEventListener('DOMContentLoaded', () => {
    renderSidebar();
    initApiCards();
});


// Global Copy to Clipboard & Syntax Highlighting
document.querySelectorAll('pre').forEach(pre => {
    if (pre.parentNode.classList.contains('code-wrapper')) return;

    const wrapper = document.createElement('div');
    wrapper.className = 'code-wrapper';
    pre.parentNode.insertBefore(wrapper, pre);
    wrapper.appendChild(pre);

    const button = document.createElement('button');
    button.className = 'copy-btn';
    button.innerText = 'Copy';

    button.addEventListener('click', (e) => {
        // Prevent triggering parent expanding click if inside api-details
        e.stopPropagation();

        const codeBlock = pre.querySelector('code');
        if (!codeBlock) return;

        navigator.clipboard.writeText(codeBlock.innerText).then(() => {
            button.innerText = 'Copied!';
            button.classList.add('copied');
            setTimeout(() => {
                button.innerText = 'Copy';
                button.classList.remove('copied');
            }, 2000);
        }).catch(err => {
            console.error('Failed to copy!', err);
        });
    });

    wrapper.appendChild(button);

    const code = pre.querySelector('code');
    if (code) {
        highlightCode(code);
    }
});

function highlightCode(element) {
    let html = element.textContent;
    // Fix: Ensure no leading/trailing whitespace impacts the rendering
    if (html) html = html.trim();

    // Fix: Escape HTML entities to prevent browser from interpreting <UNI.h> as a tag
    html = html.replace(/</g, '&lt;').replace(/>/g, '&gt;');

    const keywords = /\b(void|int|float|double|bool|char|long|unsigned|const|static|return|if|else|for|while|struct|class|public|private|new|delete|break|continue|switch|case|#define|#include)\b/g;
    const types = /\b(String|UniBase|UniBaseControl|UniDev|Adafruit_SSD1306|OdometryData)\b/g;
    const comments = /(\/\/.*)/g;
    const strings = /("[^"]*")/g;
    const numbers = /\b(\d+(\.\d+)?)\b/g;
    const functions = /\b([a-zA-Z_]\w*)(?=\()/g;

    const tokens = [];
    const saveToken = (match) => { items = tokens.push(match); return `___TOKEN${items - 1}___`; };

    html = html.replace(comments, saveToken);
    html = html.replace(strings, saveToken);

    html = html.replace(keywords, '<span class="token keyword">$1</span>');
    html = html.replace(types, '<span class="token type">$1</span>');
    html = html.replace(functions, '<span class="token function">$1</span>');
    html = html.replace(numbers, '<span class="token number">$1</span>');

    html = html.replace(/___TOKEN(\d+)___/g, (match, id) => {
        const token = tokens[id];
        if (token.startsWith('//')) return `<span class="token comment">${token}</span>`;
        if (token.startsWith('"')) return `<span class="token string">${token}</span>`;
        return token;
    });

    element.innerHTML = html;
}

// Sidebar active state is handled by renderSidebar() above.

// Global Search — built dynamically from the current page DOM
// ------------------------------------------------------------
// No hardcoded index: every api-card, section heading and example
// on the page is indexed automatically on the first keystroke.

let _searchIndex = null;

function _nearestSection(el) {
    let node = el;
    while (node) {
        let sib = node.previousElementSibling;
        while (sib) {
            if (sib.tagName === 'H2' && sib.id) return sib.id;
            sib = sib.previousElementSibling;
        }
        node = node.parentElement;
    }
    return '';
}

function buildSearchIndex() {
    const idx = [];
    const currentFile = (currentPath.split('/').pop() || 'index.html').split('#')[0];

    // Collapsible API cards (methods)
    document.querySelectorAll('.api-card:not(.static)').forEach(card => {
        const nameEl = card.querySelector('.api-name');
        const sigEl  = card.querySelector('.api-signature');
        const detEl  = card.querySelector('.api-details');
        if (!nameEl) return;
        idx.push({
            type:      'method',
            name:      nameEl.textContent.trim(),
            signature: sigEl  ? sigEl.textContent.trim() : '',
            detail:    detEl  ? detEl.textContent.replace(/\s+/g, ' ').trim().slice(0, 220) : '',
            anchor:    _nearestSection(card),
            file:      currentFile,
            el:        card
        });
    });

    // Static example cards
    document.querySelectorAll('.api-card.static[id]').forEach(card => {
        const h3 = card.querySelector('h3');
        const p  = card.querySelector('p');
        const pre = card.querySelector('pre');
        if (!h3) return;
        idx.push({
            type:      'example',
            name:      h3.textContent.trim(),
            signature: '',
            detail:    (p ? p.textContent : '') + ' ' + (pre ? pre.textContent.slice(0, 120) : ''),
            anchor:    card.id,
            file:      currentFile,
            el:        card
        });
    });

    // Section headings
    document.querySelectorAll('h2[id]').forEach(h => {
        idx.push({
            type:      'section',
            name:      h.textContent.trim(),
            signature: '',
            detail:    '',
            anchor:    h.id,
            file:      currentFile,
            el:        h
        });
    });

    // Cross-page links from nav.js (other pages only)
    if (window.NAV) {
        const lang = isRuPage ? 'ru' : 'en';
        (window.NAV[lang] || []).forEach(item => {
            if (!item.label || !item.href) return;
            const push = (it) => {
                const f = it.href.split('#')[0];
                if (f !== currentFile) idx.push({ type: 'page', name: it.label, signature: '', detail: '', anchor: it.href.split('#')[1] || '', file: f, el: null });
            };
            push(item);
            (item.children || []).forEach(push);
        });
    }

    return idx;
}

function _scrollToEl(el) {
    const hh = parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--header-height')) || 60;
    const top = el.getBoundingClientRect().top + window.scrollY - hh - 20;
    window.scrollTo({ top, behavior: 'smooth' });
}

const searchInput     = document.querySelector('.search-input');
const searchContainer = document.querySelector('.search-container');

if (searchInput) {
    const resultsDiv = document.createElement('div');
    resultsDiv.className = 'search-results';
    searchContainer.appendChild(resultsDiv);

    const noResultsText = isRuPage ? 'Ничего не найдено' : 'No results found';

    searchInput.addEventListener('input', (e) => {
        const raw = e.target.value;
        if (raw.length < 2) { resultsDiv.style.display = 'none'; return; }

        if (!_searchIndex) _searchIndex = buildSearchIndex();
        const q = raw.toLowerCase();

        const matches = _searchIndex.filter(item =>
            item.name.toLowerCase().includes(q) ||
            item.signature.toLowerCase().includes(q) ||
            item.detail.toLowerCase().includes(q)
        ).slice(0, 12);

        resultsDiv.innerHTML = '';
        if (matches.length === 0) {
            resultsDiv.innerHTML = `<div class="search-no-results">${noResultsText}</div>`;
        } else {
            matches.forEach(match => {
                const div = document.createElement('div');
                div.className = 'search-result-item';
                let ctx = match.file;
                if (match.signature) ctx = match.signature;
                else if (match.detail) ctx = match.detail.slice(0, 90) + (match.detail.length > 90 ? '…' : '');
                div.innerHTML = `<div class="result-title">${match.name}</div><div class="result-context">${ctx}</div>`;
                div.addEventListener('click', () => {
                    resultsDiv.style.display = 'none';
                    searchInput.value = '';
                    if (match.el) {
                        if (match.type === 'method' && !match.el.classList.contains('expanded'))
                            match.el.classList.add('expanded');
                        _scrollToEl(match.el);
                    } else {
                        window.location.href = match.anchor ? `${match.file}#${match.anchor}` : match.file;
                    }
                });
                resultsDiv.appendChild(div);
            });
        }
        resultsDiv.style.display = 'block';
    });

    document.addEventListener('click', (e) => {
        if (!searchContainer.contains(e.target)) resultsDiv.style.display = 'none';
    });

    searchInput.addEventListener('keydown', (e) => {
        if (e.key === 'Escape') { resultsDiv.style.display = 'none'; searchInput.value = ''; }
    });
}
