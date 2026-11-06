'use strict';

// Single source of truth for all sidebar navigation.
// To add a page: add one object here. All HTML files update automatically.
//
//   { section: 'Title' }                        a heading in the sidebar
//   { label, href, children: [...] }           a page; children are its sections
//   { group: 'Title', icon, items: [] }        a fold-out list of pages; starts folded
//   { label, soon: true, icon }                 announced, not written yet: shown, not linked
//   { anchor: 'Title', href, icon }            a highlighted link above the sections
window.NAV = {
  en: [
    { anchor: 'UNI.h reference', href: 'reference.html', icon: 'reference' },

    { label: 'Home',            href: 'index.html' },
    { label: 'Getting Started', href: 'getting-started.html' },
    { group: 'Tutorials', icon: 'lesson', items: [
        { label: '1. First drive',            href: 'tutorial-first-drive.html' },
        { label: '2. Turns and waypoints',    href: 'tutorial-odometry.html' },
        { label: '3. Arcs',                   href: 'tutorial-arcs.html' },
        { label: '4. Moving without waiting', href: 'tutorial-async.html' },
    ]},

    // The hardware pages are being reworked and are not published yet.
    { section: 'Hardware' },
    { label: 'UniBase',             soon: true, icon: 'hardware' },
    { label: 'Attachments',         soon: true, icon: 'attachments' },
    { label: 'Your own attachment', soon: true, icon: 'attachment' },
    { label: 'Connection protocol', soon: true, icon: 'protocol' },

    { section: 'Software' },
    { label: 'UNI IDE', soon: true, icon: 'ide' },
    { group: 'UNI.h library', icon: 'library', items: [
        { label: 'Overview', href: 'library-overview.html' },
        { label: 'UNI.h reference', href: 'reference.html', children: [
          { label: 'UniBase', href: 'reference.html#unibase' },
          { label: 'UniDev', href: 'reference.html#unidev' },
        ]},
        { label: 'UniBase class', href: 'unibase.html', children: [
          { label: 'Constants',        href: 'unibase.html#constants' },
          { label: 'Initialization',   href: 'unibase.html#initialization' },
          { label: 'Motor Control',    href: 'unibase.html#motor-control' },
          { label: 'Movement',         href: 'unibase.html#movement' },
          { label: 'Async Movement',   href: 'unibase.html#async-movement' },
          { label: 'Odometry',         href: 'unibase.html#odometry' },
          { label: 'Speed',            href: 'unibase.html#speed' },
          { label: 'Display',          href: 'unibase.html#display' },
          { label: 'Utility',          href: 'unibase.html#utility' },
        ]},
        { label: 'UniDev class', href: 'unidev.html', children: [
          { label: 'Port Definitions',     href: 'unidev.html#constants' },
          { label: 'Initialization',       href: 'unidev.html#dev-init' },
          { label: 'Sensors',              href: 'unidev.html#sensors' },
          { label: 'Inputs',               href: 'unidev.html#inputs' },
          { label: 'Servo',                href: 'unidev.html#servo' },
          { label: 'Light Ring (Basic)',   href: 'unidev.html#light-ring-basic' },
          { label: 'Light Ring (Effects)', href: 'unidev.html#light-ring-effects' },
          { label: 'Traffic Light',        href: 'unidev.html#traffic-light' },
        ]},
        { label: 'Examples', href: 'examples.html', children: [
          { label: 'Start',            href: 'examples.html#start' },
          { label: 'Precision Turns',  href: 'examples.html#precision-turns' },
          { label: 'Drive to Point',   href: 'examples.html#drive-to-point' },
          { label: 'Arcs',             href: 'examples.html#arcs' },
          { label: 'Async Movement',   href: 'examples.html#async' },
          { label: 'Manual Control',   href: 'examples.html#manual-control' },
          { label: 'Obstacle Stop',    href: 'examples.html#obstacle-stop' },
          { label: 'Distance Sensors', href: 'examples.html#distance' },
          { label: 'Line Sensor',      href: 'examples.html#line' },
          { label: 'Servo',            href: 'examples.html#servo' },
          { label: 'UART Control',     href: 'examples.html#uart-control' },
        ]},
        { label: 'Limitations', href: 'limitations.html', children: [
          { label: 'Movement & Commands',   href: 'limitations.html#movement' },
          { label: 'Accuracy & Odometry',   href: 'limitations.html#accuracy' },
          { label: 'Behavior & Blocking',   href: 'limitations.html#behavior' },
          { label: 'Peripherals',           href: 'limitations.html#peripherals' },
        ]},
    ]},
    { label: 'FAQ', href: 'faq.html', children: [
      { label: 'Installing and uploading',  href: 'faq.html#install' },
      { label: 'Movement',                  href: 'faq.html#movement' },
      { label: 'Sensors, screen and link',  href: 'faq.html#peripherals' },
      { label: 'Debug Tips',                href: 'faq.html#debug-tips' },
    ]},
  ],
  ru: [
    { anchor: 'Справочник UNI.h', href: 'reference_ru.html', icon: 'reference' },

    { label: 'Главная',        href: 'index_ru.html' },
    { label: 'Начало работы', href: 'getting-started_ru.html' },
    { group: 'Уроки', icon: 'lesson', items: [
        { label: '1. Первый заезд',              href: 'tutorial-first-drive_ru.html' },
        { label: '2. Повороты и езда по точкам', href: 'tutorial-odometry_ru.html' },
        { label: '3. Дуги',                      href: 'tutorial-arcs_ru.html' },
        { label: '4. Движение без ожидания',     href: 'tutorial-async_ru.html' },
    ]},

    { section: 'Hardware' },
    { label: 'UniBase',             soon: true, icon: 'hardware' },
    { label: 'Насадки',             soon: true, icon: 'attachments' },
    { label: 'Своя насадка',        soon: true, icon: 'attachment' },
    { label: 'Протокол соединения', soon: true, icon: 'protocol' },

    { section: 'Software' },
    { label: 'UNI IDE', soon: true, icon: 'ide' },
    { group: 'Библиотека UNI.h', icon: 'library', items: [
        { label: 'Обзор', href: 'library-overview_ru.html' },
        { label: 'Справочник UNI.h', href: 'reference_ru.html', children: [
          { label: 'UniBase', href: 'reference_ru.html#unibase' },
          { label: 'UniDev', href: 'reference_ru.html#unidev' },
        ]},
        { label: 'Класс UniBase', href: 'unibase_ru.html', children: [
          { label: 'Константы',          href: 'unibase_ru.html#constants' },
          { label: 'Инициализация',      href: 'unibase_ru.html#initialization' },
          { label: 'Управление моторами', href: 'unibase_ru.html#motor-control' },
          { label: 'Движение',           href: 'unibase_ru.html#movement' },
          { label: 'Асинхронное движение', href: 'unibase_ru.html#async-movement' },
          { label: 'Одометрия',          href: 'unibase_ru.html#odometry' },
          { label: 'Скорость',           href: 'unibase_ru.html#speed' },
          { label: 'Дисплей',            href: 'unibase_ru.html#display' },
          { label: 'Утилиты',            href: 'unibase_ru.html#utility' },
        ]},
        { label: 'Класс UniDev', href: 'unidev_ru.html', children: [
          { label: 'Определения портов',      href: 'unidev_ru.html#constants' },
          { label: 'Инициализация',           href: 'unidev_ru.html#dev-init' },
          { label: 'Датчики',                 href: 'unidev_ru.html#sensors' },
          { label: 'Входы',                   href: 'unidev_ru.html#inputs' },
          { label: 'Сервопривод',             href: 'unidev_ru.html#servo' },
          { label: 'Световое кольцо (Базовое)',  href: 'unidev_ru.html#light-ring-basic' },
          { label: 'Световое кольцо (Эффекты)', href: 'unidev_ru.html#light-ring-effects' },
          { label: 'Светофор',               href: 'unidev_ru.html#traffic-light' },
        ]},
        { label: 'Примеры', href: 'examples_ru.html', children: [
          { label: 'Старт',               href: 'examples_ru.html#start' },
          { label: 'Точные повороты',     href: 'examples_ru.html#precision-turns' },
          { label: 'Езда по точкам',      href: 'examples_ru.html#drive-to-point' },
          { label: 'Дуги',                href: 'examples_ru.html#arcs' },
          { label: 'Асинхронные команды', href: 'examples_ru.html#async' },
          { label: 'Прямое управление',   href: 'examples_ru.html#manual-control' },
          { label: 'Остановка у препятствия', href: 'examples_ru.html#obstacle-stop' },
          { label: 'Датчики расстояния',  href: 'examples_ru.html#distance' },
          { label: 'Датчик линии',        href: 'examples_ru.html#line' },
          { label: 'Сервопривод',         href: 'examples_ru.html#servo' },
          { label: 'Управление UART',     href: 'examples_ru.html#uart-control' },
        ]},
        { label: 'Ограничения', href: 'limitations_ru.html', children: [
          { label: 'Движение и команды',     href: 'limitations_ru.html#movement' },
          { label: 'Точность и одометрия',   href: 'limitations_ru.html#accuracy' },
          { label: 'Поведение и блокировки', href: 'limitations_ru.html#behavior' },
          { label: 'Периферия',              href: 'limitations_ru.html#peripherals' },
        ]},
    ]},
    { label: 'FAQ', href: 'faq_ru.html', children: [
      { label: 'Установка и загрузка',   href: 'faq_ru.html#install' },
      { label: 'Движение',               href: 'faq_ru.html#movement' },
      { label: 'Датчики, экран и связь', href: 'faq_ru.html#peripherals' },
      { label: 'Советы по отладке',      href: 'faq_ru.html#debug-tips' },
    ]},
  ],
};
