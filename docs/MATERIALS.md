# Материалы проекта

Индекс собранной документации. **Перед веб-поиском** — смотреть сюда.

Правила сбора: `C:\Users\Валерий\.grok\rules\project-docs.md`

## Статус

| Показатель | Значение |
|------------|----------|
| Последнее обновление | 2026-10-31 |
| Всего материалов | 5 |

## Индекс

| Дата | Название | Тип | URL | Локальный файл | Зачем нужен |
|------|----------|-----|-----|----------------|-------------|
| 2026-10-31 | ТЗ: робот-боксёр на MG90S (ROBO-ONE) | note | — (от пользователя) | `notes/2026-10-31_robo_one_boxing_spec.md` | исходное техническое задание |
| 2026-10-31 | MG90S: сводка размеров продавцов (официального чертежа нет) | note | getfpv.com, coolcomponents.co.uk, controllerstech, kpower blog | `notes/2026-10-31_mg90s_dimensions_sources.md` | откуда параметры модели и что проверить замером |
| 2026-10-31 | MG90S 3D-модель (GrabCAD «Servo, Metal Gear, MG90S») | cad | grabcad.com (скачано пользователем, `D:\Doks\RobotONE\servo-metal-gear-mg90s-1.snapshot.1`) | `../cad/purchased/MG90S_grabcad.SLDPRT`, `.STEP` | геометрия сервы для сборки (вал x=5,5, ушки ±13,75) |
| 2026-10-31 | «Servo Horn.stp» (GrabCAD mg90s-servo-motor-2) | cad | grabcad.com (у пользователя) | `../cad/purchased/MG90S_horn.stp` | оказался переходником Ø22 со штырями, НЕ штатная качалка — не использовать как размер качалки |
| 2026-10-31 | Наработки пользователя: Голень, Стопа, RobotOne (STL/КОМПАС) | cad | — (`D:\Doks\RobotONE`) | `../cad/legacy_robotone/` | подобранное гнездо MG90S 23,21×13,0 (scripts/stl_pockets.py) |
| 2026-11-01 | TowerPro MG90S — характеристики производителя | web | https://www.towerpro.com.tw/product/mg90s-3/ | `web/2026-11-01_towerpro_mg90s_specs.md` | масса 13,4 г, stall 1,8 кгс·см @4,8 В / 2,2 @6,6 В, 0,10 с/60° — масса и предел момента в физической модели Motion |
| 2026-11-15 | Качалки MG90S в наличии (крест, двуплечая, одноплечая) на клетке 5×5 мм | image | — (фото пользователя) | `images/2026-11-15_mg90s_horns_grid5_1..3.jpg`, `images/2026-11-15_horn_{single,double,cross}_mm_overlay.png` | реальные размеры качалок (`cad/mg90s_params.json` → `horns_stock`); условные размеры кармана не совпали |

## Типы

- `web` — страница (md/html)
- `pdf` — datasheet, manual
- `image` — фото, схема
- `note` — выжимка, если файл недоступен

## Поисковые запросы (уже проверено)

- «TowerPro MG90S datasheet dimensions mm mounting hole spacing» — официального чертежа нет; корпус 22,8×12,2×28,5 у большинства продавцов; шаг отверстий — только блог KPower (не подтверждён).
- SG90 datasheet (datasheetcafe) — в тексте только 22,2×11,8×31 «approx.», чертёж — картинкой.
- Подшипники 685/623 — из базы SolidWorks Toolbox (ISO 15): 685 = «385» 5×11×5, 623 = «023» 3×10×4.
