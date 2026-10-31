# robo-boxer — инструкции для агента

- Перед работой: README.md, `docs/MATERIALS.md`, ТЗ `docs/notes/2026-10-31_robo_one_boxing_spec.md`, skill `robo-boxer`.
- CAD — через solidworks-mcp (`D:\AI prodject\grok\projects\solidworks-mcp\SolidworksMCP-python\`), harness `scripts/audit_harness.py`; правила SW: `~/.grok/rules/solidworks.md`. Готовое первым: подшипники, штифты, винты — **Toolbox** (`create_toolbox_part`), отверстия под крепёж — **Hole Wizard**.
- **Размеры покупных изделий не угадывать.** MG90S: официального чертежа нет — размеры в `cad/mg90s_params.json` с источником и флагом `verify`; после замеров пользователя обновить JSON и перестроить.
- Печать FDM: карманы под покупные изделия +0,2 мм на сторону; посадка подшипника — отверстие = D (номинал) с правкой по пробной печати; стенки 2–2,5 мм, рёбра.
- Ударная нагрузка — в корпус через подшипник 685ZZ и ось 623ZZ, на вал сервопривода только момент (через качалку).
- Окна SW закрывать, в конце 0 открытых документов.
