# Vendored RLE-Bench code

Source: https://github.com/RLE-Bench/RLE-Bench at commit 34a73aa3daebd92a4adc56142b87f2bccda4780d, MIT licence (LICENSE, copyright 2026 RLE-Bench contributors).

| Here | Upstream | Changes |
|---|---|---|
| tabletop/__init__.py | tasks/task03/tabletop/__init__.py | none |
| tabletop/scenes.py | tasks/task03/tabletop/scenes.py | none |
| tabletop/analytic.py | tasks/task03/tabletop/analytic.py | none |
| tabletop/budgets.py | tasks/task03/tabletop/budgets.py | none |
| tabletop/hidden_com/__init__.py, config.py, scene.py | tasks/task03/tabletop/hidden_com/ | none |

Robo Use runs these scenes in its RLE-Bench simulator worker (../worker.py); the episode server, the step and time budgets and the scoring threshold are Robo Use's (../backend.py).
