# Paper source

Manuscript for the *International Journal of Intelligent Robotics and Applications* (Springer), in the
Springer Nature LaTeX template (`sn-jnl.cls`, `sn-mathphys-num.bst`: numbered references, decimal headings).

    latexmk -pdf main.tex        # or: pdflatex main && bibtex main && pdflatex main && pdflatex main

| File | Content |
|---|---|
| `main.tex` | the whole manuscript (title page, abstract, body, declarations) |
| `refs.bib` | bibliography; entries marked `% VERIFY` were written from memory, check them against the originals |
| `sn-jnl.cls`, `sn-mathphys-num.bst` | Springer Nature template files (LPPL) |
| `figures/` | Figs. 1, 3-6 (PDF for LaTeX, PNG for the README); made by `python scripts/paper_figures.py` |
| `Nayak_GaitPhaseAware_QuadrupedSwarms.pdf` | compiled manuscript |

Fig. 2 (the control stack) is drawn in TikZ inside `main.tex`. Table 3 comes from `python scripts/paper_tables.py`.
