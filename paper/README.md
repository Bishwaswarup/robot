# Paper source

`main.tex` builds the manuscript in a neutral single-column layout with any TeX Live:

    latexmk -pdf main.tex

`main_sn.tex` is the same paper in the Springer Nature `sn-jnl` class. Download the Springer Nature LaTeX
template (Overleaf: "Springer Nature LaTeX Template"), copy `sn-jnl.cls` and `sn-mathphys-num.bst` here, then

    pdflatex main_sn && bibtex main_sn && pdflatex main_sn && pdflatex main_sn

Both read the shared text from `abstract.tex`, `body.tex`, `declarations.tex` and `refs.bib`.
Figures 1 and 3-6 are made by `python scripts/paper_figures.py` (from `results/` and `data/`);
the stack diagram is TikZ inside `body.tex`. Table 3 numbers come from `python scripts/paper_tables.py`.
Bibliography entries marked `% VERIFY` were written from memory: check them against the originals.
