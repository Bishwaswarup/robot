# Paper source

*Gait-Aware Command Timing for Decentralized Quadruped Swarms* — preprint:
[doi:10.21203/rs.3.rs-11332226/v1](https://doi.org/10.21203/rs.3.rs-11332226/v1) (Research Square, under review).

Springer Nature LaTeX template (`sn-jnl.cls`, `sn-mathphys-num.bst`: numbered references, decimal headings).

    latexmk -pdf main.tex        # or: pdflatex main && bibtex main && pdflatex main && pdflatex main

| File | Content |
|---|---|
| `main.tex` | the whole manuscript (title page, abstract, body, declarations, appendix) |
| `refs.bib` | bibliography with DOIs |
| `sn-jnl.cls`, `sn-mathphys-num.bst` | Springer Nature template files (LPPL) |
| `figures/` | Figs. 1 and 3–8 (PDF for LaTeX, PNG for the README); made by `python scripts/paper_figures.py` |

Fig. 2 (the control stack) is drawn in TikZ inside `main.tex`. Tables 4–6 and A1 come from
`python scripts/paper_tables.py`. The supplementary video (Online Resource 1) is
[`docs/media/online_resource_1.mp4`](../docs/media/online_resource_1.mp4), made by `python scripts/make_video.py`.
