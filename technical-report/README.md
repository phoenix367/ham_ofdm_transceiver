# Technical report

LaTeX sources for the project's technical report: article reproduction,
synchronization analysis, adaptive modes, LLR recalibration, LDPC/16-QAM/HARQ,
link layer, AFC netting, RF chain, fixed-point RTL model, and consolidated
results, with verbatim regression-suite outputs as appendices.

## Building

Requires `pdflatex`, `bibtex`, and `mmdc` (mermaid-cli) on PATH.

```bash
make          # render mermaid diagrams, copy result plots, build the PDF
make quick    # single pdflatex pass, no bibliography
make clean    # remove LaTeX aux files
make cleanall # also remove the PDF and rendered images
```

Output: `build/OFDM_Transceiver_Technical_Report.pdf`.

## Layout

- `report.tex` — preamble, title page, abstract, section/appendix includes
- `sections/` — one file per numbered section
- `appendices/` — appendix text + verbatim captures of
  `verify_article.py`, `fixed_point.py`, and `demo_wav.py` output
  (re-run those scripts to refresh the captures)
- `images/` — mermaid `.mmd` sources (rendered by the Makefile) and
  measured plots copied from `../results/` (regenerate with the
  corresponding experiments before rebuilding if the results changed)
- `references.bib` — bibliography
