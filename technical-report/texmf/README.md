Vendored LaTeX packages so the report builds without `texlive-science`:

- `algorithmicx.sty`, `algpseudocode.sty` -- Szasz Janos, LPPL, from CTAN
  (macros/latex/contrib/algorithmicx). The `algorithm` float is defined in
  `report.tex` with the `float` package instead of the `algorithms` bundle.

The Makefile prepends this directory to `TEXINPUTS`.
