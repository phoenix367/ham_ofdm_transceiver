---
name: build-report
description: Build the technical report PDF, render mermaid diagrams, and check the result page by page. Use after editing anything under technical-report/ - sections, figures, or a .mmd diagram source.
---

# Building the technical report

```bash
cd technical-report && make all && make all      # twice: the first pass leaves refs undefined
pdfinfo build/OFDM_Transceiver_Technical_Report.pdf | grep Pages
```

Check for trouble between the two passes, not after (the second pass
hides what the first one said):

```bash
make all 2>&1 | grep -iE 'undefined|error' | grep -v 'Font shape'
```

`Font shape T1/cmr/m/scit undefined` is chronic and harmless; anything
else is real. `LaTeX Warning: Reference ... undefined` after the SECOND
pass means a label is misspelt.

## Diagrams

Sources are `images/*.mmd`, rendered by the Makefile rule with `mmdc`
(`-s 3 -b white`, puppeteer `--no-sandbox` config). To render one by
hand while iterating:

```bash
mmdc -i images/x.mmd -o images/x.png -s 3 -b white --puppeteerConfigFile images/puppeteer-config.json
```

- **Mermaid treats `;` as a statement separator.** A `;` followed by a
  space anywhere in a message or note breaks the parse, and the error is
  reported one line LATER than the offender. Use an em dash. (A `;`
  directly before `<br/>` survives, which is how one got through.)
- Existing sequence diagrams are the style reference:
  `images/sendfile_seq.mmd`, `images/usb_reopen_seq.mmd` (shaded
  `rect` stages).

## Checking pages

Physical page = printed folio + 1 (front matter is roman-numbered).
`.aux` `\newlabel` entries give folios. Render and look:

```bash
pdftoppm -png -r 60 -f <physical> -l <physical> build/OFDM_Transceiver_Technical_Report.pdf /tmp/pg
```

Then Read the PNG. Look for: a figure pushed to the next page leaving
half a page blank (a tall figure placed with `[H]` -- use `[htbp]` so
the following text fills the gap), overfull lines, and a `Table 22`
that came out as `Table ??`.

## Where new material goes

`sections/12b-c-implementation.tex` holds the C port, memory, on-target
measurement, USB and demonstration subsections; new subsections take a
`\label{sec:...}` and get cross-referenced from `13-discussion.tex` if
they change a conclusion. Numbers in the text must match
`cport/FEASIBILITY.md`, which is the measured record; when they
disagree, the report is the one that is wrong.
