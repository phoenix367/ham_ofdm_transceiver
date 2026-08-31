---
name: audit-docs
description: Check the repository's prose against the code - docs/, the READMEs, CLAUDE.md and the technical report - and fix what drifted. Use when a feature has landed, when a document is being trusted for a number, or when asked to verify documentation accuracy.
---

# Auditing the prose against the code

Documentation here is narrative; the code is normative. Where they
disagree the document is the bug -- unless the number came from a
measurement, in which case check `cport/FEASIBILITY.md` and the report,
which are the measured record.

Five bodies of prose must move together: `docs/` (twelve documents),
`docs/README.md`'s index table, the root `README.md` link list,
`CLAUDE.md`'s invariants, and `technical-report/sections/`.

## Sweep 1 -- stale constants

Grep each document for its numbers and compare with the defining
source. The ones that have actually drifted:

```bash
grep -rn "ST_MSG_MAX\|MAX_LLRS\|UP_MAX_PAYLOAD\|BURST_STREAM_MAX" cport/Makefile cport/src/*.h
grep -rnE "[0-9]+ ?(B/s|kB|bytes|dB)" docs/*.md | less   # then check each
```

Drift concentrates wherever the last campaign moved: message sizes,
window ceilings, throughput figures, which RAM a struct lives in. It
does not appear in the parts of the system that stopped changing.

## Sweep 2 -- omissions

Enumerate the code's sets and diff them against what the prose lists.

```bash
# every experiment script is documented
for f in experiments/*.py; do grep -q "$(basename $f)" docs/experiments.md || echo "UNLISTED $f"; done
# counts stated in prose
grep -oE "ST_EV_[A-Z_]+" cport/src/station.h | sort -u | wc -l    # diag events
grep -oE "UP_CFG_[A-Z_]+" cport/src/usb_proto.h | sort -u | wc -l # config keys
# index and link list cover every document
ls docs/*.md | while read f; do grep -q "$(basename $f)" docs/README.md README.md || echo "UNINDEXED $f"; done
```

Then read for what is *absent* rather than wrong: a mechanism added on
top of a documented one (an output map over a raw estimator), a
directory tree that predates half the repository, a command that grew
a second form.

## Reporting

Say what was verified clean as well as what was fixed -- "rf.md and
c-port-plan.md checked, unchanged" is a result. A plan document that
self-identifies as a historical plan is not drift; leave it.

Findings from the last pass, as a calibration of what this catches: a
freshly written event count that said nineteen where the enumeration
has seventeen, a broadcast command described as a single small
transfer three revisions after it became chunk-fed, a struct still
documented in the RAM it was moved out of, an estimator documented
without the map bolted on top of it, and a source layout that stopped
at the Python tree.
