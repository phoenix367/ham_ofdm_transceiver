---
name: test-python
description: Run the Python model's test suites - the 44 bit-exact article checks (verify_article), the float-vs-fixed equivalence suite (fixed_point), and the end-to-end smoke test. Use after any change under ofdm_phy/, especially to ofdm_phy/fixed/, or when asked to verify the float or fixed model.
---

# Testing the Python model

Everything runs through the repo venv. From the repo root:

```bash
./venv/bin/python experiments/verify_article.py   # 44 checks, ~10 s
./venv/bin/python experiments/fixed_point.py      # 28 checks, ~3 min
./venv/bin/python experiments/smoke_e2e.py        # 6 cases,  ~1 min
```

Expected output is a `N passed, 0 failed` line from each. Anything else
is a regression — report the failing check by name, do not summarise it
as "some tests failed".

## Which to run

- **`ofdm_phy/` changed** → `verify_article.py`. It is the fast one and
  covers CRCs, Base38/QTH, conv-code outputs, scrambler and interleaver
  sequences bit-exactly against the article's worked examples.
- **`ofdm_phy/fixed/` changed** → `fixed_point.py` as well. This is the
  suite that validates the integer model against the float one; it is
  the only place a fixed/float divergence is caught.
- **Anything in the signal chain** → add `smoke_e2e.py` for a real
  TX→channel→RX pass including CFO and the −6 dB article channel.

`fixed_point.py` takes ~3 minutes, which exceeds a single foreground
command budget. Run it with `run_in_background: true` and collect the
result, or expect it to be moved to the background automatically.

## Interpreting a failure

These suites compare against golden values with no tolerances, so a
failure is a real behavioural change, not noise. Two failure modes worth
distinguishing:

- a *single* named check failing usually points at the specific
  transform it names;
- the fixed-point suite failing while `verify_article` passes means the
  float model is fine and the integer twin has diverged — check the
  invariants in CLAUDE.md before assuming the vectors are stale.

Golden vectors are regenerated with `cport/gen_vectors.py`, but do not
regenerate them to make a test pass unless the wire format genuinely
changed. That converts a caught bug into a silent one.
