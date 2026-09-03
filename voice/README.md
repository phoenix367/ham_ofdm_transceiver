# Adapter training pipeline: FACodec timbre -> LSCodec prompt

Builds the dataset and trains the adapter that would let a **256-dim
speaker vector** stand in for LSCodec's 99x1024 WavLM prompt.

## Why this exists

LSCodec carries speech at 250 bit/s but keeps speaker identity out of the
bitstream entirely: identity lives in a *prompt*, a 2-second WavLM feature
sequence (203 kB) that must be enrolled once per correspondent. Replacing
the prompt with a compact speaker vector would remove both the enrolment
exchange and the 1.2 GB WavLM dependency at every station.

Measured on this vocoder, decoding the same tokens under different prompts
(mean absolute log-mel distance vs the true prompt):

| prompt | mel dB |
|---|---|
| true prompt, full sequence | 0.00 |
| **ceiling for a K-frame summary, K>=2** | **~2.0-2.5** |
| ceiling for ONE broadcast vector | ~4.1 |
| FACodec timbre + untrained linear adapter | 7.5 |
| a different real speaker | 8.9 |

Two things follow, and both are built into these scripts.

**Predict K frames, not one vector.** The vocoder cross-attends over the
prompt, so a constant memory discards the temporal detail it was trained
to use. Going from 1 frame to 2 is worth ~1.7 dB and costs one byte.
`--k 8` is the default.

**The adapter is the cheap half.** It changes no bitstream and needs no
vocoder retraining: a peer running stock weights still decodes. Removing
the ~2 dB ceiling entirely would mean retraining the vocoder's prompt
path, which is a different and much larger job.

## Where things live

The scripts are in the repo (`voice/`); everything heavy is **not**. The
venv, the third-party trees (LSCodec-Inference, naturalspeech3_facodec),
the checkpoints, the dataset shards and the trained adapters live under
`LSCODEC_HOME` -- the same knob the web app (`host/webvoice/`) reads --
defaulting to `/mnt/data/lscodec/adapter` on this stand. `setup.sh` builds
that directory; nothing it produces belongs in version control, and
`voice/.gitignore` keeps it out even if the pipeline is run in-tree.

Every script takes `LSCODEC_HOME` from the environment (and `WAVLM_CKPT`
for the WavLM checkpoint); the CLI flags still override per run.

## Setup

    LSCODEC_HOME=/path/to/home voice/setup.sh   # venv, repos, checkpoints (~600 MB)

The default torch index is **cu121**, deliberately. The GTX 1050 is Pascal
(sm_61) and **CUDA 13 dropped Pascal support entirely** -- a cu126/cu128/cu13
wheel installs without complaint and then fails at the first kernel launch
with `no kernel image is available for execution on the device`. If
`torch.cuda.is_available()` is False, check `torch.__version__` first: a
`+cpu` suffix means the wheel, not the driver, and no reboot will fix it.

    TORCH_INDEX=https://download.pytorch.org/whl/cpu ./setup.sh   # force CPU

`WavLM-Large.pt` (1.2 GB, fairseq format with `cfg` and `model` keys) is
**not** scripted: the official Azure link returns `AuthenticationFailed`
and the Google Drive mirror throttles to tens of kB/s. Put it at
`~/Downloads/WavLM-Large.pt` or pass `--wavlm`.

## Run

    H=$LSCODEC_HOME
    LSCODEC_HOME=$H $H/venv/bin/python voice/extract.py --n 5000   # ~26 min
    LSCODEC_HOME=$H $H/venv/bin/python voice/train_adapter.py --data $H/data/
    LSCODEC_HOME=$H $H/venv/bin/python voice/eval_adapter.py \
        --adapter $H/adapter.pt --wav held_out.wav

There is also a standalone codec round-trip that needs no training --
encode a wav to the 250 bit/s token stream and decode it back, the exact
path the radio uses minus the air:

    LSCODEC_HOME=$H $H/venv/bin/python voice/roundtrip.py in.wav \
        --prompt speaker.wav -o out.wav --bitstream out.lsc
    LSCODEC_HOME=$H $H/venv/bin/python voice/restore.py out.lsc prompt.bin

Extraction is **network-bound, not compute-bound** -- each utterance waits on
an HTTP fetch from the HF datasets server, and the models are the cheap part.
Measured: 0.76 utt/s on CPU, 0.89 on the GPU (no real gain), **3.23 utt/s**
once `--fetch-workers 12` overlaps the downloads. The GPU earns its keep in
`train_adapter.py`, not here.

`extract.py` is resumable: it records which utterance ids are already in
the shards and skips them, so interrupting it costs nothing. It refuses
to start if the disk cannot hold the plan.

## Things that will bite

- **Split by speaker, never by utterance.** `train_adapter.py` does this.
  An utterance split leaks the speaker into training and the validation
  number becomes meaningless.
- **Cosine is a proxy.** The number that decides the question is the
  decode, which is what `eval_adapter.py` measures and writes to WAV.
  Listen to them: the metric consistently understates how usable the
  audio is.
- **train.clean.360 has 1151 speakers.** With `--per-speaker 4` that caps
  the set near 4600 utterances; raise `--per-speaker` or add
  `--split train.clean.100` for more.
- The adapter predicts the **residual** after the prompt's global mean.
  That common component is large enough to swallow the whole model
  capacity if predicted directly.
