# Push-to-talk voice over the radio

A browser front end that drives two boards: hold TRANSMIT, speak, and the
far station writes a `.wav`. Speech is carried by LSCodec-25Hz at 250 bit/s
over BROADCAST (ptype 15, non-ARQ) -- nothing is acknowledged and nothing
is retransmitted, because for speech a late repeat is worse than a gap.

    LSCODEC_HOME=/mnt/data/lscodec/adapter \
        /mnt/data/lscodec/adapter/venv/bin/python host/webvoice/server.py
    # then open http://localhost:8080

## The external dependency

The codec is NOT in this repository: LSCodec-Inference, its checkpoints
(`ckpt/lscodec_25hz`) and a torch venv live outside it. Two knobs locate
them, both with defaults matching this stand:

- `LSCODEC_HOME` -- the directory holding `LSCodec-Inference/` and `ckpt/`
- `WAVLM_CKPT` -- WavLM-Large.pt, used to derive the speaker prompt

The OFDM half (`host/ofdm_modem.py`) is imported from this tree by
relative path, so the app moves with the repository.

## Two invariants worth keeping

**Audio ingest is serialized.** The HTTP server is threaded and the
browser posts from `onaudioprocess`, so without the lock in `feed()` every
block of speech runs its own thread through the encoder and the broadcast
sender at once. Measured: 27 collisions in one 20 s transmission. The
damage is not subtle -- `k`, `pending` and `_tokcarry` are
read-modify-written, and in `_send_chunk` `bc_open` was read before the
USB write and assigned after it, so two threads both saw it clear and both
sent a NON-continuation command. `bc_cmd()` treats the second as a new
broadcast: it closes the one on the air, overwrites the unsent bytes and
resets the sequence to 0. The browser chains its posts for the same
reason -- two concurrent `fetch()` calls have no wire ordering guarantee,
and `stopTalk` drains the chain so the close cannot overtake the last
block of speech.

**Warm-up waits for the exchange to FINISH.** It used to return the moment
the far station decoded the message, while the acknowledgement it owed was
still to be keyed -- and the link is half duplex, so the broadcast went out
into a deaf receiver and lost a group whole. `settle()` waits for the
transmit board's `pending` to clear and its queues to drain. Measured over
39 transmissions on a clean cross-wire at rung 12: 5 losses in 23 without
it, 0 in 16 with it.

A lost group is 30 bytes -- one SYNC frame plus its partner, 0.96 s of
speech -- and the receiver only learns of it from the sequence gap in the
next SYNC, so nothing logs it. Each recording is written with the sent and
received byte streams beside it (`tx_*.bin` / `rx_*.bin`) precisely so the
next one can be located rather than inferred.
