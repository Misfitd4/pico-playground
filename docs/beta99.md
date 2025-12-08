# Beta99 Workflow (WIP)

`beta99` is the next iteration of the SID playback pipeline. Instead of
streaming raw register frames (FDIS) to the Pico, we pre-process each tune into
**SID Sound Fragments (SSFs)** and a lightweight trigger timeline. The Pico only
needs to replay the deduplicated SSFs when their triggers arrive, which reduces
USB bandwidth, smooths playback, and matches how musicians authored the
original tracks.

## Stages

1. **Dump** – run VICE `vsid` with the `dump` sound device to capture every SID
   register write in SID-clock order:
   ```sh
   vsid -sounddev dump -soundarg tune.sid.dump -warp -limit 300000000 tune.sid
   ```
2. **Desidulate** – feed the dump into `reg2ssf`/`desidulate` to produce two
   CSV files (usually zstd-compressed):
   - `*.ssf.zst`: unique SSFs (register snapshots between a voice's gate-on and
     gate-off). Columns include `hashid`, `clock`, oscillator flags, ADSR, etc.
   - `*.log.zst`: chronological trigger log showing when each SSF fires.
3. **Pack** – convert the SSF table + trigger log into a single `beta99` asset
   that the Pico can stream or preload.
4. **Playback** – the Pico's `beta99` runtime loads the SSFs, then walks the
   trigger timeline, applying each fragment to the SID engine at the proper
   clock.

## Beta99 File Format (draft)

Binary container written little-endian:

| Offset | Size | Description |
|--------|------|-------------|
| 0x00   | 4    | Magic `B99\0` |
| 0x04   | 2    | Version (currently 1) |
| 0x06   | 2    | Reserved |
| 0x08   | 4    | SSF count |
| 0x0C   | 4    | Trigger count |
| 0x10   | ...  | SSF table (each entry has a header + packed register writes) |
| ...    | ...  | Trigger list (delta clock, SSF index, voice/filter flags) |

The exact SSF encoding is still being evaluated. The current working plan is to
store each fragment as:

```
struct beta99_ssf {
    uint64_t hashid;
    uint32_t gate_clock;   // relative to start of fragment
    uint16_t voice_mask;   // which voices/registers are touched
    uint16_t write_count;
    struct { uint8_t addr; uint8_t value; uint32_t delta; } writes[write_count];
};
```

Triggers reference SSFs by index and carry an absolute SID clock so the runtime
can schedule them accurately.

## Tools

- `tools/beta99_pack.py` – helper script that reads `*.ssf.zst` and `*.log.zst`
  files, validates their contents, and emits a preliminary `*.b99.json` bundle.
  The JSON version is meant for inspection and debugging; a binary writer will
  follow once the SSF/trigger encoding is finalized. Run it manually or let
  `sid2serial -Z <out>` call it for you.

Manual usage:

```sh
python tools/beta99_pack.py \
    --ssf Bromance-Intro.ssf.zst \
    --log Bromance-Intro.log.zst \
    --out Bromance-Intro.b99.json
```

The script expects the SSF CSV to expose at least the following columns:

```
hashid,clock,pr_frame,gate1,freq1,pwduty1,pulse1,noise1,tri1,saw1,test1,
sync1,ring1,freq3,test3,flt1,fltcoff,fltres,fltlo,fltband,flthi,fltext,
atk1,dec1,sus1,rel1,vol,rate,pr_speed,hashid_noclock,count
```

The trigger CSV must expose `hashid` and `clock`. If a `voice` column exists it
is preserved; otherwise the runtime defaults to voice 0.

- `tools/sid2serial` now understands `-Z <file>`: it will run `vsid -sounddev
  dump`, invoke `reg2ssf` on the resulting dump, and finally call
  `beta99_pack.py` to produce the requested beta99 bundle. Use `-R` to override
  the `reg2ssf` executable and `-P` to point at a custom packer script.

Example:

```sh
tools/build/sid2serial -i Bromance-Intro.sid \
    -Z Bromance-Intro.b99.json \
    -R /usr/local/bin/reg2ssf \
    -P tools/beta99_pack.py
```

This leaves `Bromance-Intro.dump`, `Bromance-Intro.ssf.zst`,
`Bromance-Intro.log.zst`, and the consolidated `Bromance-Intro.b99.json`.

## Next Steps

- Finalize the binary `*.b99` layout (compression, per-voice metadata).
- Add a Pico beta99 loader and scheduler.
- Extend `beta99_pack.py` to emit the binary format once the firmware reader is
  ready.

This document will evolve as the beta99 implementation matures.
