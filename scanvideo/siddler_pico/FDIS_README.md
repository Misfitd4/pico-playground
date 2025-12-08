# FDIS Frame Format

`siddler_pico` consumes a byte stream where every logical unit starts with `sid_header_t` (declared in `siddler_pico.c`). Fields are little-endian:

- `magic`: must be `0x53494446` – ASCII “FDIS”. Any byte sequence without this tag is ignored byte-by-byte until synchronization is restored.
- `count`: number of events in this frame, or `0xFFFF` to flag a command packet. Values above 8192 are rejected to cap load.
- `frame`: monotonically increasing host-supplied frame index used for UI hue selection, idle detection, and telemetry.

After the header, the payload is either SID register events or a single command structure.

## SID Event Frames

Each frame carries `count` packed `sid_event_t` records; each is 6 bytes:

| Field  | Size | Description                                |
|--------|------|--------------------------------------------|
| `addr` | 1    | SID register address (0x00–0x18).          |
| `value`| 1    | Data written to that register.             |
| `delta`| 4    | SID clock cycles since the previous event. |

- Hosts may emit any 32-bit `delta` for the first event; the receiver now preserves those values so long rests survive frame boundaries. The firmware still applies its `cycle_carry` compensation, so long gaps can be distributed across frames without losing time.
- Deltas use 32-bit little-endian integers. `siddler_pico` trims per-frame totals to `SIDDLER_MAX_FRAME_CYCLES`; surplus cycles spill into an internal `cycle_carry`, so hosts can emit large gaps without manual clamping.
- Event logging truncates `delta` to 16 bits purely for on-screen display; the audio/render pipeline keeps the full 32-bit value.

## Command Frames

When `count == 0xFFFF`, the payload is a single `sid_command_t`:

| Field    | Size | Description                  |
|----------|------|------------------------------|
| `opcode` | 1    | Command identifier.          |
| `param0` | 1    | First opcode-specific byte.  |
| `param1` | 1    | Second opcode-specific byte. |
| `param2` | 1    | Third opcode-specific byte.  |

Currently only `opcode = 0x01` (`SID_CMD_CYCLE_MODE`) is implemented, which cycles the visual mode. Additional opcodes can be defined; receivers ignore unknown values after consuming the 4-byte struct.

## Streaming & Resynchronization

- The receiver checks every potential header for the `'FDIS'` magic; failed checks advance by one byte, allowing resynchronization after noise or partial reads.
- `have_header` stays set until the entire payload is available; event frames wait for all `sid_event_t` entries, and command frames wait for `sid_command_t`.
- Once `events_remaining` hits zero, `complete_sid_frame()` finalizes statistics (brightness, wave depth, activity timers) and prepares for the next header.
- Disconnects or CDC suspension reset the parser state, event history, and audio queues so the next connection starts cleanly.

## Using FDIS in Other Projects

1. Emit the exact binary layout described above, little-endian, and begin each unit with `'FDIS'`.
2. Keep `count ≤ 8192` for compatibility; smaller batches reduce latency.
3. Increment `frame` once per logical frame so consumers can detect drops and drive UI state.
4. Encode real SID clock deltas; rely on the receiver’s carry logic for long gaps.
5. Inject command packets whenever you need out-of-band control; they coexist with event frames in the same stream.
6. When building another consumer, mirror the parser discipline: validate headers, bound-check `count`, force the first delta to zero, and resync gracefully on malformed data.

This reference should give you a drop-in description for tooling, encoders, or alternative visualizers that speak the FDIS stream.
