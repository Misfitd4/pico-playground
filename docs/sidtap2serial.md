# sidtap2serial Reference

This document explains how the `sidtap2serial` helper packaged with this
repository consumes SIDTap event streams and forwards them over a USB CDC
serial connection. Refer to it when adapting tooling or debugging the pipe
between a host SID capture source and the Pico-based SID emulator.

## Overview

`tools/sidtap2serial` is a small POSIX command line program that reads a
byte stream from a FIFO (default `/tmp/sid.tap`) and writes the decoded
events to a serial device (typically the `tty.usbmodem*` exposed by
`picoSid-synth`). It keeps the serial line open, optionally logs frames,
and copes with temporary disconnects through non-blocking I/O and retry
loops.

Typical usage:

```sh
mkfifo /tmp/sid.tap            # once per system
tools/sidtap2serial -f /dev/tty.usbmodem1234
```

Important command line switches:

| Flag | Description |
| ---- | ----------- |
| `-i <fifo>` | Path to the input FIFO (defaults to `/tmp/sid.tap`). |
| `-f <dev>` | Serial device to forward to (e.g. `/dev/tty.usbmodemXXXX`). |
| `-b <baud>` | POSIX baud rate used for real UARTs (defaults to 2 Mbaud, ignored for USB CDC). |
| `-v`, `-vv` | Verbose logging: one `-v` prints frame summaries, `-vv` logs every event. |
| `-h` | Show usage help. |

When the FIFO cannot be opened the tool will create it if missing and then
block until a writer connects. On `SIGINT`/`SIGTERM` it exits cleanly.

## Data Structures

The SIDTap stream consists of repeating **frame headers** and **event
records** packed in little-endian byte order. All structures are sent
without padding (`#pragma pack(1)` in `sidtap2serial.c`).

### Frame Header (`sid_header_t`)

```c
typedef struct {
    uint32_t magic;   // must equal 0x53494446 ("SIDF")
    uint32_t count;   // number of event records that follow
    uint32_t frame;   // optional frame counter (unused by picoSid-synth)
} sid_header_t;
```

The magic signature (`'S' 'I' 'D' 'F'`) is used by the tool to align with
the stream. If the signature is not seen, bytes are discarded until it is.

### Event Record (`sid_event_t`)

```c
typedef struct {
    uint8_t  chip;    // SID index and optional flags
    uint8_t  addr;    // 5-bit SID register address (0x00-0x1F used)
    uint8_t  value;   // data to write
    uint8_t  pad;     // unused, reserved for alignment/extensions
    uint32_t delta;   // cycles since previous event within the frame
} sid_event_t;
```

- **chip**: lower bits identify the target SID (0 for left, 1 for right).
  Some senders pack additional flags—`sidtap2serial` preserves them but the
  firmware masks down to the lower two bits.
- **addr**: nominally 0–31 (`$00`–`$1F`). The firmware masks to 5 bits to
  ensure safety before calling `sid_engine_queue_event`.
- **value**: raw byte written to the SID register.
- **delta**: number of SID clock cycles elapsed since the previous event.
  The firmware uses it to maintain timing accuracy.

The `pad` byte is currently unused; it simplifies host alignment and offers
space for future flags.

## Behavioural Notes

- **I/O model**: the tool uses non-blocking reads on the FIFO and blocking
  writes on the serial file descriptor. If the serial device temporarily
  deasserts RTS/CTS, the program waits via `select()` and retries.
- **Signal handling**: `SIGINT` and `SIGTERM` set a flag checked in the main
  loop, allowing the tool to close descriptors and exit without corrupting
  the FIFO.
- **Device init**: when opening a real UART (as opposed to USB CDC) the
  tool configures it for raw 8N1 mode at the requested baud rate and asserts
  DTR/RTS if supported.
- **Fault tolerance**: if the FIFO disappears (`ENOENT`) the tool attempts
  to re-create it; short reads or `EINTR` are retried. EOF on the FIFO causes
  the reader to reopen and block until a producer connects again.

## Integration Tips

- The Pico firmware expects the stream exactly as described above. When
  producing SIDTap data from another source (e.g. a VICE SID listener),
  ensure it writes the `sid_header_t` followed by `count` copies of
  `sid_event_t`.
- When debugging timing issues, launch the tool with `-vv` to dump every
  event; compare the logs with the upstream SIDTap producer.
- On macOS and Linux, a named pipe is the simplest transport. If you need
  to inject data from another protocol (e.g. network) you can bridge it
  into the FIFO with `socat` or `netcat`.
- Remember that USB CDC devices appear as `/dev/tty.usbmodem*` (macOS) or
  `/dev/ttyACM*` (Linux). Pass the correct device path with `-f`.

Keep this reference alongside the tool so future work on the SID capture or
playback pipeline has a single source of truth for the expected byte layout
and runtime behaviour.
