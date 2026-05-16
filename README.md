UNI/O Protocol Decoder
======================

`UNI/O` decoder for `sigrok` / `PulseView`.

Implemented:

- single-line `SCIO` input
- standby pulse and start-header detection
- preamble (`0x55`) timing lock
- Manchester bit decoding
- header byte decoding
- subsequent data-byte decoding
- `MAK` / `SAK` annotation

Vibecoded with Codex.
