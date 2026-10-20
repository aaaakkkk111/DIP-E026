# RA3.3-DIRECT verification, 2026-10-20

- ARMCC5.06u7 build960, O1, full rebuild 0 errors / 2 existing filter.c warnings.
- Code=51788, RO=6184, RW=1056, ZI=16600 bytes.
- Map uncompressed load size=0xE694; compressed actual HEX range=0x08000000..0x0800E33F (58176 bytes), 7360 bytes below64KB. RW load starts0x0800E274, compressed size204 bytes.
- RAM execution region=0x44F8; stack=4096, conservative enabled-path estimate1660 bytes (estimate is not hardware measurement).
- Actual HEX checksum, unique data addresses, load range checked.
- Real-C native continuity, partial rings, presets, LLM transactions, rollback, parser and memory guards PASS.
- All3 JS test files PASS, including2 accepted Mock full cycles and old fall-log replay.
- Actual hardware, Bluefy/iOS layout and external LLM service unverified in this revision.

## Release SHA256

- index.html: `80b05fcf10e56db65bc41f61f9678f4fb10aaa80d5a1c961a5a3c48ac75afd81`
- routea_remote_adaptive_bluefy.html: `80b05fcf10e56db65bc41f61f9678f4fb10aaa80d5a1c961a5a3c48ac75afd81`
- stm32_Balance_Car_L.hex: `7fb8e8960fb38d50cbc70cec4134393257d894070a7f999e4da356ccac730e77`
