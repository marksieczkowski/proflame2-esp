# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESPHome custom component for controlling ProFlame 2 gas fireplaces via ESP32 + CC1101 433MHz RF module. Uses ESP-IDF framework (not Arduino). The component transmits OOK-modulated Manchester-encoded 91-bit packets at 314.973 MHz (2400 baud) to control the fireplace.

## Build & Workflow Commands

This is an ESPHome component — there is no standalone `make`/`cmake` build. Development uses ESPHome CLI against the example YAML:

```bash
# Compile firmware
esphome compile proflame2_fireplace.yaml

# Flash to device
esphome upload proflame2_fireplace.yaml

# Monitor logs (enable VERBOSE in yaml first)
esphome logs proflame2_fireplace.yaml
```

For verbose debug output, set in the YAML:
```yaml
logger:
  level: VERBOSE
  logs:
    proflame2: VERBOSE
```

Debug buttons are defined (commented out) in `proflame2_fireplace.yaml` and can be uncommented to test CC1101 hardware, verify checksums, and inspect raw packet encoding.

## Architecture

The component has three layers:

**Python config layer** (`components/proflame2/__init__.py`) — ESPHome schema definition, YAML validation, and C++ code generation. Registers all entities: 4 switches (power, pilot, aux, secondary_flame), 3 number sliders (flame 0-6, fan 0-6, light 0-6), and a send button. This is the ESPHome integration glue.

**C++ header** (`components/proflame2/proflame2_cc1101.h`) — Defines CC1101 register constants, strobe commands, `ProFlame2Command` state struct, and the `ProFlame2Component` class interface. `ProFlame2Component` inherits from both `Component` and `spi::SPIDevice`.

**C++ implementation** (`components/proflame2/proflame2_cc1101.cpp`) — Core logic organized into:
- CC1101 hardware driver (SPI register read/write, reset, configure with 26-register config)
- Protocol encoding: `build_packet()` assembles 91-bit packet (7 × 13-bit words), `encode_manchester()` expands to 182 bits, `calculate_checksum()` uses nibble-based XOR with tuned constants (0x05,0x02 for cmd1; 0x04,0x04 for cmd2)
- Non-blocking TX state machine: `transmit_command()` queues 5 repeats, `start_tx_()` primes the 64-byte CC1101 FIFO, `service_tx_()` refills FIFO in `loop()` while monitoring MARCSTATE/TXBYTES registers
- Control setters update `current_state_` (a `ProFlame2Command` struct) and publish state back to HA entities

**Transmission flow:** HA entity change → setter updates `current_state_` + sets `buffer_dirty` → `loop()` calls `transmit_command()` when timing allows (200ms min interval) → packet built + Manchester-encoded → 5 repeats with 2ms gaps sent via FIFO → fireplace decodes and acts.

## Key Protocol Details

- **Frequency:** 314.973 MHz (some fireplaces use 315 MHz or 433 MHz)
- **Modulation:** OOK (On-Off Keying)
- **Baud rate:** 2400
- **Encoding:** Thomas Manchester encoding
- **Packet structure:** 91 bits = 7 words × 13 bits (data + parity)
- **Checksum:** Nibble-based XOR with specific constants per word
- **Serial number:** 4-byte value identifying the remote (default `0x12345678`; paired via fireplace pairing mode)

## ESPHome Component Configuration

Minimum required config in user YAML:
```yaml
external_components:
  - source: github://marksieczkowski/proflame2-esp
    components: [proflame2]

proflame2:
  cs_pin: GPIO21
```

Optional pins/settings: `gdo0_pin`, `serial_number`, and per-entity `name`/`id`/platform attributes for all 8 entities.
