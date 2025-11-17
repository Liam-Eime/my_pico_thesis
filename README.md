# Optical Wingbeat Detection (RP2040 Pico)

Firmware and logging for a proof-of-concept optical wingbeat detector for Bactrocera fruit fly (tryoni and jarvisi).
This repo is found at: https://github.com/Liam-Eime/my_pico_thesis

## Overview

This project drives an IR emitter at 20 kHz and samples a photodiode/TIA output at 100 kHz on a Raspberry Pi Pico (RP2040). A synchronous demodulator extracts the envelope of intensity changes as an insect passes through the beam. Triggered events are buffered in RAM and written to microSD as CSV for post‑processing (time series and FFT). The goal is a low-cost sensing pipeline to detect fly passages and explore wingbeat‑band energy.

High‑level flow:

ADC (100 kHz) → per‑period demod (avg ON − avg OFF) → envelope LPF → M‑period averaging → trigger → CSV to SD

## Hardware

- MCU: Raspberry Pi Pico (RP2040)
- Emitter: IR LED driven by PWM at 20 kHz, 25% duty
- Receiver: Photodiode + transimpedance amplifier (TIA)
- Sampling: RP2040 ADC via repeating timer (100 kHz)
- Storage: microSD over SPI (FatFs)
- Power: 3.3 V with adequate decoupling near the analog front‑end

Pin mapping (see `include/board_pins.h`):

- SD card (SPI0): SCK=GPIO 2, MOSI=GPIO 3, MISO=GPIO 4, CS=GPIO 5 (Mode 0, ~12 MHz)
- ADC input: GPIO 26 (ADC0) by default
- Emitter PWM: GPIO 15

## Firmware architecture

Core modules (glanceable):

| Module | Role |
| --- | --- |
| `src/adc_sampler.cpp`, `include/adc_sampler.h` | Repeating‑timer ADC at 100 kHz into ping‑pong buffers; coherent demodulation helper (`demodulate_to_envelope`). |
| `src/emitter_pwm.cpp`, `include/emitter_pwm.h` | Hardware PWM at 20 kHz, 25% duty to drive the IR LED. |
| `src/event_logger.cpp`, `include/event_logger.h` | Triggered capture on the envelope with pre/post windows; buffers full event in RAM and writes a single CSV at the end. |
| `src/sd_helpers.cpp`, `include/sd_helpers.h` | Thin wrappers for FatFs mount/open and close/unmount. |
| `include/board_pins.h` | Pin map and SPI configuration for the SD card and IOs. |
| `src/main.cpp` | Wires everything together: init, demodulate, average, and log. |

Notes:

- Sampling uses a repeating timer (not DMA). This is sufficient at 100 kHz for the current prototype.
- Demodulation is implemented alongside the sampler in `adc_sampler.cpp` to keep the data path tight.

## Signal processing pipeline

Carrier: 20 kHz, duty 25%. For each carrier period:

1) Per‑period demod: Compute avg(ON) − avg(OFF) using the known duty and tracked phase across buffers. This yields one envelope sample per period (≈ 20 kSa/s before further averaging).
2) Envelope LPF: Optional first‑order IIR at the envelope rate, with cutoff set in firmware (default 1.5 kHz).
3) M‑period averaging: Average blocks of M consecutive envelope samples to improve SNR. Default M=4 → effective envelope Fs ≈ 5 kSa/s.

The demodulator operates on raw ADC counts; conversion to volts is applied when logging.

Key parameters (defaults in `src/main.cpp`):

- ADC sample rate: 100 kHz
- Carrier: 20 kHz, 25% duty (`emitter_pwm_start_20k_25()`)
- Envelope LPF cutoff: 1.5 kHz (applied at envelope rate)
- M‑period averaging: 4 (effective envelope ≈ 5 kHz)
- Trigger threshold: 0.2 V (event when envelope drops ≤ threshold)
- Pre/post windows: 0.5 s before and after the trigger

## Event logging and data format

When the envelope crosses below the threshold, the logger captures a pre/post window around the trigger. To avoid SD‑induced noise, it buffers the entire event in RAM and writes once at the end.

File naming: `event_XXXX.csv` (zero‑padded index)

CSV format:

```csv
n,envelope_V
0,0.325000
1,0.323750
...
```

- `n` is the envelope sample index (monotonic across the session)
- `envelope_V` is the demodulated envelope converted to volts using `v_ref` (3.3 V by default)
- Envelope sample rate is `20 kHz / M` (default ≈ 5 kSa/s)

## Build and flash

Prerequisites:

- CMake ≥ 3.13
- Windows build tools (e.g., Visual Studio Build Tools with "NMake Makefiles")
- Pico SDK (fetched via `pico_sdk_import.cmake` if not present)
- Optional: VS Code with CMake Tools

Quick build (PowerShell):

```powershell
cmake -S . -B build -G "NMake Makefiles"
cmake --build build
```

Artifacts: `build/my_pico_thesis_project.uf2` and related outputs.

Flash the Pico:

1) Hold BOOTSEL and plug in the Pico via USB.
2) Copy the UF2 to the RPI‑RP2 drive.

Using VS Code tasks (recommended):

- Run "Configure CMake (Pico)" to generate NMake files
- Run "Build (Pico)" to compile and produce the UF2

## Repository layout

- `src/` — application sources (ADC, demod, PWM, logging, SD helpers)
- `include/` — public headers
- `external/no_os_fatfs/` — FatFs + SD drivers (vendored)
- `pico_sdk_import.cmake` — Pico SDK bootstrap helper
- `build/` — generated artifacts (ignored by Git)

## Tuning at a glance

You can adjust the following without structural changes:

- Envelope LPF cutoff (Hz): `envelope_fc_hz` in `main.cpp`
- Averaging depth M: `env_avg_M` in `main.cpp`
- Threshold (V): `EventLogger::set_threshold()` or `init(...)`
- Pre/post window (s): `EventLogger::set_pre_post()`
- SPI clock and pins: `include/board_pins.h`

## License and attribution

This firmware is for academic research. The SD/FatFs library under `external/no_os_fatfs` retains its own license; please consult its README and LICENSE files.
