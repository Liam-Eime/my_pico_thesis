# EG4011/12 Thesis Software — RP2040 Pico

This repository contains the source code and build configuration for my EG4011/12 thesis project targeting the Raspberry Pi Pico (RP2040).

## Layout

- `src/` — Application sources (C/C++)
- `CMakeLists.txt` — Pico SDK CMake project
- `pico_sdk_import.cmake` — Helper to locate or fetch the Pico SDK
- `build/` — Generated build artifacts (ignored by Git)
- `external/` — Optional third-party components

## Prerequisites

- CMake 3.13+
- A Windows build toolchain (e.g. Visual Studio Build Tools with "NMake Makefiles")
- Raspberry Pi Pico SDK available at `../pico-sdk` relative to this project, or let the import helper fetch it
- Optional: VS Code with CMake Tools and C/C++ extensions

## Build (from PowerShell)

```powershell
cmake -S . -B build -G "NMake Makefiles"
cmake --build build
```

Artifacts (including a `.uf2`) will be in the `build/` folder.

## Flashing

1. Hold the BOOTSEL button on the Pico and plug it into USB.
2. A mass storage device appears (RPI-RP2). Copy the generated `.uf2` onto it.

## Notes

- This repo is a minimal starting point for the thesis software.
