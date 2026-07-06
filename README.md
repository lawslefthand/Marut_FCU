# Marut FCU

Open-source tri-mode flight control firmware from India, built to unify quadcopter, fixed-wing, and VTOL development on STM32F4 hardware.

![License](https://img.shields.io/badge/license-MIT-blue) ![Platform](https://img.shields.io/badge/platform-STM32F4-lightgrey) ![Build](https://img.shields.io/badge/build-STM32CubeIDE-green) ![Version](https://img.shields.io/github/v/tag/TheGoodDevil00/Marut_FCU?label=release) ![Status](https://img.shields.io/badge/status-active_development-yellow)

Marut FCU is a student-built open-source flight control unit developed with the explicit goal of supporting quadcopter, fixed-wing, and VTOL platforms under one repository and one evolving firmware architecture. The project is built around STM32F4 targets, STM32CubeIDE, CMSIS-RTOS v2 on FreeRTOS, and a handwritten control and telemetry stack that remains readable enough for contributors to extend across avionics, firmware, and hardware domains. What makes it distinctive is not only the tri-mode ambition, but the decision to keep the hardware, firmware structure, build artifacts, and engineering workflow visible in-repo rather than hiding them behind opaque tooling. The repository already contains active integrated FCU targets, isolated mode-validation targets, and open PCB/composite scaffolding, while some fixed-wing and VTOL paths remain explicitly marked as pending hardware validation. It should be read as real embedded systems work under active development, not as a finished flight-certified stack.

---

## Feature Highlights

- **Tri-mode flight architecture:** The repository is structured around a single program direction that covers quadcopter, fixed-wing, and VTOL control paths, with the F446 multi-mode targets carrying the broadest implementation surface today.
- **Full sensor stack:** The active targets integrate the expected FCU sensor layers for IMU, barometric altitude, magnetometer heading, and GPS parsing through target-specific modules.
- **MAVLink telemetry contract:** The codebase implements MAVLink helpers for core GCS-facing telemetry including `HEARTBEAT`, `ATTITUDE`, `GPS_RAW_INT`, `GLOBAL_POSITION_INT`, and `BATTERY_STATUS`.
- **FreeRTOS task model:** Firmware targets use CMSIS-RTOS v2 task definitions generated from CubeMX and implemented in handwritten `main.c` task bodies.
- **Open hardware domains:** PCB assets and composite-domain scaffolding are versioned in the same repository so avionics, board, and vehicle contributors can work from the same project root.
- **Tracked release artifacts:** `Debug/` outputs, including `.elf` and related build files, are intentionally kept in version control as part of the repository workflow.

---

## Getting Started

1. Read [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).
2. Follow [`docs/DEVELOPMENT_SETUP.md`](docs/DEVELOPMENT_SETUP.md) to install STM32CubeIDE, import a target, and build it correctly.
3. Open the `.ioc` file for the target you want to change and review pin assignments, timer ownership, UART routing, and RTOS declarations before touching source.
4. Read that target's `main.c` top-to-bottom before making behavioral changes.
5. Review [`CONTRIBUTING.md`](CONTRIBUTING.md) for repository hygiene, tracked build artifacts, and target-boundary expectations.
6. Review [`docs/CODING_STANDARDS.md`](docs/CODING_STANDARDS.md) and [`docs/TESTING.md`](docs/TESTING.md) before opening a pull request.

---
## Hardware

The board for `firmware/core` is the custom Marut FCU flight controller based on the `STM32F411CEUx` in the `UFQFPN48` package. The `firmware/nucleo` target uses the `NUCLEO-F446RE` as the primary bring-up and benchtop development platform. PCB assets live under [`pcb/`](pcb/), with the currently populated board workspace centered on [`pcb/marut_ground_fill/`](pcb/marut_ground_fill/); the fabrication-oriented folders [`pcb/schematics/`](pcb/schematics/), [`pcb/gerbers/`](pcb/gerbers/), and [`pcb/bom/`](pcb/bom/) are present and ready to receive design exports. The flight-computer BOM belongs in [`pcb/bom/`](pcb/bom/). Composite and structural assets are scaffolded under [`composite/`](composite/), specifically in [`composite/cad/`](composite/cad/), [`composite/layup_specs/`](composite/layup_specs/), and [`composite/simulations/`](composite/simulations/), but those domains are still at an early repository stage.

---

## Telemetry

Marut FCU uses MAVLink over UART for the ground-station link. The active repository telemetry contract includes `HEARTBEAT`, `ATTITUDE`, `GPS_RAW_INT`, `GLOBAL_POSITION_INT`, and `BATTERY_STATUS`, while some targets also emit pressure and status-text messages during their current bench loops. Mission Planner and QGroundControl are the intended ground-control tools for validation and operator-facing telemetry work. The full message contract, target-specific notes, current IDs, and timing caveats live in [`docs/TELEMETRY.md`](docs/TELEMETRY.md).

---

## Contributing

Read [`CONTRIBUTING.md`](CONTRIBUTING.md) before changing any firmware, hardware, or tracked build outputs. Use [`docs/DEVELOPMENT_SETUP.md`](docs/DEVELOPMENT_SETUP.md) to get the toolchain and target import flow right before your first build. Community standards and contributor conduct live in [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md). Questions, issue-routing guidance, and support expectations live in [`SUPPORT.md`](SUPPORT.md).

---


## License

Marut FCU is released under the [MIT License](LICENSE).
