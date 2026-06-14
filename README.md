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

## Repository Layout

```text
.
|-- firmware/
|   |-- core/
|   |-- nucleo/
|   |-- nf446re/
|   |-- modes/
|   |   |-- rate/
|   |   `-- stabilize/
|   `-- debug/
|-- pcb/
|   |-- marut_ground_fill/
|   |-- schematics/
|   |-- gerbers/
|   `-- bom/
|-- composite/
|   |-- cad/
|   |-- layup_specs/
|   `-- simulations/
|-- docs/
|-- misc/
`-- .metadata/
```

| Path | Purpose |
|---|---|
| `firmware/` | Embedded firmware targets, board variants, and isolated mode-validation projects |
| `pcb/` | PCB project assets, board workspace files, and fabrication-oriented folders |
| `composite/` | Structural, CAD, layup, and simulation domain scaffolding |
| `docs/` | Architecture, setup, testing, telemetry, release, and domain documentation |
| `misc/` | Holding area for files that do not yet belong to a stable domain |
| `.metadata/` | STM32CubeIDE workspace metadata intentionally preserved in-repo |

---

## Firmware Architecture

### Hardware Stack

| Layer | Component | Role |
|---|---|---|
| HAL | STM32 HAL | Peripheral access |
| RTOS | FreeRTOS (CMSIS-RTOS v2) | Task scheduling |
| IMU | MPU6050 | Accel/gyro + Kalman estimation |
| Barometer | BMP280 | Altitude |
| Magnetometer | QMC5883 | Heading |
| GPS | NMEA parser | Fix, position |
| Telemetry | MAVLink | GCS link |
| RC Input | PPM via timer capture | Pilot control input |
| Motor/Servo | PWM via timer channels | Actuation |

### Firmware Targets

#### `firmware/core`

- **Directory:** [`firmware/core/`](firmware/core/)
- **MCU:** `STM32F411C(C-E)Ux`
- **Active peripherals from `.ioc`:** `ADC1`, `I2C1` in standard mode, `USART1`, `USART2`, `USART6`, `TIM2`, `TIM3`, `TIM4`, `TIM9`, `TIM10`
- **RTOS tasks defined in the target:**

  | Task | Role |
  |---|---|
  | `ArmDisarm` | arm/disarm state management |
  | `ModeHandler` | flight-mode switching and task handoff |
  | `Debounce_Handle` | mode-input debouncing |
  | `QuadTask` | quadcopter control loop |
  | `TelemetryTask` | MAVLink telemetry emission |
  | `FixedWingTask` | fixed-wing control path compiled into source |
  | `VtolMode` | VTOL control path compiled into source |


- **Status:** Active

This is the primary integrated FCU target and the closest thing to the canonical custom-board firmware in the repository. It carries the broadest STM32F411 integration surface, including sensor drivers, RC capture, PWM output, and MAVLink telemetry. The fixed-wing and VTOL handlers exist in the source and can be spawned by mode logic, but they are not created at startup by default and should not be treated as validated production paths.
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

## Documentation Index

| Document | Contents |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | System-level design overview |
| [`docs/DEVELOPMENT_SETUP.md`](docs/DEVELOPMENT_SETUP.md) | Toolchain, build, and flash instructions |
| [`docs/CODING_STANDARDS.md`](docs/CODING_STANDARDS.md) | Naming, formatting, and embedded workflow conventions |
| [`docs/TESTING.md`](docs/TESTING.md) | Bench test requirements and PR evidence expectations |
| [`docs/TELEMETRY.md`](docs/TELEMETRY.md) | MAVLink message set, units, timing notes, and target differences |
| [`docs/RELEASE_PROCESS.md`](docs/RELEASE_PROCESS.md) | Versioning, branch model, and release tagging |
| [`CONTRIBUTING.md`](CONTRIBUTING.md) | Change hygiene, PR expectations, and commit format |
| [`CHANGELOG.md`](CHANGELOG.md) | Version history and current release state |
| [`SECURITY.md`](SECURITY.md) | Vulnerability reporting policy |
| [`SUPPORT.md`](SUPPORT.md) | Support channels and bug-report expectations |
| [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md) | Contributor behavior and enforcement expectations |
| [`docs/firmware/DOCUMENTATION.md`](docs/firmware/DOCUMENTATION.md) | Firmware-domain documentation placeholder |
| [`docs/firmware/PROGRESS.md`](docs/firmware/PROGRESS.md) | Firmware-domain progress placeholder |
| [`docs/firmware/TASKS.md`](docs/firmware/TASKS.md) | Firmware-domain task placeholder |
| [`docs/pcb/DOCUMENTATION.md`](docs/pcb/DOCUMENTATION.md) | PCB-domain documentation placeholder |
| [`docs/pcb/PROGRESS.md`](docs/pcb/PROGRESS.md) | PCB-domain progress placeholder |
| [`docs/pcb/TASKS.md`](docs/pcb/TASKS.md) | PCB-domain task placeholder |
| [`docs/composite/DOCUMENTATION.md`](docs/composite/DOCUMENTATION.md) | Composite-domain documentation placeholder |
| [`docs/composite/PROGRESS.md`](docs/composite/PROGRESS.md) | Composite-domain progress placeholder |
| [`docs/composite/TASKS.md`](docs/composite/TASKS.md) | Composite-domain task placeholder |
| [`docs/frontend/DOCUMENTATION.md`](docs/frontend/DOCUMENTATION.md) | Frontend-domain documentation placeholder |
| [`docs/frontend/PROGRESS.md`](docs/frontend/PROGRESS.md) | Frontend-domain progress placeholder |
| [`docs/frontend/TASKS.md`](docs/frontend/TASKS.md) | Frontend-domain task placeholder |

---

## License

Marut FCU is released under the [MIT License](LICENSE).
