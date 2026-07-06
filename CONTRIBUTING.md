# Contributing

Thanks for contributing to Marut_FCU. This repository mixes generated embedded project files, handwritten flight logic, hardware workspace assets, and tracked build outputs, so careful change hygiene matters more than usual.

## General Standards

### 1. Keep changes scoped

- Make the smallest change that solves the problem.
- Avoid opportunistic rewrites in unrelated modules.
- Prefer improving one firmware target at a time unless the change is intentionally cross-target.

### 2. Preserve intent before improving style

- Understand whether a file is generated, hand-maintained, or mixed before editing it.
- Match the existing local style in the file you are touching.
- Do not rename, move, or delete project assets casually in STM32CubeIDE-based directories.

### 3. Document decisions

- Update docs when changing architecture, workflows, pin assignments, telemetry behavior, or task structure.
- If a change affects multiple targets differently, note that explicitly in PR descriptions or domain docs.

### 4. Be conservative with embedded changes

- Assume hardware-facing changes can have real bench impact.
- Call out changes to clocks, timers, interrupt priority, UART routing, ADC scaling, I2C speed, PWM ranges, or control constants.
- When tuning control loops, describe the expected effect on behavior.

## Repository-Specific Standards

### 1. Respect generated STM32 project structure

This repository contains STM32Cube-generated projects with files like:

- `.ioc`
- `.project`
- `.cproject`
- `.mxproject`
- `.settings/`
- linker scripts
- HAL/CMSIS/FreeRTOS generated files

When working in these projects:

- Do not manually "clean up" generated files unless the team wants a full regeneration.
- Be cautious editing generated sections inside files like `main.c`, `stm32f4xx_hal_msp.c`, and interrupt handlers.
- Prefer placing custom logic inside `USER CODE BEGIN` / `USER CODE END` regions where applicable.

### 2. Treat build outputs as intentionally tracked

Unlike many repositories, this one intentionally tracks:

- `Debug/` directories
- `*.elf`
- `*.map`
- `*.o`
- `*.d`
- `*.mcuvproj`

That means:

- Do not add `.gitignore` rules that hide these artifacts.
- Do not mass-delete tracked build outputs as a cleanup step.
- If regeneration changes these files, include them when they are part of the intended update.

### 3. Keep board/target boundaries clear

The repo currently contains several firmware targets:

- `firmware/core`
- `firmware/nucleo`
- `firmware/nf446re`
- `firmware/modes/rate`
- `firmware/modes/stabilize`

When contributing:

- State clearly which target you tested or modified.
- Do not assume a fix in one target applies to all targets.
- If you port a fix across targets, do it deliberately and mention each affected directory.

### 4. Be careful with peripheral changes

Changes to these areas should be called out explicitly:

- timer prescalers and periods
- PWM output mapping
- RC/PPM capture logic
- UART selection and baud rates
- I2C bus speed or bus selection
- ADC scaling for battery or analog telemetry
- RTOS task priorities, stack sizes, and synchronization objects

These changes often alter behavior even when the diff looks small.

### 5. Keep telemetry contracts stable

The repository contains MAVLink and GPS-related code. When editing telemetry:

- note message type changes
- note unit conversions
- note system/component ID changes
- note any packet timing or ordering changes

Ground tools and downstream integrations can break on subtle telemetry changes.

### 6. Keep hardware-domain artifacts organized

For `pcb/` and `composite/`:

- place files in the domain-specific folders introduced by the restructure
- use descriptive names for new schematics, gerbers, BOM exports, CAD assets, and simulation files
- avoid dropping hardware assets at the repo root

### 7. Leave `.metadata/` alone unless there is a specific reason

The restructure intentionally preserved `.metadata/` in place.

- Do not reorganize it as part of normal contribution work.
- If you must update something there, explain why.

## Branches and Commits

### Branch naming

Use short, descriptive branch names. For example:

- `fix/f411-ppm-capture`
- `feature/mavlink-battery-telemetry`
- `docs/readme-contributing`
- `chore/nf446re-project-cleanup`

### Commit messages

Prefer conventional, high-signal commit messages such as:

- `fix: correct ppm channel capture rollover`
- `feat: add global position mavlink message`
- `docs: expand repository onboarding documentation`
- `chore: reorganize pcb exports`

If a commit changes generated project files, mention that in the body.

## Pull Request Expectations

A good PR should say:

- which target(s) changed
- whether the change is generated, handwritten, or both
- what was tested
- what hardware assumptions were involved
- whether tracked build outputs changed intentionally

Example checklist:

- target identified
- hardware/peripheral impact described
- docs updated if behavior changed
- generated files reviewed
- tracked build artifacts included intentionally

## Documentation Expectations

Please update documentation when you:

- add a new board target
- change the repo layout
- modify task architecture
- change telemetry behavior
- add new sensors or buses
- change contributor workflow expectations

At minimum, consider whether updates belong in:

- [`README.md`](/C:/Dev/Marut_FCU/README.md)
- [`docs/ARCHITECTURE.md`](/C:/Dev/Marut_FCU/docs/ARCHITECTURE.md)
- one of the domain-specific files under [`docs/`](/C:/Dev/Marut_FCU/docs)

## What Not To Do

- Do not delete tracked `Debug/` outputs because they look temporary.
- Do not move files between targets unless the change is intentional and documented.
- Do not edit codegen-heavy embedded files without understanding the CubeMX project around them.
- Do not mix documentation-only work with unrelated firmware refactors.
- Do not assume every STM32 target is feature-equivalent.

## When in Doubt

If you are unsure whether a file is generated, whether a tracked artifact should stay versioned, or whether a change belongs in one target or several, pause and narrow the scope first. In this repo, careful incremental work beats broad cleanup every time.
