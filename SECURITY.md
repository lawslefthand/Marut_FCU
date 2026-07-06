# Safety Policy

Defects in code can translate directly into unsafe vehicle behavior.

## Scope

Security-relevant and safety-relevant defects include:

- unintended arming or disarming behavior
- RTOS race conditions affecting the control path
- memory corruption in telemetry or parser code
- timer, interrupt, or clock defects that degrade control-loop timing
- any issue that could compromise safe operation of a vehicle running this firmware

## Reporting

Do not report safety-critical vulnerabilities through a public issue first. Use a private maintainer contact once one is configured for the public project.

A good report should include:

- affected target
- MCU or board used
- reproduction steps or code path
- expected impact on vehicle behavior
- any mitigation or fix direction already identified

## Current Repository Gap

The repository documentation still contains placeholder contact guidance inherited from draft policy documents. That should be replaced before public release.
