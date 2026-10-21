# Documentation Guidelines

[← Repository README](../README.md)

This document defines how SWAS documentation is written and maintained.

## Goals

Documentation should allow:

- A manager or reviewer to understand what each part of the system does.
- A new developer to understand how the system works and where to make changes.
- An operator to build, launch, monitor, and recover the system safely.
- A maintainer to keep documentation aligned with the code.

## File organization

- The repository root `README.md` is the entry point.
- System-level documentation lives in `docs/`.
- Each ROS 2 package has its own `README.md`.
- Large packages may add additional Markdown files under a package-local `docs/` directory.
- Do not document generated files in `build/`, `install/`, `log/`, or `__pycache__/`.
- Do not document backup files, duplicate historical folders, or obsolete experiments as active architecture.

## Required structure

Every package README should contain:

1. Title and breadcrumb navigation.
2. Purpose.
3. Short summary.
4. Architecture.
5. Nodes and modules.
6. Interfaces.
7. Configuration.
8. Launch files.
9. Implementation notes.
10. Dependencies.
11. Testing.
12. Troubleshooting.
13. Related documentation.

## Navigation

- Use relative Markdown links only.
- Every package README must link back to the repository root README.
- Every package README must link to the system architecture document.
- Every node or module section should link directly to its source file.
- Every launch-file entry should link to the launch file, relevant configuration, and relevant nodes.
- Every message documented in `swas_msgs` should link to its `.msg` file.

Example breadcrumb:

```markdown
[← Repository README](../../../README.md) · [System architecture](../../../docs/SYSTEM_ARCHITECTURE.md)
```

## Paths

Use relative paths for all Markdown links. Never use absolute local paths such as `/home/<user>/...` in documentation.

| Path type | Convention |
|---|---|
| Documentation links | Relative repository paths, e.g. `../../../README.md` |
| Package assets | Paths relative to the installed package share directory, e.g. `calib/file.npz` |
| Hardware devices | Stable `/dev/serial/by-id/...` or `/dev/v4l/by-id/...` paths |
| Deployment-specific settings | Launch arguments or environment variables, not hard-coded user paths |

Runtime code and launch files must not contain user-specific absolute paths. Package resources must be resolved with `get_package_share_directory()`. Hardware devices should use stable device identifiers where available.

## Writing style

- Write in present tense.
- Be direct and factual.
- Explain what a component does before explaining how it is implemented.
- Use tables for nodes, topics, parameters, messages, launch files, and hardware mappings.
- Use diagrams for architecture, workflows, coordinate frames, and state machines.
- Link to source files instead of copying large code blocks.
- State units, coordinate frames, assumptions, and limitations explicitly.
- Use `TODO` for information that is not yet verified.
- Do not include personal development history, internship context, motivational text, questions, or speculative claims.
- Do not describe obsolete code as active unless it is explicitly labeled as legacy or reference material.

## Simulation and hardware

Clearly state whether a node, launch file, or workflow applies to simulation, hardware, or both.

- Use **simulation only** for functionality that has not been validated on hardware.
- Use **hardware and simulation** only when both paths are implemented and tested.
- Describe simulation-specific assumptions, such as ideal cameras, zero distortion, simulated lift behavior, and `use_sim_time`.
- Do not imply hardware readiness for a simulation-only feature.

## Updating documentation

- Update the relevant README in the same merge request as the code change.
- If a node, topic, parameter, launch file, or hardware interface changes, update every affected documentation page.
- If behavior is uncertain, mark it as `TODO: verify` rather than guessing.
- Keep the root README short; put detail in the relevant package or `docs/` page.

## Terminology

| Term | Meaning |
|---|---|
| **SWAS** | RU Technologies automated EV battery-swap system |
| **BU** | Battery Unit; removable battery pack |
| **BRU** | Battery Receiving Unit; stationary unit that receives, locks, and releases a BU |
| **BXU** | Battery Exchange Unit; autonomous robot that carries and transfers a BU |
| **Front approach** | Initial centering and rough alignment of the BXU beneath the BRU |
| **Docking** | Final precise alignment between BXU and BRU before battery transfer |
| **HMI** | Operator-facing human-machine interface |