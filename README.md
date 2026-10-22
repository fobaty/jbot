<div align="center">

# JBOT-WheeledBiped

**An autonomous wheeled-biped balancing robot built in Rust.**

Dynamically stable locomotion, real-time motor control, LiDAR SLAM, and autonomous navigation — from the first simulation model to a field-ready platform.

[![Rust](https://img.shields.io/badge/Rust-2021-000000?logo=rust&logoColor=white)](https://www.rust-lang.org/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205%20%2B%20STM32H7-c51a4a)](docs/architecture.md)
[![Status](https://img.shields.io/badge/Status-Phase%201%20%C2%B7%20Design-blue)](docs/roadmap.md)
[![Docs](https://img.shields.io/badge/Docs-SRS%20%C2%B7%20Architecture%20%C2%B7%20Roadmap-informational)](docs/)
[![Unsafe](https://img.shields.io/badge/unsafe-forbidden-success)](docs/skills/rust_robotics_guidelines.md)

</div>

---

## Overview

**JBOT-WheeledBiped** is a research-grade mobile robot that combines wheeled drive with active leg joints to balance dynamically on two wheels. The project covers the full engineering cycle: control theory and simulation, power electronics, embedded firmware, onboard autonomy, and system validation.

The design follows a strict split between a **hard real-time MCU** and a **Linux-class companion computer**. Balance and motor loops never depend on Linux scheduling; perception, mapping, planning, and networking live on the Raspberry Pi 5.

| | |
|---|---|
| **Balance loop** | 1 kHz on MCU, jitter < 50 µs |
| **FOC current loop** | 20–40 kHz |
| **Command timeout → safe stop** | 100 ms |
| **State estimation** | 100–200 Hz EKF |
| **Autonomy** | LiDAR SLAM, costmaps, global/local planning |
| **Runtime** | 20–45 min on Milwaukee M18 XC5.0 |

---

## Key Features

- **Deterministic real-time core** — 1 kHz balancing, 20–40 kHz FOC, hardware watchdog, and a fail-safe stop path enforced below the OS.
- **Safety-first architecture** — dual-channel hardware E-stop, battery protection, precharge, and motor-power contactor independent of software.
- **Rust workspace with one-way dependencies** — perception and control crates are pure computation with no I/O and are fully unit-testable.
- **Onboard autonomy** — LiDAR SLAM without external positioning infrastructure, waypoint missions, patrolling, and teleoperation.
- **Modular by construction** — camera, manipulator, AI accelerator, and docking station extend the platform without redesigning the control path.
- **Simulation before hardware** — every control loop is validated in a simulator before power reaches the motors.

---

## System Architecture

```mermaid
flowchart TB
    subgraph Cloud["Operator / Cloud"]
        SSH["SSH · VPN · Zenoh · ROS 2 tooling"]
    end

    subgraph RPi["Raspberry Pi 5 — Linux / aarch64"]
        PER["Perception"]
        EST["State estimation (EKF)"]
        SLAM["Localization / SLAM"]
        NAV["Navigation / planner"]
        BEH["Behavior / mission"]
        SUP["Supervisor"]
    end

    subgraph MCU["Real-time MCU — STM32H7 / Teensy 4.1"]
        BAL["Balance control 1 kHz"]
        FOC["FOC 20–40 kHz"]
        IMU["IMU sampling 1–4 kHz"]
        WD["Watchdog"]
    end

    subgraph HW["Power & actuation"]
        MOT["Motor drivers"]
        SEN["Encoders · IMU · current · temperature"]
        PWR["M18 battery · protection · DC-DC · E-stop"]
    end

    SSH <-->|"Ethernet / Wi-Fi"| RPi
    PER --> EST --> NAV
    SLAM --> NAV --> BEH
    BEH -->|"CAN FD (primary)\nUART (fallback)"| MCU
    SUP -.-> BEH
    IMU --> BAL --> FOC
    FOC --> MOT --> SEN --> MCU
    PWR --> MOT
```

Full rationale, timing budgets, and the safety concept are in [`docs/architecture.md`](docs/architecture.md).

---

## Workspace Layout

```text
jbot/
├── Cargo.toml                  # Workspace definition and release profile
├── crates/
│   ├── jbot_common/            # Shared messages, state types, kinematic models
│   ├── jbot_hardware/          # HAL: CAN/serial transport, sensors, actuators
│   ├── jbot_estimation/        # EKF / attitude estimation (no I/O)
│   ├── jbot_control/           # Balance control and trajectory generation (no I/O)
│   ├── jbot_navigation/        # Costmaps, planners, navigation stack
│   └── jbot_node/              # High-level executable for the Raspberry Pi 5
└── docs/
    ├── specification.md        # System Requirements Specification (SRS-001)
    ├── architecture.md         # System architecture and safety concept
    ├── roadmap.md              # Phased delivery plan and gate criteria
    ├── system_requirements.md  # High-level requirements
    ├── agent_workflow.md       # Engineering workflow
    ├── tasks.md                # Task breakdown
    └── skills/                 # Rust robotics engineering guidelines
```

> **Dependency rule:** `jbot_estimation` and `jbot_control` stay free of I/O. Everything flows inward toward `jbot_common`; the node wires the pieces together.

---

## Quick Start

### Prerequisites

- [Rust toolchain](https://rustup.rs/) 1.75+ (edition 2021)
- A C linker (`cc` / `clang`) for the host build
- Optional for embedded work: `probe-rs`, `cargo-embed`, and the target toolchain

### Build and test

```bash
git clone git@github.com:fobaty/jbot.git
cd jbot

cargo build --workspace
cargo run -p jbot_node
cargo test --workspace
```

### Quality gates

Run before every merge:

```bash
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
```

### Release build

```bash
cargo build --release -p jbot_node
```

The release profile enables LTO, single codegen unit, and `panic = "abort"` for predictable footprint and latency.

---

## Roadmap

| Phase | Focus | Gate |
|:-----:|-------|:----:|
| **1** | Simulation, WIP model, LQR, stability analysis | G1 |
| **2** | Power subsystem, FOC test bench, MCU protocol | G2 |
| **3** | Raspberry Pi 5, LiDAR, telemetry, EKF, HIL | G3 |
| **4** | SLAM, localization, navigation, 100 autonomous runs | G4 |
| **5** | Camera/AI, manipulator, autonomous docking | G5 |

Detailed tasks, gate criteria, and the critical path live in [`docs/roadmap.md`](docs/roadmap.md).

---

## Safety

This robot stores enough energy to injure people and damage property. The safety concept is layered:

1. **Hardware plane** — fuses, precharge, contactor, and a dual-channel E-stop that cuts motor power without software involvement.
2. **Real-time plane** — watchdog and command timeouts force a damped, low-energy state within 100 ms of link loss.
3. **Supervisor plane** — monitors battery, temperature, latency, and fault flags; requests an orderly stop before limits are exceeded.

> Never disable a safety layer for convenience. Changes to firmware, transport, or control loops require a review against [`docs/skills/rust_robotics_guidelines.md`](docs/skills/rust_robotics_guidelines.md).

---

## Documentation

| Document | Contents |
|---|---|
| [`docs/specification.md`](docs/specification.md) | Requirements, priorities (P1/P2/P3), traceability |
| [`docs/architecture.md`](docs/architecture.md) | Compute tiers, interfaces, power, safety concept |
| [`docs/system_requirements.md`](docs/system_requirements.md) | Hardware and software requirements |
| [`docs/roadmap.md`](docs/roadmap.md) | Phases, gates, schedule, critical path |
| [`docs/tasks.md`](docs/tasks.md) | Granular task breakdown |
| [`docs/agent_workflow.md`](docs/agent_workflow.md) | Contribution and review workflow |

---

## Contributing

1. Branch from `main` using a prefixed name: `feature/`, `fix/`, `refactor/`, `hotfix/`, or `chore/`.
2. Keep changes scoped and covered by tests.
3. Run the full quality gate suite above.
4. Write all code comments, commit messages, and documentation in **English**.
5. Open a pull request with context, test evidence, and safety impact.

---

## License

Released under the [MIT License](LICENSE). © 2026 Oleksii Shataliuk.

---

## Status

The workspace skeleton and engineering documentation are in place. The project is in **Phase 1**: mathematical modeling and balance-controller synthesis. Expect APIs to move until the G1 gate is passed.

---

<div align="center">
<sub>JBOT-WheeledBiped · SRS-001 · Architecture v0.2 · Roadmap RB-001</sub>
</div>
