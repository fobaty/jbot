# Orchestrator and subagents: JBOT-WheeledBiped development workflow

This document describes how the main orchestrator agent decomposes work on the robot,
which subagent roles exist, which tools they use, and in what order results are
reviewed and integrated.

Related documents: `docs/system_requirements.md`, `docs/architecture.md`,
`docs/specification.md`, `docs/roadmap.md`, `docs/skills/rust_robotics_guidelines.md`.

---

## 1. Principles

1. **Contract before code.** Types, protocols, and crate boundaries are fixed first,
   then the implementation is written. Shared types live in `jbot_common`.
2. **One owner per crate.** A subagent does not edit other crates without
   agreement with the orchestrator and architect.
3. **Verification at every step.** A task is not considered complete until
   `cargo fmt`, `cargo check`, `cargo test`, and `cargo clippy` pass for the affected crates.
4. **Isolation of computation and I/O.** `jbot_estimation` and `jbot_control` are pure
   math without I/O; transport and drivers live only in `jbot_hardware`.
5. **Safety over speed.** Bypassing the watchdog, removing limits, `unsafe`,
   and changes to the control loop require a separate review.
6. **The orchestrator is the single integration point.** Subagents work in their own
   branches and do not run `git push`, change the workspace, or merge code.

---

## 2. Topology

```
                          ┌───────────────────────────┐
                          │        Orchestrator       │
                          │  plan, tasks, integration │
                          └─────────────┬─────────────┘
                                        │ task packets / handoff
          ┌─────────────────────────────┼───────────────────────────────┐
          │                             │                              │
┌─────────▼─────────┐         ┌─────────▼─────────┐         ┌──────────▼──────────┐
│     Architect     │         │   Rust-Host-Dev   │         │  Rust-Embedded-Dev  │
│   ADR, contracts  │         │   common/hw/est/  │         │      firmware/      │
│  crate boundaries │         │    control/node   │         │   no_std, CAN, FOC  │
└─────────┬─────────┘         └─────────┬─────────┘         └──────────┬──────────┘
                                        │
                             ┌──────────▼──────────┐
                             │    Navigation-Dev   │
                             │   navigation, SLAM  │
                             └──────────┬──────────┘
                                        │
                             ┌──────────▼──────────┐
                             │      QA-Tester      │
                             │   tests, sim, HIL   │
                             └──────────┬──────────┘
                                        │
                             ┌──────────▼──────────┐
                             │   Safety-Reviewer   │
                             │      final gate     │
                             └─────────────────────┘
```

The orchestrator does not duplicate the work of subagents: it plans, removes blockers,
integrates, and runs final checks. The Architect and Safety-Reviewer are
review roles; they do not write production code without an explicit assignment.

---

## 3. Agent roles

### 3.1. Orchestrator

**Responsible for:** decomposing requirements, task assignment, priorities, conflict
resolution, branch integration, final workspace verification, and status reporting.

**Rights:** read the entire repository; write to `docs/`, the workspace `Cargo.toml`,
and integration branches; run any checks; merge after the gate passes.

**Does not:** implement large modules in place of subagents, bypass QA,
or run `git push` and deploy without explicit user permission.

**Input:** requirement, bug, roadmap item, QA result.
**Output:** task packet, integrated and verified code, report, updated docs.

### 3.2. Architect

**Responsible for:** architectural decisions (ADR), crate boundaries, message schemas,
the RPi–MCU protocol, library selection, and impact analysis of changes on safety and
real-time behavior.

**Rights:** write to `docs/architecture.md`, `docs/adr/`, interface stubs in
`jbot_common`/`jbot_protocol`; read all code.

**Output:** an ADR or a brief decision, approved interfaces, a list of affected
crates and risks, and acceptance criteria for subagents.

### 3.3. Rust-Host-Dev

**Responsible for:** the host part on RPi 5 — `jbot_common`, `jbot_hardware`,
`jbot_estimation`, `jbot_control`, and the composition in `jbot_node`; Tokio tasks,
CAN/UART transport, EKF, the balance loop, configuration, and telemetry.

**Rights:** write to the listed crates and their tests; run cargo and
log analysis tools.

**Constraints:** does not change the protocol without the Architect; does not add
dependencies without agreement; does not block the runtime with synchronous I/O; does not
use `unwrap()` in the control loop.

### 3.4. Rust-Embedded-Dev

**Responsible for:** MCU firmware (`firmware/`, the future `jbot_protocol`), `no_std`,
FOC/drives, IMU polling, CAN/UART framing, the watchdog, a deterministic
loop ≥ 1 kHz, and `defmt` logs.

**Rights:** write to `firmware/` and `jbot_protocol`; run cross-build, `probe-rs`,
and read RTT/`defmt`.

**Constraints:** does not disable the watchdog or hardware limits; changes to the control
loop go through HIL and the Safety-Reviewer; work with the real test bench is allowed only with
user permission.

### 3.5. Navigation-Dev

**Responsible for:** `jbot_navigation` — costmap, global/local planner,
path following, SLAM/localization integration, autonomous motion scenarios,
and navigation quality metrics.

**Rights:** write to `jbot_navigation` and its tests; run the simulator and replay.

**Constraints:** does not change the state/command format without the Architect; does not bypass
the staleness check; with stale pose/scan, must reduce speed
or stop.

### 3.6. QA-Tester

**Responsible for:** the test plan, unit/property/integration tests, `vcan` scenarios,
simulation runs, replay regressions, HIL checklists, and reproducible bug reports.

**Rights:** write to test modules and `tests/`, `benches/`; run cargo, the
simulator, and log analysis; read all code.

**Constraints:** does not modify production code to "fit" a test; does not close a
bug without reproduction; does not accept changes without running the gate commands.

### 3.7. Safety-Reviewer

**Responsible for:** review of the state machine, watchdog chains, limits, fault handling,
E-stop, behavior on communication loss, `unsafe` blocks, and panics in the hot path.

**Rights:** read everything; blocking assessment at the final gate; write only to
`docs/` or separate safety reports.

**Output:** approve / request changes with references to lines and mandatory
tests.

---

## 4. Agent tools

### 4.1. Common tools

| Tool | Purpose | Used by |
|---|---|---|
| `read`, `glob`, `grep` | Targeted reading and searching across code and documentation | all |
| `write`, `edit` | Creating and patching files within one's own area | dev roles, Orchestrator |
| `bash` / terminal | Running builds, tests, git, and diagnostics | all except read-only reviews |
| `git status/diff/log/branch/commit` | Local work with branches and diffs | dev roles, Orchestrator |
| `git push`, deploy | Publishing changes | **user only** |

Prohibited: reading secrets and `.env`, committing keys/passwords, deleting other people's
changes, `git push --force`, and working with real hardware without confirmation.

### 4.2. Rust and build

| Tool / command | Purpose |
|---|---|
| `cargo fmt --all -- --check` | Formatting |
| `cargo check -p <crate> --all-targets` | Fast type check |
| `cargo test -p <crate>` | Crate unit/integration tests |
| `cargo clippy -p <crate> --all-targets -- -D warnings` | Linting without warnings |
| `cargo check --workspace --all-targets` | Check of the entire workspace |
| `cargo test --workspace` | Full test run |
| `cargo build --release -p jbot_node` | Release build of the host binary |
| `cargo build --release --target aarch64-unknown-linux-gnu -p jbot_node` | Cross-build for RPi 5 |
| `cargo build --release --target thumbv7em-none-eabihf -p firmware` | Firmware cross-build |
| `cargo deny check`, `cargo audit` | Licenses and dependency vulnerabilities |
| `probe-rs run` / `probe-rs attach` | Flashing and debugging the MCU |
| `defmt-print`, RTT | Reading binary firmware logs |

Cross commands are run only if the corresponding target/toolchain is configured;
otherwise this is recorded as a limitation in the handoff.

### 4.3. Log and telemetry analysis

| Tool | Purpose |
|---|---|
| `tracing` JSON + `jq`/`rg` | Finding errors, timings, and correlation IDs |
| `dmesg`, `journalctl -u jbot-node` | OS, CAN, and USB errors, service crashes |
| `candump`, `cansniffer`, `cangen` on `vcan0` | Debugging frames and timeouts |
| `rerun` | Visualization of state, trajectories, and point clouds |
| `perf`, jitter logs | Hot path profiling |
| blackbox dump | Analysis of the last seconds before a fault |

### 4.4. Simulator and test bench

| Tool | Purpose | Access |
|---|---|---|
| `jbot_sim` (future) | Plant model, sensor noise, balance and navigation scenarios | dev roles, QA |
| Replay of recorded logs | Regression without hardware | QA, Navigation-Dev |
| Gazebo/ROS2 + `zenoh-bridge-ros2dds` | External simulation and RViz | Navigation-Dev, QA |
| `vcan` | Transport integration tests | Host-Dev, QA |
| HIL test bench (MCU + drivers + RPi) | Verification of balance, watchdog, and communication loss | Embedded-Dev, QA |
| Logic analyzer | Bus and signal timing | Embedded-Dev |

Access to HIL and real drives is allowed only with explicit user confirmation;
the E-stop must be within reach.

---

## 5. Interaction order

### Step 0. Task intake

The orchestrator takes a requirement from the specification, roadmap, bug, or user
request, checks the related documents, and forms the task. If the requirement
is ambiguous, it asks a clarifying question before starting work.

### Step 1. Architectural decision

The Architect determines:

- affected crates and the direction of dependencies;
- changes to public types and the protocol;
- impact on real-time behavior, safety, and resources;
- new dependencies (with justification);
- acceptance criteria and risks.

Result: an ADR or a short decision + approved interfaces. Without this,
subagents do not start changing contracts.

### Step 2. Task assignment (task packet)

The orchestrator creates one packet per subagent. Template:

```yaml
id: JBOT-123
goal: "Add a speed limit based on pose freshness"
owner: Navigation-Dev
scope: [crates/jbot_navigation]
depends_on: [JBOT-120]
interfaces:
  input: RobotState, PoseAge
  output: VelocityCommand
acceptance:
  - cargo test -p jbot_navigation
  - sim scenario: stale_pose_stop
constraints:
  - do not change jbot_common without the Architect
  - no new dependencies
handoff: "diff + command results + risks"
```

Packet rules:

- one task — one verifiable result;
- an explicit list of files/crates that may be changed;
- dependencies between tasks are stated explicitly;
- acceptance criteria are formulated as commands or scenarios.

### Step 3. Parallel implementation

Subagents work in separate branches (`feature/`, `fix/`, `refactor/`,
`hotfix/`, `chore/`). Tasks that do not overlap in
files and contracts can be run in parallel. If two agents need the same type, the
Architect first makes the change in `jbot_common`, and then both update.

After making changes, each subagent must run locally:

```bash
cargo fmt --all -- --check
cargo check -p <crate> --all-targets
cargo test -p <crate>
cargo clippy -p <crate> --all-targets -- -D warnings
```

For navigation changes, additionally a simulation scenario; for firmware,
a cross-build and, if available, HIL; for the control loop, unit tests and verification of
jitter/latency.

### Step 4. Handoff

The subagent returns a report to the orchestrator:

```text
status: done | blocked | needs-design
changed: <files and key lines>
commands: <command -> result>
tests: <what is covered, what is not>
risks: <impact on safety/real-time behavior>
open_questions: <if any>
next: <proposed next step>
```

Unfinished or failing work is not masked: a `blocked` status with
reproduction is better than a "green" report without a run.

### Step 5. Integration

The orchestrator:

1. Checks the diff against the scope and architecture.
2. Integrates branches into the integration branch.
3. Resolves conflicts only in its own area; escalates contract conflicts to the
   Architect.
4. Runs a check of the entire workspace:

```bash
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
```

5. If necessary, builds the target artifacts:

```bash
cargo build --release -p jbot_node
cargo build --release --target aarch64-unknown-linux-gnu -p jbot_node
cargo build --release --target thumbv7em-none-eabihf -p firmware
```

6. Verifies that new dependencies and the protocol do not violate constraints.

### Step 6. QA gate

The QA-Tester runs the levels corresponding to the change:

| Level | What is checked | Tool |
|---|---|---|
| Unit | Math, framing, CRC, limits, state machine | `cargo test` |
| Property | Frame round-trip, counter overflow, garbage data | `proptest` |
| Integration | CAN/UART loopback, failover, timeouts | `tokio::test`, `vcan` |
| Simulation | Balance, navigation, sensor staleness | `jbot_sim`, Gazebo |
| HIL | Real MCU, drivers, communication loss, watchdog | test bench, `defmt` |
| Regression | Replay of recorded logs, metric comparison | replay, `rerun` |

A QA bug report contains: the scenario, expected/actual behavior, logs,
the reproduction command, and the affected crate.

### Step 7. Final review and merge

- The Safety-Reviewer checks changes in the control loop, watchdog, fault
  handling, E-stop, and `unsafe`.
- The Architect confirms that contracts and dependencies have not degraded.
- The orchestrator merges only with green checks and no blocking
  comments.
- After the merge, `docs/` is updated and known limitations are recorded.

`git push`, publication, and deployment are performed only at the explicit request
of the user.

---

## 6. Conflict resolution and escalation rules

1. **File conflict:** the crate owner has priority; the second agent waits for
   integration or receives a separate task.
2. **Contract conflict:** the Architect makes the decision, and the change is made
   once in `jbot_common`/`jbot_protocol`.
3. **Failing tests:** QA returns the task to the owner with reproduction.
   After three failed iterations, the orchestrator revises the decomposition.
4. **Safety risk:** work stops until the Safety-Reviewer review.
5. **Insufficient data:** the orchestrator asks the user rather than guessing
   requirements.

---

## 7. Definition of Done

A change is considered complete if all of the following are satisfied simultaneously:

- [ ] `cargo fmt --all -- --check` with no diff;
- [ ] `cargo check --workspace --all-targets` with no errors;
- [ ] `cargo test --workspace` fully green;
- [ ] `cargo clippy --workspace --all-targets -- -D warnings` with no warnings;
- [ ] target cross-builds pass (if RPi/MCU is affected);
- [ ] relevant sim/HIL/replay scenarios pass;
- [ ] there are no changes outside the agreed scope;
- [ ] docs and the handoff report are updated;
- [ ] the Safety-Reviewer has no blocking comments;
- [ ] changes are not pushed to the remote without user permission.

For control loop changes, additionally: the balance HIL test is passed,
latency and jitter are within budget, and a rollback path exists.
