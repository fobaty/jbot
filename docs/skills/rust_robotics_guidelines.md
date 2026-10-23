# Rust Robotics Engineering Guidelines

Status: living document. Audience: Orchestrator, Architect, Rust-Host-Dev,
Rust-Embedded-Dev, Navigation-Dev, QA-Tester, Safety-Reviewer.

This document is the project-specific skill for Rust work on JBOT-WheeledBiped. It turns
the system requirements into engineering rules for the two compute tiers:

- **MCU tier (ESP32-S3 prototype, STM32H723/H743 production candidate):** `no_std`,
  deterministic control, FOC, IMU, watchdog, CAN/UART.
- **SBC tier (Raspberry Pi 5, 16 GB):** Linux, Tokio, sensor drivers, estimation,
  navigation/SLAM, telemetry, Zenoh/ROS 2 bridge.

Read this together with `docs/architecture.md`, `docs/specification.md`,
`docs/system_requirements.md`, and `docs/agent_workflow.md`. If this document and the
architecture disagree, the architecture and the requirement IDs win; raise the conflict
instead of choosing silently.

---

## 0. How agents must use this document

### 0.1. Rules of engagement

1. **Find the requirement ID first.** Every change must trace to a requirement or task
   (`SYS-*`, `INT-*`, `NAV-*`, `SAF-*`, `M3-*`, `M4-*`, `M5-*`, `M6-*`).
2. **Design the contract before the code.** Types, units, wire layout, timestamps, and
   failure behavior are part of the contract. Do not invent them locally.
3. **One owner per crate.** Do not edit another crate or `Cargo.toml` without the
   Orchestrator/Architect agreeing.
4. **Safety is the highest priority.** Never disable or weaken a watchdog, limit,
   E-stop path, staleness check, or command-age check to make a test pass.
5. **Do not add dependencies by convenience.** New crates require an explicit decision;
   check whether the workspace already uses an equivalent crate.
6. **No `git push`, publish, deploy, or hardware run** without explicit user approval.
7. **English only** in code comments, doc comments, commit messages, and documentation.
8. **Measure, do not assume.** Any claim about jitter, latency, throughput, or memory
   must be backed by a command, log, or measurement artifact.

### 0.2. Stop-and-escalate triggers

Stop and request an Architect/Safety-Reviewer decision before changing any of:

- the 1 kHz balance loop, FOC loop, watchdog feeding, or fail-safe state machine;
- E-stop, command timeout, DAMPING transition, or actuator limits;
- the RPi-MCU wire protocol, frame header, CRC, sequence, or CAN ID allocation;
- timestamping, time synchronization, or sensor staleness policy;
- any `unsafe` block, DMA ownership, MMIO, FFI, or custom allocator;
- the public types in `jbot_common` or a crate dependency boundary;
- a dependency that pulls `std` into firmware or allocates in a hot path.

### 0.3. Requirement anchors used below

| Area | Requirement IDs |
|---|---|
| Balance loop and jitter | `SYS-02`, `SAF-*`, `M3-*` |
| End-to-end latency | `SYS-07` |
| Transport | `INT-01`-`INT-10` |
| Runtime and memory | `INT-11`-`INT-19` |
| LiDAR, SLAM, localization | `NAV-01`-`NAV-35` |
| Extensions and isolation | `EXT-09`, `EXT-13`, `EXT-33` |

---

## 1. System invariants

### 1.1. Tier boundaries

| Concern | MCU | Raspberry Pi 5 |
|---|---|---|
| Balance and posture control | **Authority**, 1 kHz | Never |
| FOC / motor current loop | **Authority**, 20-40 kHz | Never |
| Watchdog and fail-safe | **Authority** | Supervisor only |
| Command validation and limits | Final authority | Proposes commands |
| LiDAR / camera ingestion | No | **Authority** |
| SLAM, localization, planning | No | **Authority** |
| Telemetry and logging | Producer | Aggregation/storage |
| Time base | Monotonic timer, sample timestamps | Monotonic clock + offset estimate |

The RPi may use `PREEMPT_RT`, CPU isolation, and real-time priorities to reduce latency.
That improves soft real-time behavior; it does **not** make the RPi the safety authority.

### 1.2. Non-negotiable rules

1. **No heap allocation in the control path.** No `Vec` growth, `String`, `format!`,
   `Box`, `Arc` creation, or allocating error context inside a control step.
2. **No `panic!`, `unwrap()`, or `expect()` in production control/transport paths.**
   Convert invalid input into a typed error and enter a safe state.
3. **All queues are bounded.** Telemetry may drop; E-stop, heartbeat, and safety events
   must never be blocked by a full telemetry queue.
4. **Commands are latest-value, not a work queue.** Do not replay stale setpoints after
   a stall. Validate sequence and age before applying a command.
5. **Control commands do not update the actuator setpoint when invalid.** Validate CRC,
   version, length, sequence, timestamp, and value ranges first.
6. **Timestamp at the sample/ingestion point.** Never timestamp a measurement when it is
   finally processed by the application.
7. **Use monotonic time for intervals and deadlines.** `SystemTime` is for logs and
   wall-clock display only.
8. **Do not move the balance loop to Linux.** The MCU keeps the robot safe if the RPi
   stalls, reboots, or loses the link.
9. **Do not put Zenoh, ROS 2, DDS, USB, or disk I/O on the 1 kHz path.**
10. **No `unsafe` without a documented safety argument and review.** Prefer safe
    abstractions and isolate unavoidable hardware access in a HAL module.

---

## 2. Safe and performant Rust

### 2.1. Type and API discipline

- Make invalid states unrepresentable where it is cheap: newtypes for units
  (`MetersPerSecond`, `Radians`, `Microseconds`), enums for modes, and validated
  constructors for limits and configuration.
- Keep wire types and domain types separate. A `FrameHeader` is not a `RobotState`.
  Deserialize into a wire type, validate, then convert.
- Derive `Debug`, `Clone`, `Copy` for small POD messages. Avoid `String` in messages;
  use fixed-size arrays or an interned ID.
- Return typed errors from libraries. Use `anyhow` only at application boundaries;
  it allocates and is not `no_std`.
- Mark validation boundaries with `#[must_use]` and reject non-finite values
  (`NaN`, `inf`) before they enter estimation or control.
- Use checked arithmetic for counters and indices derived from untrusted input.
  Plain arithmetic is acceptable in a proven, bounded hot loop, but document the bound.

```rust
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MetersPerSecond(pub f32);

impl MetersPerSecond {
    pub fn new(value: f32, limit: f32) -> Option<Self> {
        (value.is_finite() && value.abs() <= limit).then_some(Self(value))
    }
}
```

### 2.2. Error handling and validation

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandError {
    BadVersion,
    BadLength,
    BadCrc,
    Stale,
    OutOfRange,
}

pub fn validate_command(raw: &[u8], now_us: u32) -> Result<Setpoint, CommandError> {
    // Order matters: cheap structural checks first, semantic checks last.
    // Any error leaves the previous safe setpoint untouched.
    let frame = decode(raw).map_err(|_| CommandError::BadCrc)?;
    if frame.version != PROTOCOL_VERSION {
        return Err(CommandError::BadVersion);
    }
    if frame.payload_len > MAX_PAYLOAD {
        return Err(CommandError::BadLength);
    }
    if now_us.wrapping_sub(frame.timestamp_us) > COMMAND_MAX_AGE_US {
        return Err(CommandError::Stale);
    }
    Setpoint::try_from(frame.payload).ok_or(CommandError::OutOfRange)
}
```

Rules:

- Validate in this order: size/version, CRC, sequence, age, semantic range.
- On error, increment a counter and keep the last safe setpoint or enter `DAMPING`
  according to the state machine. Never partially apply a frame.
- Do not log every rejected frame at high rate. Count and sample.
- Treat sequence gaps as diagnostics, not as permission to apply an old command.
  A command older than the newest accepted sequence is ignored.

### 2.3. `no_std` firmware

**Memory model**

- Static allocation only in the control path: `static` buffers, `heapless::Vec`,
  `heapless::spsc`, `arrayvec`, or fixed arrays.
- No heap in interrupt context or control tasks. If a heap is linked for boot-time
  configuration, allocate and validate everything before the control loop starts.
- DMA buffers must be `'static`, correctly aligned, and owned by one component.
  Never pass a stack buffer to DMA.
- Avoid large stack frames in ISRs and tasks. Measure the worst-case stack and keep
  margin; recursion is forbidden in the control path.

```rust
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize clocks, watchdog, IMU, bus, and PWM here.
    // Freeze configuration into static state before entering the loop.
    let mut ticker = Ticker::every(Duration::from_micros(1_000));
    loop {
        ticker.next().await;
        // One fixed-work balance step: no heap, no logging, no blocking waits.
        // The hardware timer and watchdog enforce the deadline.
    }
}
```

**Concurrency and interrupts**

- Use `critical-section` or an embassy `Mutex<CriticalSectionRawMutex, RefCell<_>>`
  for shared state. Keep critical sections to a few instructions.
- Prefer single-producer/single-consumer queues or `watch`-style latest-value state over
  a global mutex. The ISR writes; the task consumes.
- Never call a blocking function, allocate, log, or wait from an ISR.
- Timestamp at the hardware event (DRDY, CAN RX, timer capture), not after the ISR.
- Use `Acquire`/`Release` for ownership handoff, `Relaxed` only for statistics that do
  not gate behavior.
- `static mut` is forbidden. Use `static` with a safe interior-mutability abstraction or
  a critical-section-protected cell.

**Async on the MCU**

- Embassy tasks are acceptable when they compile to fixed, static state machines and do
  not allocate. RTIC is acceptable when priority-based preemption is the clearer model.
- A `.await` in a control task is a scheduling point. Do not put unbounded work between
  the timer tick and the control output.
- Keep task futures small. Large futures consume stack/RAM; inspect sizes when adding a
  task or a large local buffer.
- Panic strategy in release is `abort`. Provide a panic handler that records a fault
  marker and lets the hardware watchdog reset the MCU. Do not rely on unwinding.

**Logging**

- Use `defmt` with deferred/RTT logging, not `println!`.
- Do not log in the 1 kHz loop. Record counters and a ring buffer, then dump on request
  or after a fault.
- Log values, not formatted strings, to keep flash/RAM and timing predictable.

### 2.4. Tokio and async on the Raspberry Pi 5

Tokio is the orchestration runtime, not a hard real-time guarantee. The task model is
defined in `docs/architecture.md` §6.3; follow it.

**Task placement**

| Work | Placement |
|---|---|
| CAN/UART receive | Dedicated blocking thread + bounded channel, or a carefully isolated async driver |
| Command egress | Tokio task with absolute periodic deadline, or a real-time thread if latency is measured |
| Estimation | Tokio task, 100-200 Hz, no I/O |
| Navigation/SLAM | Tokio task or separate process, 10-30 Hz |
| Perception | Tokio task or separate process; CPU/GPU heavy work off the runtime |
| Telemetry/logging | Bounded channel, drop-oldest for loss-tolerant data |
| Shutdown | `CancellationToken` or equivalent, coordinated |

**Periodic loop pattern**

```rust
use std::time::Duration;
use tokio::sync::watch;
use tokio::time::{interval_at, Instant, MissedTickBehavior};

#[derive(Clone, Copy)]
struct State {
    timestamp_us: u64,
    velocity: f32,
}

#[derive(Clone, Copy)]
struct Setpoint {
    velocity: f32,
}

async fn control_task(
    mut state: watch::Receiver<State>,
    setpoint: watch::Sender<Setpoint>,
) {
    let period = Duration::from_millis(10);
    let mut tick = interval_at(Instant::now() + period, period);
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
        tick.tick().await;

        // Copy out of the watch immediately; do not hold the Ref across an await.
        let snapshot = *state.borrow_and_update();
        let command = compute(&snapshot);

        // Latest-value semantics: overwrite, never queue stale commands.
        setpoint.send_replace(command);
    }
}
```

**Async rules**

- `tokio::time::interval` is a soft timer. Use `MissedTickBehavior::Skip` or `Delay`;
  never `Burst`, which can produce a catch-up storm.
- Use `watch` for latest state/setpoint, bounded `mpsc` for streams, `broadcast` for
  faults/mode changes, and `oneshot` for request/response.
- Never use an unbounded channel. Bound every queue and define the overflow policy.
- Never hold `std::sync::MutexGuard` or a `Ref` across `.await`.
- Do not call blocking `std::fs`, `serialport`, or `socketcan` from an async task.
  Use a dedicated thread for long-lived blocking I/O; use `spawn_blocking` only for
  bounded blocking work.
- Treat `select!` as a cancellation boundary: verify each branch is cancellation-safe.
  Use `biased;` when priority order is required (safety/cancel before data).
- Cancel tasks explicitly on shutdown and wait for them. Dropping a task that owns
  hardware without a shutdown sequence can leave the bus or device in a bad state.
- Log with `tracing` spans, but not at high rate in the control path. Counters are
  cheaper than log lines.

**Linux latency hygiene (when measured jitter matters)**

- Prefer `PREEMPT_RT`; isolate CPUs (`isolcpus`, `nohz_full`, `rcu_nocbs`) and pin the
  communication/latency-sensitive thread.
- Use `SCHED_FIFO` for the dedicated comms/control-egress thread only, with a priority
  below critical kernel work. Do not make every Tokio worker real-time.
- `mlockall(MCL_CURRENT | MCL_FUTURE)` to avoid page faults; pre-touch buffers.
- Keep disk, network, USB enumeration, and SLAM contention away from the latency thread.
  Use systemd resource limits/cgroups for the navigation and perception processes.
- Do not claim hard real-time on Linux. Measure with `cyclictest`, `perf`, and
  end-to-end timestamping; record p50/p99/p99.9/max and deadline misses.

### 2.5. `unsafe` policy

- Pure computation crates and modules, including `jbot_common` and any future
  estimator crate, should use `#![forbid(unsafe_code)]`.
- Crates with hardware access should use `#![deny(unsafe_code)]` and allow it only in a
  small, reviewed HAL module. Every block needs a `// SAFETY:` comment explaining the
  invariant, ownership, alignment, and lifetime.
- Do not use `unsafe` to bypass borrow checking, silence a lint, or micro-optimize
  without measurement.
- Do not `transmute` network/sensor bytes into structs. Parse explicitly with
  `from_le_bytes`, or use a reviewed zero-copy crate (`zerocopy`, `bytemuck`) only after
  checking alignment, padding, and endianness.
- Miri can check host-side UB; loom can check concurrency primitives. Neither replaces
  HIL testing. Hardware registers and DMA still need bench verification.

### 2.6. Performance rules

- Optimize after measuring. Use `criterion` benches, `perf`, and MCU cycle counters.
- Prefer data-oriented, cache-friendly layouts. For large point clouds, consider
  structure-of-arrays; for small messages, a plain `Copy` struct is fine.
- Avoid per-iteration allocation, formatting, and dynamic dispatch in hot paths.
- Avoid repeated `sin`/`cos`/`sqrt` for the same values. Precompute tables or use
  squared distances.
- Use `f32` on the MCU unless `f64` is required; `f64` may compile to slow software
  routines. Match the precision of the algorithm and the sensor.
- Do not add `#[inline(always)]` everywhere. Let LTO and the compiler work; inspect the
  generated code before overriding.
- The release profile already uses LTO, one codegen unit, and `panic = "abort"`. Do not
  weaken these settings for firmware or control binaries.

---

## 3. Serial / UART

### 3.1. Contract

- Transport: 3.3 V TTL, full duplex, 921600 or 1 Mbaud, 8N1 (`INT-04`).
- Framing: COBS with `0x00` delimiter, max frame 256 bytes, CRC-16/CCITT-FALSE over
  header + payload.
- Resynchronization: after a CRC error or oversized frame, discard until the next
  `0x00`; never try to "repair" a partial frame.
- The logical schema is shared with CAN FD. Keep one protocol implementation and one
  set of tests, not two divergent parsers.

### 3.2. Decoder rules

- A streaming decoder must handle arbitrary chunk boundaries: one read may contain half
  a frame, two frames, or garbage.
- Use fixed buffers. No `Vec` growth and no `String` parsing.
- Check maximum encoded and decoded length before copying.
- Verify CRC before deserializing. Verify version/length before applying.
- Track and expose counters: bytes, frames, CRC errors, oversize, resyncs, sequence gaps,
  last valid frame age.
- Never block the reader on a slow consumer. For commands, publish latest value; for
  telemetry, drop oldest and count drops.

```rust
const MAX_WIRE_FRAME: usize = 256;

struct FrameDecoder {
    encoded: [u8; MAX_WIRE_FRAME],
    decoded: [u8; MAX_WIRE_FRAME],
    len: usize,
    crc_errors: u32,
    oversize: u32,
}

impl FrameDecoder {
    fn feed(&mut self, bytes: &[u8], mut on_frame: impl FnMut(&[u8])) {
        for &byte in bytes {
            if byte == 0 {
                if self.len > 0 {
                    if let Some(frame) = self.finish_frame() {
                        on_frame(frame);
                    }
                    self.len = 0;
                }
                continue;
            }
            if self.len == self.encoded.len() {
                self.oversize = self.oversize.wrapping_add(1);
                self.len = 0;
                continue;
            }
            self.encoded[self.len] = byte;
            self.len += 1;
        }
    }

    fn finish_frame(&mut self) -> Option<&[u8]> {
        // Decode COBS in place, validate length and CRC, then hand out the payload.
        // Return None and increment the appropriate counter on any error.
        decode_and_validate(&self.encoded[..self.len], &mut self.decoded).ok()
    }
}
```

The exact COBS and CRC calls depend on the adopted crate versions. Pin the versions and
add golden vectors from the protocol specification, including the CRC check value.

### 3.3. Linux implementation

- Use `tokio-serial` for an async UART, or `serialport` on a dedicated thread. Do not
  mix blocking and async access to the same port.
- Configure baud rate, 8 data bits, no parity, one stop bit, and no software flow control
  unless the hardware uses RTS/CTS.
- Drain stale bytes at startup before enabling the protocol, then reset the decoder.
- Handle `WouldBlock`, partial reads, EOF, and disconnect. A zero-byte read is not a
  valid frame boundary.
- Use a read buffer of at least the maximum encoded frame; call the decoder for every
  chunk. Do not use `read_line`, UTF-8 conversion, or `BufReader::lines`.
- Split read and write halves if the driver supports it, or protect the port with a
  single owner. Serialize writes through one task to avoid interleaved frames.
- On disconnect, report a typed fault, increment counters, and let the supervisor decide
  whether to fail over to CAN or enter a safe state.

```rust
let mut port = tokio_serial::new("/dev/ttyAMA0", 921_600)
    .open_native_async()?;
let mut raw = [0u8; 512];
let mut decoder = FrameDecoder::new();

loop {
    let n = port.read(&mut raw).await?;
    if n == 0 {
        continue;
    }
    decoder.feed(&raw[..n], |frame| {
        // Validate and publish. Keep this closure non-blocking and allocation-free.
    });
}
```

The constructor name and feature flags vary by `tokio-serial` version; verify the
adopted version and document it in the crate.

### 3.4. MCU implementation

- Prefer DMA receive with an idle-line interrupt or a circular buffer. Timestamp when
  the delimiter is received, not when the parser task runs.
- Keep the ISR short: copy/queue bytes and return. Parse in a lower-priority task.
- Use `heapless` buffers sized to the protocol maximum. Never allocate in the UART path.
- Apply hardware flow control if the cable or baud rate requires it. On overflow,
  discard the partial frame and count the event.
- A UART fault must not block the control loop. The control loop continues with the last
  safe setpoint until the command watchdog expires.

### 3.5. Tests

- Unit: COBS encode/decode round-trip, max length, zero-length payload, embedded zeros,
  CRC corruption, truncated frame.
- Property: arbitrary byte streams never panic, never overrun buffers, and always
  resynchronize at the next delimiter.
- Integration: loopback with a pty or a virtual serial pair; split frames across reads;
  inject garbage and long runs without delimiters.
- Fault: disconnect mid-frame, flood the receiver, stall the consumer; verify bounded
  memory and the documented overflow policy.

---

## 4. CAN and CAN FD

### 4.1. Contract and priorities

- Primary bus: CAN FD ISO 11898-2, 1 Mbit/s arbitration, 5 Mbit/s data phase
  (2 Mbit/s fallback), 120 ohm termination at both ends, stub < 0.3 m (`INT-01`-`INT-03`).
- 11-bit IDs for control; 29-bit IDs reserved for expansion. Lower IDs win arbitration,
  so E-stop and heartbeat must use low IDs and must not be displaced by telemetry.
- Commands are latest-value. Telemetry is loss-tolerant. E-stop is redundant and
  highest priority.
- All multi-byte fields are little-endian. The application header and CRC rules are the
  same as for UART.

Known contract ambiguity: `docs/architecture.md` §5.3 describes an 8-byte header but
lists fields totalling 9 bytes. Do not implement a new wire layout from a guess. The
Architect must freeze the exact layout, size, and padding in the protocol ADR, and the
implementation needs golden byte vectors and property tests.

### 4.2. Linux SocketCAN setup

```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
candump -ta can0
```

- Verify the adapter and kernel actually support CAN FD. A classic-CAN-only interface
  will silently reject FD frames or fail to configure.
- Use `restart-ms` for automatic bus-off recovery, but the MCU must still enter a safe
  state if bus-off persists.
- Monitor error counters and bus-off events. Do not hide them in debug logs only.
- Use `vcan0` for integration tests; it validates framing and application logic, not
  timing or electrical behavior.

### 4.3. Receive and transmit architecture

The Linux `socketcan` API is blocking. Keep it off the async runtime:

```rust
let (tx, mut rx) = tokio::sync::mpsc::channel::<ReceivedFrame>(256);

std::thread::Builder::new()
    .name("can-rx".into())
    .spawn(move || {
        let socket = socketcan::CanSocket::open("can0")?;
        loop {
            match socket.read_frame() {
                Ok(frame) => {
                    if tx.blocking_send(frame).is_err() {
                        break; // Runtime is shutting down.
                    }
                }
                Err(error) => {
                    // Count the error, apply the documented recovery policy, and
                    // avoid a busy loop on a persistent failure.
                    let _ = error;
                }
            }
        }
        Ok::<(), std::io::Error>(())
    })?;
```

- Use one writer task/thread per bus. Serialize writes so frames from different
  subsystems never interleave.
- Use hardware acceptance filters to drop irrelevant IDs early. Do not subscribe to
  everything and filter in userspace unless diagnosing.
- Timestamp at reception in the driver thread; convert to the common time base with the
  estimated offset.
- For CAN FD, use the crate/version that exposes FD frames and payload lengths up to 64
  bytes. Do not emulate FD by sending multiple classic frames unless the protocol
  explicitly defines fragmentation.
- Do not use CAN for time-critical work if the adapter is on USB and the measured
  latency is unstable. The architecture requires a real CAN FD HAT/adapter; verify it
  on the bench.

### 4.4. `no_std` CAN

- Use `embedded-can` traits for portable frame handling; use the MCU HAL/FDCAN driver
  for hardware access.
- Configure bit timing and sample point from the transceiver/clock budget, not by
  copying a random example. Validate with an oscilloscope and bus load tests.
- Use RX FIFO interrupts/DMA, hardware filters, and timestamps. Keep the ISR to
  enqueue/copy and return.
- Frames are fixed-size; no heap. Use `heapless::Vec<u8, 64>` or a fixed array with an
  explicit length.
- Validate DLC/length, version, CRC where applicable, and sequence before use. Reject
  malformed frames without changing the actuator setpoint.
- On bus-off, stop transmitting motion commands, report the fault, and follow the
  state machine. Do not spin in a recovery loop inside the control task.

### 4.5. Command semantics and faults

- Apply only the newest valid command. If a command is older than the newest accepted
  sequence, ignore it even if it arrives later.
- Check command age before use. The MCU must stop motion after 100 ms without a valid
  command (`SAF-*`).
- E-stop is idempotent, repeated, and must not depend on the normal telemetry path.
- Track: RX/TX counts, bus-off, error-warning/passive, CRC errors, sequence gaps,
  command age, queue drops, and last valid frame time.
- On persistent bus-off or transport failure, enter `DAMPING` and report the fault.
  Automatic recovery must not resume motion without a fresh valid command and mode.

### 4.6. Tests

- Unit: frame construction, length validation, ID filters, sequence wrap-around.
- Property: random malformed frames never panic and never update the setpoint.
- Integration: `vcan` loopback; duplicate, reordered, stale, and missing frames; queue
  overflow; bus-off simulation where supported.
- HIL: real transceiver, termination, bus load, error frames, disconnect, and recovery.
- Measure command age and end-to-end latency; do not infer them from loop rate alone.

---

## 5. LiDAR, point clouds, and maps

### 5.1. Pipeline placement

```
LiDAR device
  -> driver task (bytes/packets -> validated scan)
  -> timestamp + frame transform
  -> bounded scan channel (latest or small queue)
  -> SLAM / localization / obstacle layer (10-30 Hz)
  -> map / costmap
  -> planner
```

- The driver owns the device and its buffers. It publishes immutable, timestamped scans.
- SLAM and scan matching are CPU-heavy: run them in a dedicated task/thread or separate
  process, never on the control path. Use `spawn_blocking` or a dedicated worker for a
  bounded compute job; do not lock the async runtime with a long synchronous loop.
- Use a bounded channel with a documented policy. For live navigation, the latest scan
  is usually more valuable than an old backlog; drop old scans and count the drops.

### 5.2. Ingestion and validation

- Read exact packet lengths; reject short/oversized packets before indexing.
- Parse little-endian fields explicitly; never cast a byte slice to a struct.
- Validate: protocol version, packet type, point count, angle range, range limits,
  intensity/reflectivity range, timestamps, and sequence/ring ordering.
- Reject `NaN`/`inf`, negative ranges, and returns outside the sensor's documented
  minimum/maximum. Count rejected points instead of silently accepting them.
- Capture a timestamp as close to the measurement as possible. For USB/serial LiDARs,
  record arrival time and, if available, the device timestamp; for Ethernet LiDARs, use
  the sensor timestamp and compensate for transport delay.
- Use a monotonic time base. Convert device time with the estimated offset; do not mix
  `SystemTime` and `Instant`.

```rust
#[derive(Clone, Copy, Debug, Default)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub t_offset_us: u32,
}

pub fn scan_to_points(
    ranges: &[f32],
    start_angle: f32,
    angle_increment: f32,
    min_range: f32,
    max_range: f32,
    out: &mut Vec<Point>,
) {
    out.clear();
    for (index, &range) in ranges.iter().enumerate() {
        if !range.is_finite() || range < min_range || range > max_range {
            continue;
        }
        let angle = start_angle + index as f32 * angle_increment;
        let (sin, cos) = angle.sin_cos();
        out.push(Point {
            x: range * cos,
            y: range * sin,
            z: 0.0,
            intensity: 0.0,
            t_offset_us: 0,
        });
    }
}
```

For production, reserve `out` once and reuse it. Never allocate per point. For 3D
clouds, include ring/laser ID and per-point time offset; do not assume all points share
one timestamp.

### 5.3. Point cloud representation

- Choose AoS (`Vec<Point>`) for simplicity or SoA (`xs`, `ys`, `zs`, ...) for filtering
  and SIMD-friendly access. Decide based on measurement, not fashion.
- Preallocate to the sensor's maximum points per scan and reuse the buffer. `clear()`
  keeps capacity; `Vec::new()` in the loop causes repeated allocation.
- Cap the number of points per scan and per map update. A malformed sensor must not
  exhaust memory.
- Downsample before scan matching: voxel/grid filter for 3D, angular/range decimation
  for 2D. Do the cheap rejection first.
- Use squared distances for threshold comparisons; avoid `sqrt` until the final result.
- Store timestamps alongside points when deskewing is required. Deskewing without
  per-point time is a guess; document the approximation.

### 5.4. Frames and transforms

- Follow `REP-105` frames: `map`, `odom`, `base_link`, and sensor frames
  (`NAV-19`). Do not mix frame conventions inside one module.
- Keep transforms typed (`Isometry3<f32>` or a project type) and calibrated in config.
  Do not hard-code mounting offsets in the SLAM code.
- Transform once per scan/cloud, not once per point, when the transform is constant.
  For per-point timestamps, interpolate the transform deliberately and document it.
- Check that the transform is finite and that the timestamp is aligned before fusing.
- Publish the pose with its frame IDs and timestamp; downstream code must not infer them.

### 5.5. Mapping and costmaps

- Map resolution is at most 5 cm (`NAV-07`). A 1000 m² map at 5 cm is about 400,000
  cells. Store a flat grid, not one `Vec` per cell.
- Use compact cell types: `i16` log-odds or `u8` probability/cost. Avoid `f32` per cell
  unless the memory and bandwidth cost is justified.
- Keep static, obstacle, inflation, and cost layers separate but backed by flat arrays.
  Reuse buffers across updates.
- Ray-cast only within the sensor range and map bounds. Clamp log-odds; do not let a
  single noisy return dominate the map.
- Apply inflation/costmap updates outside the control path. A navigation update must not
  block the safety loop (`NAV-26`, `EXT-33`).
- Save maps atomically: write to a temporary file, fsync, then rename. Version the map
  with the robot configuration hash (`NAV-10`, `NAV-12`).
- Loading a 1000 m² map must not stall the runtime. Load in a worker, validate the
  header/resolution/bounds, then publish atomically.

### 5.6. Staleness and degradation

- Every scan, pose, and map update carries a timestamp and source ID.
- Compute age with monotonic time. If LiDAR or localization is older than 300 ms, stop
  or reduce speed (`NAV-31`); do not drive on stale data (`NAV-18`).
- After a gap, require a fresh, valid scan/pose for a defined settle period before
  resuming autonomous motion.
- Distinguish "no data" from "zero returns" and "all returns invalid". They require
  different fault handling.
- If SLAM fails, keep the last safe map/pose only for diagnostics; do not continue
  planning on an unvalidated map.
- Camera/AI failure must not block baseline LiDAR navigation (`EXT-13`).

### 5.7. Tests

- Replay recorded scans and point clouds; compare pose/map metrics against golden traces.
- Inject malformed packets, truncated packets, bad lengths, NaN/inf ranges, timestamps
  that jump backwards, duplicate packets, and out-of-order rings.
- Verify memory stays bounded for a long run and for adversarial point counts.
- Verify staleness behavior: disconnect, freeze, slow publisher, and clock offset.
- Verify map save/load round-trip and atomic replacement under a simulated crash.
- On HIL, verify timestamp alignment against IMU/odometry and the 10 ms synchronization
  target (`NAV-05`, `EXT-09`).

---

## 6. Memory management and jitter

### 6.1. Allocation policy

| Context | Allocation | Policy |
|---|---|---|
| MCU ISR / control / FOC | Forbidden | Static, `heapless`, fixed arrays |
| MCU boot/config | Allowed before the loop | Validate, freeze, then no growth |
| RPi control/egress 100 Hz | No steady-state allocation | Preallocate, reuse, `watch`/bounded channels |
| RPi estimation 100-200 Hz | Avoid in the step | Fixed-size state, reusable scratch |
| RPi SLAM/perception | Allowed | Bounded, outside control, monitor RSS |
| Logging/telemetry | Bounded | Ring buffer; drop and count on overflow |
| Tests | Allowed | Use a counting allocator to prove hot-path behavior |

- Reserve capacities at construction. In a hot loop, `Vec::clear()` is fine; `push`
  beyond capacity is not.
- Do not use `String`, `format!`, `to_string`, or `Box<dyn Error>` in a control step.
- Avoid `Arc<Mutex<Vec<_>>>` as a shared bus. It combines locking, allocation, and
  unbounded growth. Prefer ownership transfer and latest-value channels.
- If shared immutable data is needed, `Arc` at initialization is acceptable; cloning an
  `Arc` in a hot loop is usually a design smell.
- Prefer `bytes::Bytes` or an arena only when the crate is approved and the lifetime
  model is clear. Cheap clones are not free if they hide unbounded retention.
- Use a counting global allocator in host tests to assert zero allocations in selected
  functions. This catches accidental `format!`, `collect`, and boxed futures.

### 6.2. The 1 kHz MCU loop

The budget is 1000 µs per period with jitter ≤ 50 µs (`SYS-02`).

- Trigger from a hardware timer. Use a fixed-period schedule, not a delay chain that
  accumulates drift.
- Keep the ISR minimal. If the control law runs in a task, use a priority that cannot be
  preempted by telemetry/housekeeping.
- Read IMU/encoder data from DMA-completed buffers with a timestamp captured at the
  sample event.
- Use fixed-work math. Avoid division, `sin`/`cos`, and `f64` unless the cost is measured
  and fits the budget.
- Do not log, allocate, lock a contended mutex, wait for a bus, or perform flash/EEPROM
  writes in the loop.
- Feed the independent watchdog only after all health checks pass. Never feed it from a
  timer ISR unconditionally (`SAF-04`).
- On a missed deadline or invalid sample, apply the documented safe behavior; do not
  silently continue with stale data.

### 6.3. Host latency and jitter

- The RPi command/estimation loops are soft real-time. Keep periods absolute and skip
  missed ticks; do not burst.
- Isolate the latency-sensitive thread from Tokio worker contention when measured
  latency requires it. A dedicated `std::thread` with `SCHED_FIFO` and a monotonic
  absolute sleep is often more predictable than a busy Tokio task.
- Avoid synchronous disk/network/USB operations in latency-sensitive tasks. Use bounded
  queues and background workers.
- Avoid page faults: preallocate, pre-touch, and `mlockall` the latency-sensitive
  memory. Do not grow a `Vec` in the loop.
- Remember that USB CAN/serial adapters add variable latency. Measure the actual adapter;
  do not assume a datasheet number.
- CPU frequency scaling, thermal throttling, and background SLAM can all increase
  jitter. Test under realistic load and record the conditions.

### 6.4. Measurement

- MCU: toggle a GPIO at loop entry/exit or use the DWT cycle counter; capture with a
  logic analyzer/oscilloscope. Record min/max/jitter and deadline misses over hours.
- MCU: keep a ring buffer of period, execution time, sensor age, and fault counters for
  blackbox upload.
- Host: measure end-to-end with timestamps at sample, command egress, bus receipt, and
  actuator application. Report p50/p99/p99.9/max, not only the mean.
- Use `cyclictest` and `perf sched` for the OS baseline; use `hdrhistogram` or equivalent
  for application latency. Attach results to the task/PR.
- A change that touches the control path is not done until it has a before/after jitter
  measurement or an explicit statement that the path is unchanged.

### 6.5. Common failure modes

| Symptom | Likely cause | Correct response |
|---|---|---|
| Periodic spikes every N ms | Allocator, logging, or timer catch-up | Remove allocation/logging; fix tick policy |
| Increasing latency over time | Unbounded queue or memory growth | Bound the queue; monitor depth/RSS |
| Random multi-ms stalls on RPi | Page faults, disk I/O, CPU contention | Prefault, isolate, move work off the thread |
| Control output jumps after a gap | Stale command/scan applied | Age/sequence validation, safe state |
| Bus errors under motor load | Grounding/termination/EMI | Fix wiring; verify with scope; do not mask |
| Watchdog resets under load | Loop overrun or blocking call | Profile the loop; reduce work; never disable IWDG |

---

## 7. Verification and definition of done

### 7.1. Required checks

```bash
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
```

For affected targets, also run the cross-builds:

```bash
cargo build --release -p jbot_node
cargo build --release --target aarch64-unknown-linux-gnu -p jbot_node
cargo build --release --target thumbv7em-none-eabihf -p firmware
```

Add `cargo deny check` and `cargo audit` where the dependency set changes (`INT-21`).

### 7.2. Change-specific gates

| Change | Additional evidence |
|---|---|
| Framing / protocol | Golden vectors, property tests, malformed-input tests, UART loopback |
| CAN | `vcan` integration, filter tests, bus-off/recovery, HIL transceiver test |
| LiDAR / SLAM | Replay regression, malformed packets, staleness test, memory bound |
| Control / estimation | Unit tests, allocation check, HIL jitter/latency measurement |
| Async / Tokio | No blocking in async, bounded channels, cancellation test |
| Firmware | Cross-build, stack/RAM report, watchdog test, HIL bench |
| Any safety path | Safety-Reviewer approval and fault-injection evidence |

### 7.3. Definition of done

- Requirement ID and affected crates are stated.
- Public contracts and units are documented; no local protocol guess.
- No new allocation, blocking call, `unwrap`, or `unsafe` in a hot/safety path.
- Bounded queues and documented overflow behavior.
- Timestamps and staleness checks are present where data crosses a boundary.
- Errors lead to a defined safe state; counters/metrics are exposed.
- Tests cover the normal path, malformed input, and the relevant fault path.
- Formatting, check, tests, and clippy are green; cross-builds pass if affected.
- Jitter/latency/memory claims have measurements or are explicitly out of scope.
- Documentation is updated in English; the handoff report includes risks and open
  questions.

---

## 8. Review checklist for agents

### Before coding

- [ ] I know the requirement ID and the crate(s) I may modify.
- [ ] I have read the relevant architecture section and protocol contract.
- [ ] I know whether this path is hard real-time, soft real-time, or best effort.
- [ ] I know the allocation/latency/memory budget and the failure behavior.
- [ ] I am not inventing a dependency, wire layout, unit, or timestamp convention.
- [ ] A safety review is requested if the change touches control, watchdog, E-stop,
      protocol, timestamps, or `unsafe`.

### While coding

- [ ] No `unwrap`/`expect`/`panic!` in production control or transport code.
- [ ] No heap allocation or logging in the hot path.
- [ ] All channels/queues are bounded with an explicit overflow policy.
- [ ] Latest-value semantics for commands; no stale command replay.
- [ ] No blocking I/O in async tasks; no lock held across `.await`.
- [ ] Timestamps are monotonic and captured at the source.
- [ ] Inputs are validated before affecting actuators or maps.
- [ ] `unsafe` is absent, or isolated with a `// SAFETY:` argument and review.
- [ ] Comments explain *why*, in English; no conversational comments.

### Before handoff

- [ ] `cargo fmt`, `check`, `test`, `clippy -D warnings` pass.
- [ ] Relevant cross-builds and sim/HIL/replay tests pass.
- [ ] Jitter/latency/memory measurements are attached for hot-path changes.
- [ ] Failure behavior and metrics are documented.
- [ ] No changes outside the agreed scope; no secrets; no `git push`.
- [ ] The report states what is covered, what is not, and the remaining risks.

---

## 9. Anti-patterns

| Anti-pattern | Why it is dangerous | Use instead |
|---|---|---|
| `unwrap()` in a control/transport path | Panic or wrong output on bad input | Typed error + safe state |
| `Arc<Mutex<Vec<_>>>` as a bus | Lock contention, allocation, unbounded growth | Ownership + `watch`/bounded channel |
| Unbounded `mpsc` | Memory growth and latency under load | Bounded channel + drop/count policy |
| Queuing every control command | Stale commands execute late | `watch` latest-value semantics |
| `sleep(period)` delay chain | Drift and catch-up bursts | Absolute timer/`interval_at` + `Skip` |
| `MissedTickBehavior::Burst` | Burst of ticks after a stall | `Skip` or `Delay` |
| `SystemTime` for `dt` | Clock jumps break control | Monotonic `Instant`/timer |
| `Vec::new()`/`push` per point | Allocator jitter and memory growth | Preallocate/reuse/cap |
| One `Vec` per map cell | Massive allocation and poor locality | Flat grid with compact cells |
| `format!`/`tracing` in the loop | Allocation and I/O jitter | Counters/ring buffer/`defmt` |
| Blocking `socketcan`/serial in Tokio | Stalls worker threads | Dedicated thread + channel |
| `std::sync::Mutex` across `.await` | Deadlock/stall risk | Restructure or async-aware primitive |
| `transmute` on network/sensor bytes | UB from alignment/padding/endianness | Explicit parsing or audited zero-copy |
| Ignoring sequence gaps | Stale/duplicate commands applied | Sequence + age validation |
| Feeding the watchdog unconditionally | Hides a hung control loop | Health-gated feed + IWDG |
| Treating CAN/UART as reliable | Silent loss and stale control | Timeouts, latest-value, fault policy |
| Transforming every point with a full matrix | CPU cost and jitter | Transform per cloud or precompute |
| `f64` math on a small MCU | Soft-float latency | Measured `f32`/fixed-point design |
| `#[inline(always)]` everywhere | Code bloat and I-cache pressure | Measure, then optimize selectively |
| Disabling a limit to pass a test | Safety regression | Fix the design or change the test |

---

## 10. References

Internal:

- `docs/system_requirements.md` - hardware tiers, real-time targets, safety.
- `docs/specification.md` - requirement IDs and acceptance criteria.
- `docs/architecture.md` - task model, protocol, CAN IDs, timing, crate boundaries.
- `docs/agent_workflow.md` - roles, task packets, handoff, review gates.
- `docs/roadmap.md` and `docs/tasks.md` - phase order and task evidence.

External (read the version that matches the adopted dependency):

- The Embedded Rust Book; the Embassy Book; the RTIC Book.
- `embedded-hal`, `embedded-can`, `heapless`, `defmt`, `postcard`, `cobs`, `crc`.
- Tokio documentation: tasks, `watch`, `mpsc`, `select!`, timers, cancellation.
- Linux kernel documentation: SocketCAN, CAN FD, `PREEMPT_RT`, CPU isolation.
- ROS conventions: REP-103 (units/frames), REP-105 (coordinate frames).
- ISO 11898-2 for CAN FD physical layer; the chosen LiDAR's protocol manual.
