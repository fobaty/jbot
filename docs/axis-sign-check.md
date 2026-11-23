# Sign and axis check before any balance run

A re-mounted BNO055 or a re-wired drive can break one of four conventions the
200 Hz law depends on. Every armed run after such a change must be preceded by
this check; a wrong sign does not look like a tuning problem, it looks like the
machine fighting itself.

| Convention | What it protects |
|---|---|
| Forward lean increases `pitch` | The law leans the machine the way it is falling |
| `gy` has the same sign as `d(pitch)/dt` | The derivative term damps instead of exciting |
| Counter-clockwise turn gives positive `gz` | The yaw loop damps instead of spinning the machine |
| `test 50 50` rolls forward, `test 100 -100` pivots negative | The mixer reaches the wheels as drive, not as steer |

## Procedure

The robot is on blocks, rope attached, hands on the body, e-stop reachable, test
zone at most 0.5 m. The balance law is never armed; the tool sends `clear`,
`disarm`, `stream 100`, then the guided poses, and ends with `test 0`,
`stream 0` and `estop`.

```sh
cargo run -q -p jbot_console --bin axis_check --release -- --run
```

With the wheels free, the optional drive check adds two bounded spin tests:

```sh
cargo run -q -p jbot_console --bin axis_check --release -- --run --wheels
```

Each pose is announced on stdout and captured for three seconds after a 1.5 s
lead-in. Follow the prompts: hold upright, lean forward, lean backward, turn the
body counter-clockwise seen from above.

## Reading the report

The tool prints the measured deltas, the pitch-rate fit and the wheel signs, then
a single `PASS`/`FAIL` per convention:

| Line | Pass condition | Remedy when it fails |
|---|---|---|
| `pitch` | forward delta ≥ +3°, backward delta ≤ −3° | wrong mount selector or a zero captured in the wrong pose: re-run the guided axis calibration, then `zero` and `level` |
| `rate` | correlation ≥ 0.7 and slope ≥ 0.3 | the damper reads the wrong raw axis or the wrong sign: check the mount selector and the raw channel pairing in `firmware/pico_core/src/mount.rs` |
| `yaw` | counter-clockwise mean `gz` ≥ +5 dps | inverted yaw sign: the yaw loop would drive the spin, not damp it |
| `wheels` | `test 50 50` gives `L > 0`, `R < 0` and the pivot gives negative `gz` | mixer sign or motor wiring; see `docs/design_balance_home.md` §8.2 |

A `FAIL` on `pitch` or `rate` is a hard stop: do not arm, do not raise `pscale`.
The recording is written to `csv/jbot-axis-<timestamp>.csv` for the offline
analyzer.

## Relationship to the other checks

- The GUI console's guided **axis calibration** picks the mount selector from
  the same forward/backward/left/right poses, but it does not check the rate
  pairing or the yaw sign. Run both after a re-mount.
- `jbot-analyze` performs the same pitch/rate pairing offline on a recorded
  session; this tool performs it live and adds the yaw and wheel conventions.
- The e-stop is latched at the end, as with the other drivers. Send `clear`
  before the next run.
- The boot path used to restore the pitch zero only from a saved `pose_home`,
  so a `zero`/`save` without a home pose left `zcapt=0` after a reboot and the
  arm gate refused. Firmware now stores a `zero captured` flag in the NVM flags
  byte and restores `zero_pitch_rad` from it. Legacy records decode with the
  flag clear, so they still fall back to `pose_home` and need one fresh `zero`
  capture after the upgrade. `hold_sweep` is independent of this: it captures
  both `zero` and `level` at its own quiet baseline before arming.
