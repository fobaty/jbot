# Repeatable release fixture for balance trials

## Why this is needed

Hand-held rope releases are not repeatable. Telemetry from
`csv/jbot-hold-1795455497614.csv` shows a stable stance for about 9 s followed
by a `gy` jump from 5.9 to 134 dps in one second, caused by the operator's hand
moving at the release cue. `csv/jbot-hold-1795455254278.csv` shows the same law
holding for the full window when that disturbance did not happen. Control
conclusions drawn from hand releases are therefore dominated by the fixture,
not by the law.

The fixture must remove the support **vertically and slowly**, with no lateral
impulse, and must remain under the body as a fall catch.

## Minimal variant: cut-the-thread release

When no jack or threaded rod is available, use the existing rope with a fixed
anchor and a pair of scissors.

1. Fix the upper end of the rope to a rigid point above the robot - a door
   frame, a shelf bracket or a hook. The anchor must not move when loaded.
2. Adjust the rope length so it holds the body upright with the wheels on the
   floor and only light tension. Keep the blocks below the body as a catch.
3. Arm the robot through the driver. No hand touches the body at this point.
4. Cut the rope with scissors away from the robot. Cutting removes the tension
   without a push, which is the whole point of the fixture.
5. Tie a fresh piece of rope before the next profile.

The driver's `release-file` marker must be created before the cut, so the
scored window starts just before the body becomes free. The first few hundred
milliseconds of that window are the still-supported baseline; scoring it is
harmless because the pitch error there is near zero.

This variant needs only the existing rope, scissors and a rigid anchor. It is
less smooth than a screw-driven cradle but removes the dominant disturbance:
the operator's hand moving at the release instant.

## Design: screw-driven cradle

A rigid plate under the body is carried by a threaded rod/nut and two vertical
guide rods. Turning the handwheel lowers the plate by roughly 30 mm, which is
enough to clear the belly while keeping the plate below the body as a catch.

### Requirements

- Wheels stay on the floor for the whole trial.
- The plate contacts the belly, never the wheels or the axle.
- Travel is purely vertical; the guide rods prevent tilt.
- Lowering takes 1-2 s and is smooth enough that `gy` stays below 2 dps during
  the load transfer.
- The plate stays at least 20 mm below the belly after release so a fall is
  caught without a free drop to the floor.
- The complete fixture fits inside the 0.5 m test zone.

### Suggested bill of materials

| Item | Purpose |
|---|---|
| Plywood plate about 250 x 250 x 12 mm | Cradle under the belly |
| Thin foam or rubber pad | Protect the body, add friction |
| Trapezoidal threaded rod, about 8 mm, with nut and handwheel | Smooth vertical drive |
| Two smooth vertical rods with bushings | Anti-tilt guidance |
| Rigid base plate | Anchor for nut and guides |
| Two loose blocks or shims | Set the rest height without load |

Any equivalent screw jack may be used instead, provided it can be cranked
smoothly by hand and holds its position without creeping.

### Build notes

1. Mount the nut and the two guide bushings on the base plate so the cradle
   stays level through the full travel.
2. Attach the cradle plate to the screw and the two guide rods.
3. Fit the pad so the belly rests on a flat, centred surface.
4. Check the full travel with the robot absent: the cradle must drop clear of
   the belly and must not touch the wheels.

## Bench procedure

1. Wheels on the floor, body upright, cradle raised until it just touches the
   belly with no load. Rope attached as a secondary catch, e-stop in reach.
2. Start the driver. It arms only from a quiet pose and writes the ready file:

   ```
   cargo run -q -p jbot_console --bin hold_sweep --release -- --run \
     --pscale 0.30 --slews 300 --hold-ms 10000 --db 0.00 \
     --quiet-pitch 3 --quiet-gy 3 --release-timeout-ms 5000 \
     --release-file <path>/jbot-release --ready-file <path>/jbot-ready
   ```

3. When the ready file appears, lower the cradle smoothly over 1-2 s until it
   clears the belly, then create the release file. The scored 10 s window
   starts at that moment.
4. The driver ends each profile with `estop`. Raise the cradle again before
   the next profile.

## Fixture acceptance test

Before any gain or compensation conclusion, qualify the fixture itself:

- Run the same profile three times.
- For each run, measure `gy` over the first 200 ms after the release marker in
  the CSV (`profile` column `p...`).
- The fixture is acceptable when the peak `|gy|` in that window is at most
  2 dps in all three runs and the sign of the initial rate is consistent.

If this gate fails, improve the fixture before interpreting balance results.
Do not tune the control law to compensate for release disturbance.

## Safety

Blocks, rope and e-stop remain mandatory. The cradle is an additional catch,
not a replacement for them. No torque is applied unless mainboard feedback is
fresh. Stop immediately on any guard, fall latch or disarm.
