# Quad-X mixer

`fc::mix` turns one throttle value and three body-axis torque commands
(the outputs of the roll, pitch and yaw rate `Pid`s) into four motor
commands in [0, 1]. It is a pure function: no state, no heap, no
exceptions.

## Conventions

- Body axes: x forward, y right, z down.
- +roll = right side down, +pitch = nose up, +yaw = nose right
  (clockwise seen from above).
- Roll, pitch and yaw here are **torque commands about the body axes**,
  not angles. They are normalized PID outputs.

Motor layout (top view, front is up):

    M3 front-left (CCW)    M1 front-right (CW)       front

    M2 rear-left (CW)      M0 rear-right (CCW)       rear

Output index: 0 = rear-right (CCW), 1 = front-right (CW),
2 = rear-left (CW), 3 = front-left (CCW).

(Check this diagram against your real frame and fix it if it differs. The
table below is what the code uses.)

## Formula

    motor[i] = throttle + roll * R[i] + pitch * P[i] + yaw * Y[i]

| Index | Motor        | Spin | R  | P  | Y  |
|-------|--------------|------|----|----|----|
| 0     | rear-right   | CCW  | -1 | -1 | +1 |
| 1     | front-right  | CW   | -1 | +1 | -1 |
| 2     | rear-left    | CW   | +1 | -1 | -1 |
| 3     | front-left   | CCW  | +1 | +1 | +1 |

Why these signs:

- **Roll:** right side down means the left motors speed up (2, 3).
- **Pitch:** nose up means the front motors speed up (1, 3).
- **Yaw:** a propeller's reaction torque on the frame is opposite to its
  spin. To yaw clockwise, the CCW props (0, 3) speed up.

Each column sums to zero, so a pure axis command redistributes thrust
without changing the total. The roll and pitch columns are orthogonal, so
those axes do not couple.

## Saturation

The motors only accept [0, 1], but `throttle + axis part` can leave that
range. A per-motor clamp would change the *differences* between motors,
and those differences are the attitude command, so the correction would
point in the wrong direction. Instead:

1. **Scale.** Let `d[i]` be the axis part (without throttle) and
   `spread = max(d) - min(d)`. If `spread > 1` no shift can make it fit,
   so all axes are multiplied by `1 / spread`. Roll, pitch and yaw shrink
   by the same factor, so their ratios are kept. Throttle is never scaled:
   saturation must not make the craft sink.
2. **Shift.** If the highest motor is above 1, subtract the excess from
   all motors. Otherwise, if the lowest is below 0, add the deficit to all
   motors. Every motor moves by the same amount, so the differences are
   preserved exactly.
3. **Clamp.** A final clamp to [0, 1] only guards against float rounding
   (for example 1.0000001). It should not change anything meaningful.

Example (throttle 0.9, roll 0.3): the raw motors are `{0.6, 0.6, 1.2, 1.2}`.
The shift is -0.2, giving `{0.4, 0.4, 1.0, 1.0}`. The left/right difference
is still 0.6. A per-motor clamp would give `{0.6, 0.6, 1.0, 1.0}`, a
difference of only 0.4.

Example of scaling (throttle 0.5, roll 0.4, pitch 0.4): the axis part is
`{-0.8, 0, 0, +0.8}`, spread 1.6, scale 0.625. The motors become
`{0.0, 0.5, 0.5, 1.0}`.

Side effect: the shift changes collective thrust. At the top it lowers it.
At the bottom it raises it, so motors spin faster than the throttle
command asks for. This is the same idea as airmode and keeps attitude
control at low throttle.

## Result

    struct MixResult { bool ok; bool saturated; float scale; float shift; };

- `ok`: false if any input was NaN or infinite. In that case `out` is **not
  written** and must not be sent to the ESCs.
- `saturated`: true if the axis part was scaled or the set was shifted.
- `scale`: 1.0 when unchanged, below 1.0 when the axes were shrunk.
- `shift`: 0.0 when unchanged, negative when pulled down from the top,
  positive when lifted from the bottom.

`saturated` is the signal conditional integration in the PID will need. The
mixer runs after the PIDs, so the flag from cycle N reaches them in cycle
N+1, a harmless one-sample delay.

`mix` is `[[nodiscard]]`, so ignoring `ok` produces a warning.

## Known gaps

- **No idle.** Motors can reach 0. A minimum command for in-flight
  motors is not implemented.
- **`out` is untouched on bad input.** A caller that ignores `ok` sends
  stale values to the ESCs. A safe fallback (zeros) is an option.
- **No feedback to the PIDs yet.** `saturated` is reported but nothing
  consumes it.
- **Throttle is not validated.** Values outside [0, 1] are shifted or
  clamped, not rejected.

## Test notes

`tests/test_mixer.cpp` encodes the physics, not just numbers: for each
positive axis command it checks which motors are larger, so reordering
the table makes tests fail. It also checks exact patterns, that axes
are independent and mirror each other, that outputs are always in
[0, 1] over a grid, that differences are preserved whenever no scaling
happened, that throttle is not scaled, and that `MixResult` reports
`saturated`, `scale` and `shift` correctly.