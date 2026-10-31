# PID controller

`fc::Pid` is a single-axis PID controller for the rate loop. Pure C++17,
float-only, no heap allocation, no exceptions. One instance controls one
axis (roll, pitch or yaw).

## Structure

    error = setpoint - measurement

    P = Kp * error
    I = clamp(I + Ki * error, -iLimit, +iLimit)
    D = Pt1( -Kd * (measurement - prevMeasurement) )

    output = P + I + D

## Gains are pre-multiplied by dt

`setGains(kp, ki, kd, dt)` stores:

    Ki_ = ki * dt        Kd_ = kd / dt

so `update()` does no division and no extra multiplication by `dt`. The
cost is that `dt` is fixed at configuration time: if the loop rate changes,
`setGains` and `setFilteredD` must be called again with the new `dt`.

Because the integrator stores its value in **output units** (it holds the
sum of `Ki_ * error`, not the sum of `error`), changing `ki` at runtime
changes only the future increments. The stored integrator does not jump.
This is a test (`ChangingKiMidRunDoesNotJumpTheIntegrator`).

## Derivative on measurement

D is computed from the measurement, not from the error:

    D = -Kd * (measurement - prevMeasurement)

If D used the error, a sudden setpoint step would produce a one-sample
spike (derivative kick). With D on measurement, a setpoint step changes
only P (and I over time). The minus sign makes D oppose the motion: a
rising measurement gives a negative contribution, which damps the response.

The consequence is that D does not help the craft follow the stick. It only
damps motion. Following the stick quickly is the job of feedforward
(planned).

## D-term filter

Differentiation amplifies high-frequency noise, and gyro noise from motor
vibration is exactly that. The raw derivative is passed through an
`fc::Pt1` low-pass filter, configured with `setFilteredD(cutoffHz, dt)`.

- Only the D path is filtered. P and I use the raw measurement, so they
  get no extra lag.
- `prevMeasurement_` always holds the raw previous measurement. The only
  filtered state is inside the `Pt1`.
- `setFilteredD` calls `Pt1::setCutoff`, not `init`, so retuning at
  runtime keeps the filter state and does not cause a glitch.
- Cost: phase lag in the D path. Lower cutoff means less noise and more
  delay (see `pt1.md`). This is the central tuning trade-off.

Hand-checked example (kd = 0.01, dt = 1/4000, so Kd_ = 40): a measurement
rising 0.1 per sample gives a raw D of -4. With a 100 Hz filter the first
output is alpha * -4 (alpha is about 0.136), and it settles to -4 after
many time constants.

## Anti-windup: integrator clamp

If the actuator saturates (motors at maximum, craft on the ground) the
error persists and an unclamped integrator keeps growing. When the error
finally flips sign, the integrator must unwind everything it accumulated
before the output reverses.

`setILimit(limit)` bounds the integrator to `[-limit, +limit]` in output
units. Example: with `Ki_ = 0.0025` per sample and a limit of 0.2, the
integrator saturates after 80 samples and unwinds in 80 samples. Without
the clamp, 1000 samples of windup take about 1000 samples to unwind.

Limits of this approach: clamping the integrator alone does not stop windup
against the **total** output saturating. If P and D already push the output
to its maximum, an integrator below its own limit still adds to it.
Output limits and conditional integration are planned.

## Input validation

**Setters** (`setGains`, `setILimit`, `setFilteredD`) return `false` and
leave all state unchanged if a parameter is invalid: non-positive or NaN
`dt`, negative or NaN gains, non-positive or NaN limit, or an invalid
filter cutoff. They use accept-list conditions (`!(x > 0 && ...)`) so NaN is
rejected. Negative gains are rejected on purpose: for a rate loop with
correctly oriented motors, a negative gain is a configuration error.

**`update()`** checks that both inputs are finite (`isFinite`, true for
neither NaN nor infinity). If either is not, it returns the previous output
and touches no state. This matters because the integrator, `prevMeasurement_`
and the `Pt1` all remember their past: a single NaN would otherwise poison
them permanently, and `0 * inf = NaN` can reach the integrator even with
`ki = 0`. The guard sits before every state change, so the next good sample
continues as if the bad one never happened (`BadSamplesDoNotChangeLaterBehavior`).

These checks rely on IEEE semantics (`x - x == 0` is false for NaN and
infinity), so `-ffast-math` must not be used.

## Reset

`reset(measurement)` clears the integrator, the D filter and the stored
output, and seeds `prevMeasurement_` with the current measurement. Seeding
matters: without it the first D term would see `measurement - 0`, a large
jump, and spike. (A test hit exactly this: calling `update(3, 3)` after
`reset(0)` gives D = -16.29 because the measurement moved from 0 to 3.)

## Known gaps

- **No output limits.** `P + I + D` can exceed what the mixer can use, and
  can overflow to infinity with huge finite inputs and gains.
- **Unconfigured defaults are silent.** A default `Pid` has an I limit of 0
  (integrator pinned at zero) and a default `Pt1` has `k = 0` (D always 0).
  Both are safe but easy to forget.
- **Held output on bad input.** A dead sensor holds the last output
  indefinitely. A bad-sample counter for failsafe is planned.
- **`dt` is given to two setters** (`setGains` and `setFilteredD`), so the
  caller can pass inconsistent values. A single `configure(...)` would
  remove that.

## Test notes

`tests/test_pid.cpp` isolates each term by zeroing the other gains. Values
are derived by hand: with `ki = 10` and `dt = 1/4000`, the integrator rises
0.0025 per sample at unit error. Tests were also verified by deliberately
breaking the code (flipped `isFinite` guard, removed `dFilter_.reset`,
removed the clamp) and checking that the matching tests go red.