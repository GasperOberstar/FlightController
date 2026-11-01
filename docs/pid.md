# PID controller

`fc::Pid` is a single-axis PID controller with feedforward, for the rate
loop. Pure C++17, float-only, no heap allocation, no exceptions. One
instance controls one axis (roll, pitch or yaw).

## Structure

    error = setpoint - measurement

    P  = Kp * error
    I  = clamp(I + Ki * error, -iLimit, +iLimit)
    D  = Pt1( -Kd  * (measurement - prevMeasurement) )
    FF = Pt1(  Kff * (setpoint    - prevSetpoint)    )

    output = clamp(P + I + D + FF, -outLimit, +outLimit)

P, I and D react to error. D and FF are each filtered with an `fc::Pt1`
low-pass. P and I are not filtered, so they get no extra lag.

## API

    bool  setGains(kp, ki, kd, dt, kff);
    bool  setILimit(limit);
    bool  setOutLimit(limit);
    bool  setFilteredD(cutoffHz, dt);
    bool  setFilteredFF(cutoffHz, dt);
    void  reset(setpoint, measurement);
    float update(setpoint, measurement);

All setters return `false` and leave state unchanged on invalid input.
`dt` is the loop period in seconds.

## Gains are pre-multiplied by dt

`setGains` stores:

    Ki_ = ki * dt        Kd_ = kd / dt        Kff_ = kff / dt

so `update()` does no division and no extra multiplication by `dt`. The
cost is that `dt` is fixed at configuration time: if the loop rate
changes, `setGains`, `setFilteredD` and `setFilteredFF` must all be
called again with the new `dt`.

Because the integrator stores its value in **output units** (the sum of
`Ki_ * error`, not the sum of `error`), changing `ki` at runtime changes
only future increments. The stored integrator does not jump
(`ChangingKiMidRunDoesNotJumpTheIntegrator`).

## Derivative on measurement

D is computed from the measurement, not the error:

    D = -Kd * (measurement - prevMeasurement)

With D on error, a sudden setpoint step would produce a one-sample spike
(derivative kick). With D on measurement, a setpoint step changes only P
(and I over time). The minus sign makes D oppose motion: a rising
measurement gives a negative contribution, which damps the response.

The consequence is that D does not help the craft follow the stick. It
only damps motion. Following the stick quickly is the job of feedforward.

## D-term filter

Differentiation amplifies high-frequency noise, and gyro noise from motor
vibration is exactly that. The raw derivative passes through an `fc::Pt1`
configured with `setFilteredD(cutoffHz, dt)`.

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

## Feedforward

P, I and D all react to error, so they act only after the craft has
already deviated from the command. Feedforward reacts to how fast the
**setpoint** is changing, so the correction starts at the same moment as
the stick movement:

    rawFF = Kff * (setpoint - prevSetpoint)
    FF    = Pt1(rawFF)

It is the mirror image of D. D uses the measurement so a setpoint step
causes no kick. FF uses the setpoint on purpose, because the setpoint
rate is exactly the signal it needs.

`Kff_ = kff / dt` is pre-multiplied like `Kd_`, so the gain is in output
units per unit of setpoint rate. With kff = 0.01 and dt = 1/4000,
Kff_ = 40.

### FF filter

RC setpoints arrive at a few hundred Hz while the loop runs at kHz. The
difference of that stepwise signal is a pulse on each RC update and zero
in between. The `Pt1`, configured with `setFilteredFF(cutoffHz, dt)`,
smooths it. Like the D filter it uses `Pt1::setCutoff`, so retuning at
runtime keeps the filter state.

### First-sample behavior

A setpoint step of 1 gives a one-sample raw pulse of 40. The filter starts
at 0 and closes only a fraction alpha of the gap per sample (alpha is
about 0.136 for a 100 Hz cutoff at 4 kHz):

    sample 1:  0.136 * 40           = 5.4
    sample 2:  5.4 * (1 - 0.136)    = 4.7
    sample 3:  4.7 * 0.864          = 4.1   ...

So the peak is about 14% of the raw value, not 100%. This is not a bug. A
Pt1 has unity DC gain, so the total correction is preserved: the outputs
sum to 40, the same area as the raw pulse, delivered over about
1/alpha = 7 samples instead of one.

For a constant setpoint rate the raw FF is constant (0.1 per sample gives
40 * 0.1 = 4), and the output settles to it after a few time constants.

Tuning: a higher FF cutoff gives a bigger first sample and less delay but
passes more RC-update jitter. A lower cutoff is smoother and later. `kff`
can compensate for the reduced peak. As with D, this is a noise-versus-lag
trade-off.

## Anti-windup: integrator clamp

If the actuator saturates (motors at maximum, craft on the ground) the
error persists and an unclamped integrator keeps growing. When the error
finally flips sign, the integrator must unwind everything it accumulated
before the output reverses.

`setILimit(limit)` bounds the integrator to `[-limit, +limit]` in output
units. Example: with `Ki_ = 0.0025` per sample and a limit of 0.2, the
integrator saturates after 80 samples and unwinds in 80 samples. Without
the clamp, 1000 samples of windup take about 1000 samples to unwind.

## Output limit

`setOutLimit(limit)` clamps the final sum to `[-limit, +limit]`:

    output = clamp(P + I + D + FF, -outLimit, +outLimit)

This is a different job from the integrator clamp. The integrator clamp
caps how much authority I alone can have. The output clamp caps the total
command the mixer can receive. Each term can be inside its own bounds
while the sum is not (for example P = 0.4 and I = 0.3 give 0.7 against a
limit of 0.5), and only a clamp on the sum catches that
(`ClampsTheSumNotTheIndividualTerms`).

- The **clamped** value is stored as the previous output, so the value held
  on bad input is also within limits.
- It bounds the result even when huge finite inputs overflow the sum to
  infinity (`HugeFiniteInputsStayWithinLimits`).
- Sizing: values are normalized to the mixer's range. Reasonable starting
  points are about 0.5 for roll and pitch and 0.4 for yaw, with the I limit
  well below the output limit (roughly 20 to 40% of it).

Limit of this approach: the integrator still accumulates while the output
is saturated, as long as it is below its own limit. Conditional
integration (stop integrating while saturated and the error pushes
further out) is planned.

## Input validation

**Setters** return `false` and leave all state unchanged if a parameter is
invalid.

- `setGains` requires `dt`, `kp`, `ki`, `kd` and `kff` to be finite, with
  `dt` positive and the gains non-negative. Negative gains are rejected on
  purpose: for a rate loop with correctly oriented motors, a negative gain
  is a configuration error.
- `setILimit` and `setOutLimit` require a finite, positive limit. An
  infinite limit would silently disable the clamp.
- `setFilteredD` and `setFilteredFF` forward to `Pt1::setCutoff`, which
  rejects zero, negative, NaN and at-or-above-Nyquist cutoffs.

All checks are accept-lists (`!(isFinite(x) && x > 0 ...)`), so NaN is
rejected rather than slipping through a `<=` comparison.

**`update()`** checks that both inputs are finite (`isFinite`, false for
NaN and infinity). If either is not, it returns the previous output and
touches no state. This matters because the integrator, `prevMeasurement_`,
`prevSetpoint_` and both `Pt1` filters remember their past: a single NaN
would otherwise poison them permanently, and `0 * inf = NaN` can reach the
integrator even with `ki = 0`. The guard sits before every state change,
so the next good sample continues as if the bad one never happened
(`BadSamplesDoNotChangeLaterBehavior`,
`NonFiniteSetpointDoesNotSpikeLaterFF`).

These checks rely on IEEE semantics (`x - x == 0` is false for NaN and
infinity), so `-ffast-math` must not be used.

## Reset

`reset(setpoint, measurement)` clears the integrator, both filters and the
stored output, and seeds `prevSetpoint_` and `prevMeasurement_` with the
current values. Seeding both matters: without it the first D term sees
`measurement - 0` and the first FF term sees `setpoint - 0`, and both
spike. Tests that caught this: `ResetSeedsMeasurementSoFirstDHasNoSpike`,
`ResetSeedsSetpointSoFirstFFHasNoSpike`, `ConstantSetpointGivesZeroFF`.

## Known gaps

- **No conditional integration.** The integrator keeps accumulating while
  the total output is saturated, up to its own limit.
- **Unconfigured defaults are silent.** A default `Pid` has I and output
  limits of 0 (everything pinned at zero) and default `Pt1` filters have
  `k = 0` (D and FF always 0). Safe, but easy to forget. Tests must call
  every setter.
- **Held output on bad input.** A dead sensor holds the last output
  indefinitely. A bad-sample counter for failsafe is planned.
- **`dt` is passed to three setters**, so the caller can pass inconsistent
  values. A single `configure(...)` would remove that and also make
  half-configured states impossible.
- **Scaled gains can overflow.** `kd / dt` and `kff / dt` can become
  infinite even when the inputs are finite (for example a huge gain with a
  tiny `dt`).
- **NaN from overflow maps to the lower limit.** If the sum itself becomes
  NaN (for example `inf - inf`), `clamp` returns its lower bound, which is
  a full negative command.

## Test notes

`tests/test_pid.cpp` isolates each term by zeroing the other gains. The
`configure()` helper sets wide limits (I limit 100, output limit 1000) so
older tests are not affected by the clamps; output-limit tests tighten
them explicitly. Values are derived by hand: with `ki = 10` and
`dt = 1/4000`, the integrator rises 0.0025 per sample at unit error.

FF tests (`PidFF.*`) zero P, I and D so only FF contributes. They check:
first sample = alpha * Kff_, then decay by (1 - alpha); the sum of outputs
over a step equals Kff_ (area preserved); a ramp settles to
Kff_ * rate per sample; a constant setpoint gives exactly zero FF; and
`reset` seeds the setpoint.

Output tests (`PidOutput.*`) check clamping in both directions, values
inside and exactly on the limit, that the sum (including FF) is what gets
clamped, overflow safety, that the held output on bad input is within
limits, and rejection of invalid limits.

Tests were also verified by deliberately breaking the code (flipped
`isFinite` guard, removed `dFilter_.reset`, removed the clamps, a
self-assigned `prevSetpoint_`) and checking that the matching tests go
red. One such experiment also exposed a silent-default trap: with the
output limit defaulting to 0, every nonzero-output test failed until
`configure()` set a limit.