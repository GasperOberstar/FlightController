# PT1 low-pass filter

`fc::Pt1` is a first-order low-pass filter (proportional element with one
time constant). It is the same as an RC filter or an exponential moving
average. Planned uses: gyro smoothing and D-term filtering in the PID.

## Continuous time

    tau * dy/dt + y = x        H(s) = 1 / (tau * s + 1)

with tau = 1 / (2 * pi * fc), where fc is the cutoff frequency in Hz.

- Step response: y(t) = 1 - e^(-t/tau). Reaches 63.2% at t = tau.
- At fc the gain is 1/sqrt(2) = 0.707 (-3 dB).
- Roll-off above fc is only 20 dB/decade.

## Discretization (backward Euler)

Replace dy/dt with (y[n] - y[n-1]) / dt and evaluate x at step n:

    tau * (y[n] - y[n-1]) / dt + y[n] = x[n]

Solving for y[n]:

    y[n] = y[n-1] + k * (x[n] - y[n-1])
    k    = dt / (tau + dt) = w / (1 + w),   w = 2 * pi * fc * dt

Cost per sample: one subtract, one multiply, one add. State: one float.

The exact (step-invariant) alternative is k = 1 - e^(-w). It matches backward
Euler when w is small (cutoff much lower than the sample rate) and differs
near Nyquist. Backward Euler needs no `exp()`, so it is cheaper to
recompute when the cutoff changes at runtime.

## Design decisions

**Invalid parameters are rejected, not clamped.** `init()` and `setCutoff()`
return `false` if cutoff or dt is zero, negative, NaN, or at/above Nyquist
(`cutoff * 2 * dt >= 1`). State is left unchanged. Clamping would hide a
unit mix-up, such as passing a sample rate where a period belongs, and the
filter would silently run with the wrong cutoff. Returning `bool` is an
exception-free way to report errors, as `core/` is built with
`-fno-exceptions`.

**The validation uses an accept-list** (`!(x > 0 && ...)`) so NaN fails the
check. NaN compares false with everything, so `x <= 0` would let it through.
This relies on IEEE semantics, so `-ffast-math` must not be used.

**A default-constructed filter is inert.** k = 0, so the output never moves.
This is safe but silent, and a test documents it.

## Known limits

- At the Nyquist limit w = pi, so k = pi / (1 + pi) = 0.76. Backward Euler
  never becomes a pass-through, so a filter near Nyquist still smooths.
- Every low-pass filter adds phase lag. In a feedback loop, lag erodes
  stability, so less noise means more delay.

## Phase lag at the cutoff

H(j*2*pi*fc) = 1 / (1 + j), so the phase is -atan(1) = -45 degrees, and the
magnitude is 0.707.

Delay equivalent: phase = 2 * pi * f * delay, so at fc, delay = 1 / (8 * fc).

- fc = 150 Hz: delay = 0.83 ms.
- Well below fc, the delay approaches tau = 1 / (2 * pi * fc), which is
  1.06 ms at 150 Hz.

At a 4 kHz loop (0.25 ms per sample), this is several loop cycles of delay
in the D-term path. That is the trade-off behind FPV filter tuning.

## Test notes

- The closed-form test checks y[n] = 1 - (1 - k)^n to 1e-5.
- After about one tau (6 samples at 100 Hz / 4 kHz) the output is 0.583, not
  0.632. Rounding n accounts for about 0.022, and the backward Euler pole
  (1/(1+w) = 0.864 vs. e^-w = 0.855) accounts for about 0.027.