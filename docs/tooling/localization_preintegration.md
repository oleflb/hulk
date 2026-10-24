# IMU preintegration

The trajectory estimator integrates IMU readings into relative rotation, velocity, and position constraints between spline endpoints.
Preintegration retains high-rate motion evidence without evaluating a separate spline factor for every reading.
There is no GTSAM runtime dependency; GTSAM is used only to generate an independent test reference.

For recorded-data workflows, see the [localization replay guide](https://github.com/HULKs/hulk/blob/main/crates/nodes/localization-3d/examples/README.md).

## Configuration

The shipped timing, calibration, noise, and bias settings are in [`localization3d.json5`](https://github.com/HULKs/hulk/blob/main/etc/parameters/base/localization3d.json5).
[`TimingParameters`](https://github.com/HULKs/hulk/blob/main/crates/nodes/localization-3d/src/parameters.rs) defines backward-compatible defaults for omitted fields, which are not necessarily the shipped values.
Trajectory spacing and preintegration interval are distinct settings; interval boundaries also respect pose and bias knots.
Use a fresh localization epoch after changing physical calibration: retained samples can be recalibrated, but a marginalized prior cannot.

## Model and covariance

Specific force is calibrated in Robot axes: `(raw - fixed_bias) * scale`.
Integration is at the physical IMU origin, avoiding numerical angular-acceleration differencing.
For gyro `w`, force `f`, interpolated bias `b`, and held sample duration `h`:

```text
a       = delta_R * (f - b_a)
delta_p = delta_p + delta_v*h + a*h²/2
delta_v = delta_v + a*h
delta_R = delta_R * Exp((w - b_g)*h)
```

Force uses the old rotation.
Bias interpolation uses the step midpoint, with separate sensitivities for both knots.
Bias column order is gyro XYZ then accel XYZ.
Actual timestamps determine duration.

Spline endpoint states are transformed to the sensor origin using `p_s = p + R*r` and `v_s = v + R*(omega × r)`.
With positive-up compensation `g = [0,0,9.81]`, residuals are:

```text
R: Log(corrected_delta_R⁻¹ * R0⁻¹ * R1)
V: R0⁻¹ * (v_s1 - v_s0 + g*dt) - corrected_delta_v
P: R0⁻¹ * (p_s1 - p_s0 - v_s0*dt + g*dt²/2) - corrected_delta_p
```

Endpoints are evaluated on the spline, not read from control values.
The analytic mounting-offset derivatives include both position and angular-velocity contributions.
Upright rest supplies +9.81 m/s²; **free fall supplies zero**, not a missing measurement.

The full 9×9 covariance uses `[right rotation, velocity in interval-start axes, position in interval-start axes]`.
Let `D = Exp((w-b_g)*h)ᵀ`, `E = -delta_R*[f-b_a]×*h`, and `H = E*h/2`:

```text
F = [D, 0, 0; E, I, 0; H, h*I, I]
P = F*P*Fᵀ + Q
Q_RR = Jr((w-b_g)*h) * Jr((w-b_g)*h)ᵀ * gyro_density²*h
Q_VV = accel_density²*h*I
Q_VP = Q_PV = accel_density²*h²/2*I
Q_PP = (accel_density²*h³/4 + integration_density²*h)*I
```

This follows GTSAM's discrete held-measurement noise convention using continuous densities, not an exact continuous stochastic solution.
Isotropic acceleration noise removes the rotation from its noise blocks.
Whitening `L⁻¹`, for `P = L*Lᵀ`, is prepared when publishing a factor; covariance is never replaced by independent R/V/P weights.

## Lifecycle

- Ordered arrivals extend cached deltas; completed prefixes are not reintegrated every solve.
  Raw samples, interval caches, and graph factors retire with the window.
- The active partial interval reaches the newest source sample.
  That newest gyro has no following span yet, so one temporary instantaneous constraint stabilizes the tail.
  It is removed before the next solve when the reading enters an integral.
- Gaps exceeding `timing.max_imu_gap` are not integrated.
  Gyros without usable following spans retain instantaneous constraints.
  Missing, nonfinite, or disabled acceleration creates rotation-only pieces; full pieces resume when force returns.
  Missing force is never replaced with zero.
- SDK up-direction evidence is one duration-weighted endpoint observation per bin.
  Partial intervals carry proportionally less information, relative to `imu_preintegration.reference_duration`.
  Isolated gap readings retain the instantaneous tilt weight.
  The newest reading supplies tilt only if an interval does not already supply it.
  Dense attitude history remains available for exposure-time interpolation, independent heading validation, and recovery.
- Late or replaced readings invalidate affected spans; calibration or noise changes invalidate active caches.
  Accepted bias changes exceeding the configured reintegration thresholds at either knot trigger reintegration between solves.
  The optimizer uses analytic first-order correction within a solve; rejected trial biases never become integration references.
- Recovery replays raw IMU data through the same insertion and preparation path.
  Bias rollback, joint marginalization, broad recovery confidence, heading references, and generation/epoch checks remain in force.
- Preparation failures must stop optimization and visual-rejection retries rather than solve a stale graph.
  Diagnostics retain the failing interval, timestamps, operation, and underlying error.

## Limitations

- SDK attitude is correlated with raw IMU evidence.
  Covariance describes the configured model, not independently calibrated physical uncertainty.
- Bias estimation needs independent motion evidence, and has a learning transient.
  A single stationary orientation cannot separate accelerometer bias, scale, tilt, and gravity errors or establish temperature calibration.
  Do not assume stationary feet during unknown contact or flight.
- Numerical convergence does not certify physically correct motion or successful field recovery.
  Application-level heading, tilt, visual, and covariance checks remain necessary.
- The coarse spline limits motion bandwidth, and the zero-rotation process prior biases sustained blind turns.
  The `sustained_fast_turn_preserves_off_grid_camera_estimates` regression checks continuity and covariance, not high-accuracy blind-turn tracking.
  Cost and Jacobian evaluation must reject the same nonsmooth rotation-log domain.

## Reference and regression checks

Reference sources are GTSAM 4.2 `ManifoldPreintegration.cpp`, `ImuFactor.cpp`, `TangentPreintegration.cpp`, and `NavState.cpp` under its BSD license.
Reproduce the fixture from the repository root:

```sh
uv run --no-project --python 3.11 --with gtsam==4.2 --with 'numpy<2' \
  python docs/tooling/preintegration_reference.py
```

The wheel uses tangent preintegration, whose multistep rotation update differs from manifold integration.
The reference uses native single-sample PIMs, converts their covariance chart, then composes them using numerical derivatives of GTSAM NavState operations.
Finally it rotates velocity/position error axes and permutes `[R,P,V]` to `[R,V,P]`.
It does not assert equality with the wheel's unconverted multistep covariance.

The 100 ms fixture includes nonzero bias, changing rotation, 1–4 ms sample periods, and a force impulse.
Mean agreement is within `1e-12`; the covariance difference whitened by the Rust covariance is below `1e-7`.
Separate finite differences test both interpolated bias Jacobians.
f32/f64 automatic differentiation covers both factor modes and nonzero mounting offsets.
A physical 4 rad/s offset-IMU test distinguishes rotation about the Robot origin from spurious body translation.

Additional checks cover late-data cost/information equivalence, partial prefixes, missing force/free fall, impulses, gaps/late bridges, calibration changes, bias boundaries and reintegration, retirement, rollback/recovery, double flight, and allocation-free factor evaluation after workspace preparation.

```sh
cargo test --locked -p localization-fagra -p localization-3d
cargo clippy --locked -p localization-fagra -p localization-3d --all-targets --no-deps -- -D warnings
```

External-recording and GPU tests remain explicitly ignored; the normal suite does not establish their results.

## Profiling

The synthetic tracking benchmark needs no external recording:

```sh
cargo test --locked --release -p localization-3d benchmark_tracking_estimation -- --ignored --nocapture --test-threads=1
```

Compare the same inputs, parameters, build settings, and solve schedule, with one benchmark process at a time.
Measure process CPU time or `perf stat` task-clock/instructions separately from wall time: live-system contention can dominate elapsed time.
Replay throughput and isolated benchmarks do not establish a hard real-time guarantee under the full robot workload.
