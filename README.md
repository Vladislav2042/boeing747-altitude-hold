# Boeing 747 Altitude-Hold Controller — PID Design (Part C)

Group course project for **Principles of Automatic Control** (SJTU). Part C asks for an altitude-hold
controller on the longitudinal channel of a Boeing 747, simulated in MATLAB, with a stability and
performance analysis.

**My part — the report and both MATLAB scripts in this repository:** the PID controller design, the
MATLAB implementation, the closed-loop pole and stability-margin analysis, and the comparison between
the two tuning iterations. The rest of the group project (the Part A model, the Part B yaw damper, the
Part D robustness study) is the work of the other group members.

## The plant

Fifth-order longitudinal model of the 747 taken from Part A, with elevator deflection `δₑ` as the input
and altitude `h` as the output:

```matlab
A = [-0.00643   0.0263    0      -32.2    0;
     -0.0941   -0.624   820       0      0;
     -0.000222 -0.00153 -0.668    0      0;
      0         0        1        0      0;
      0        -1        0       830     0];
B = [0; -32.7; -2.08; 0; 0];
C = [0 0 0 0 1];
D = 0;
```

What the open-loop analysis (computed, not quoted) says about it:

| Property | Value | Consequence |
|---|---|---|
| Eigenvalues of `A` | `0`, `-0.6463 ± 1.1211i`, `-0.0030 ± 0.0098i` | **Marginally stable**, not unstable: the pole at the origin is the altitude channel with nothing holding it |
| Zeros of `h(s)/δₑ(s)` | `-5.6449`, **`+5.6111`**, `-0.0045` | A **right-half-plane zero** — the plant is non-minimum phase |
| Phugoid pair | `-0.0030 ± 0.0098i` (ζ ≈ 0.29) | almost undamped, very slow |
| Transfer function | `(32.7 s³ + 1.3 s² − 1035.7 s − 4.7) / (s⁵ + 1.2984 s⁴ + 1.6822 s³ + 0.01 s² + 0.0002 s)` | — |

Two consequences drive the whole design:

1. **The pole at the origin** means a steady elevator input produces a ramp in altitude, not a new
   equilibrium. The controller has to supply the restoring action itself; this is why the settling time
   in the results below is measured in minutes.
2. **The RHP zero** imposes a hard limit: any controller that demands a fast altitude response pays for
   it with a large inverse (undershoot/overshoot) excursion. No amount of PID tuning removes it — it is
   a property of the aircraft, not of the gains.

## Design

Two tuning iterations, both in `matlab/`. Gains were found by manual tuning plus automatic search in
MATLAB, with the closed-loop poles checked at every step.

| | Version 1 (`partc_pid_baseline.m`) | Version 2 (`partc_pid_improved.m`) |
|---|---|---|
| `Kp` | `-0.00022` | `-0.000300` |
| `Ki` | `-3.5·10⁻⁶` | `-5.0·10⁻⁶` |
| `Kd` | `-0.0018` | `-0.0017` |
| Dominant closed-loop poles | `-0.0015 ± 1.2299i` | `-0.0061 ± 1.1969i` |
| Remaining poles | `-1.0965`, `-0.1168`, `-0.0187`, `-0.0045` | `-1.0160`, `-0.1915`, `-0.0186`, `-0.0045` |

All closed-loop poles have negative real parts in both versions, so the design is asymptotically
stable. Version 2 moves the dominant pair further into the left half-plane, which is what cuts the
settling time.

## Results

Step command of `+100 m` in altitude, measured by `stepinfo` in MATLAB R2025a by running the scripts in
this repository:

| Metric | Version 1 | Version 2 |
|---|---|---|
| Rise time | 1.017 s | 1.027 s |
| Peak time | 3.28 s | 3.35 s |
| Peak value | 1.85 | 1.91 |
| **Settling time (2 %)** | **2401.8 s** (40 min) | **601.9 s** (10 min) |
| Overshoot | 85.46 % | 90.85 % |
| Undershoot | 0.81 % | 0.75 % |
| Steady-state error | 0 | 0 |
| Gain margin | — | 1.017 dB @ 1.20 rad/s |
| Phase margin | — | 1.08° @ 1.19 rad/s |

The 4× improvement in settling time is the point of version 2. It does **not** come for free: overshoot
grows from 85 % to 91 %, and the stability margins are razor thin — a phase margin of about 1° is
effectively no robustness at all, so any change in the aircraft model would destabilise the loop.

## Reading the result honestly

The two numbers that matter here are not the ones that look good.

* **Settling time is not set by the oscillation.** The response peaks in 3.3 s but only settles after
  10 minutes, because the tail is governed by the slow poles near the origin (`-0.0045` and `-0.0061`,
  time constants of 160–220 s), not by the fast oscillatory pair. Shortening the settling time further
  means moving those poles, which a single-loop PID on altitude cannot do — it has one measured signal.
* **Overshoot near 90 % is structural.** With an RHP zero at `+5.6 rad/s`, aggressive altitude response
  and small overshoot are mutually exclusive for this plant. Chasing the overshoot with the PID gains
  trades directly against stability margin, which is exactly what the 1.08° phase margin shows.

So the honest conclusion of Part C is: **a PID controller stabilises the altitude loop and gives zero
steady-state error, but it cannot deliver a flyable response on this model** — the trade-off between
speed, overshoot and robustness is exhausted. Fixing it requires a controller with more information,
which is what the group's later LQI extension does: augmenting the plant with an integrator state and
solving the optimal problem moves the design out of the PID trade-off entirely. That extension is
included in `report/group_extension/` for comparison, but it is not my work.

## Repository contents

```
boeing747-altitude-hold/
├─ matlab/
│  ├─ partc_pid_baseline.m    # version 1: first PID tuning
│  └─ partc_pid_improved.m    # version 2: improved tuning + margin analysis
├─ report/
│  ├─ Part_C_PID_Report.docx  # the report (Word)
│  ├─ Part_C_PID_Report.pdf   # the same report, readable in the browser
│  └─ group_extension/        # the group's LQI extension, for comparison
└─ README.md
```

The MATLAB listings were recovered from the appendix of the report and verified by running them; every
number in the tables above is printed by `partc_pid_improved.m` / `partc_pid_baseline.m`. Personal
identifiers (student number, class) were removed from the published report.

## How to run

Requires MATLAB with the Control System Toolbox.

```matlab
cd matlab
partc_pid_improved      % prints poles, step metrics, gain and phase margins; plots step and Bode
partc_pid_baseline      % version 1, for comparison
```

Both scripts build the plant from the state-space matrices, so they are standalone — nothing external to
load.

## Next steps I would take

1. **Move the slow poles.** Either a cascade structure (inner pitch-rate or pitch-attitude loop feeding
   the altitude loop) or an observer-based design, since a single altitude measurement cannot move four
   slow poles.
2. **Reclaim the margin.** Even keeping PID, robustness to the RHP zero improves with a rate-limited or
   filtered reference, or with `pidtune` under an explicit phase-margin target rather than manual gain
   search.
3. **Test the model, not just the numbers.** Part D's turbulence and parameter-variation study is where
   the 1.08° phase margin would show up as an unstable aircraft — which is the right reason to move to
   LQI.

## Stack

MATLAB R2025a · Control System Toolbox (`ss2tf`, `tf`, `pid`, `feedback`, `pole`, `stepinfo`, `margin`)
