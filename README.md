# Boeing 747 Altitude-Hold Controller — from PID to LQI

Course project for **Principles of Automatic Control** (SJTU), Part C: design a longitudinal control
system that holds the altitude of a Boeing 747, simulate it in MATLAB/Simulink, and analyse stability
and performance.

The engineering story is a progression, not a single design: a PID that technically works but is
uselessly slow, then a standard LQR that is fast but leaves a steady-state error, then an **LQI**
controller — the LQR augmented with an integrator state — which is the first design that satisfies
both requirements at once.

This repository contains the technical report (`.docx` and `.pdf`). The models and simulations live in
MATLAB and were not part of the submission archive, so no source code is published here.

## Scope of contribution

Part C of a **group** course project (Group 3). My part was the control engineering itself: evaluating
the PID baseline, then the LQR, then designing and tuning the LQI controller — the weighting matrices,
gains, simulations, pole analysis and the comparison table below are the result of that work. The
written report was assembled within the group from these results, and is published here with personal
identifiers (name, student number) removed.

## The problem

From Part A of the project, the longitudinal dynamics of the Boeing 747 give a **fourth-order**
state-space model and a transfer function from elevator deflection `δₑ` to altitude `h`. The open-loop
plant is **unstable**, which rules out simple lead–lag compensation: cascade compensation on an
unstable plant cannot relocate the right-half-plane pole reliably, so a feedback controller with a
clear stability argument is required instead.

Task, as stated in the assignment:

1. design a longitudinal control system for altitude hold;
2. simulate it in MATLAB/Simulink;
3. analyse stability and performance.

## Design path

### 1. PID — stable, but too slow

A PID controller was tuned with the MATLAB **PID Tuner** (automatic optimisation). The closed loop is
stable and the steady-state error is small, but the settling time comes out around **2100 s** — half an
hour to settle is not a controller, it is a suggestion. The reason is structural: the plant's slow
mode dominates, and a single-loop PID has no way to address the multi-state dynamics directly.

### 2. Standard LQR — fast, but no integral action

State feedback with LQR weights produced a fast transient, but zero steady-state error is only
guaranteed for the nominal model. Any constant disturbance or model offset leaves a residual altitude
error, because the controller has no memory of accumulated error.

### 3. LQI — the design that works

The plant is augmented with an **integrator state** on the altitude error, and LQR is solved on the
augmented system. The weighting matrices that were settled on after iterative tuning:

```
Q_aug = diag([0.01, 0.1, 1, 10, 50])
R_aug = 50
```

The largest weight, `50`, sits on the **integral error state**: that is what forces zero steady-state
altitude error. The physical states carry smaller weights, trading a little transient aggressiveness
for smooth elevator activity.

Resulting gains (MATLAB, `lqr` on the augmented system):

```
K  = [-0.0158,  0.5216,  -16.4488,  -519.9373]
Ki = 1.0000
```

## Results

Performance as reported in the project:

| Method | Settling time | Overshoot | Undershoot | Steady-state error |
|---|---|---|---|---|
| Open loop | > 300 s | — | — | large |
| PID | ~2100 s | ~0 % | — | ~0.01 % |
| **LQI** | **1.13 s** | **1.5 %** | **12.6 %** | **0 %** |

Closed-loop poles of the LQI system:

```
-8.04 ± 8.22i,  -2.94 ± 1.94i,  -0.0045
```

All poles have negative real parts, so the closed loop is asymptotically stable. The pole at
`-0.0045` is the shifted integrator pole — slow by design, and exactly what provides the integral
action that removes the steady-state error. The pair at `-2.94 ± 1.94i` shapes the dominant transient,
and `-8.04 ± 8.22i` is the fast pair that keeps the elevator response crisp.

The undershoot of 12.6 % is worth noting: the altitude response initially moves away from the setpoint
before the integrator pulls it back, so a real implementation would have to check that this excursion
stays inside the altitude band the aircraft is allowed to fly.

## Repository contents

```
boeing747-lqi-control/
├─ report/
│  ├─ Part_C_LQI_Report.docx   # original report (Word)
│  └─ Part_C_LQI_Report.pdf    # same report, rendered for reading in the browser
└─ README.md
```

The published copy of the report has the author's name and student identification number removed.

## Stack

MATLAB · Control System Toolbox · Simulink · PID Tuner · LQR/LQI design
