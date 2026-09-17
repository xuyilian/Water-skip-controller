# Water-skip-controller

Code for the water-skipping MAV: the ground-station flight controller, the hop
simulation, and the offline analysis that produces the thesis figures.

This repository holds **two unrelated projects on two branches**. They share no
history and are never merged.

| Branch | Contents | Checked out at |
|---|---|---|
| `main` | Ground station (Python) and analysis (MATLAB) | `code/Water-skip-controller/` |
| `waterskip-fw` | Crazyflie firmware fork (C) | `crazyflie-firmware/` |

The firmware branch is a fork of `bitcraze/crazyflie-firmware`. It lives here so
that all code for the project sits in one repository. Do not merge the branches.

## Live files

| File | What it does |
|---|---|
| `revolvingdario.py` | The flight controller. Runs off-board at 100 Hz, takes mocap pose over UDP, sends setpoints over the radio. **Must stay at the repository root**: it imports the sibling packages `RisLib/` and `jumping/`. |
| `simu.m` | Blade-element hop simulation. Produces the Chapter 2 figures. |
| `figs_results.m` | Builds the Chapter 5 figures from flight logs. Writes straight into the thesis `Assets/` folder, resolved relative to this file, so **do not move it** without updating `OUT_DIR`. |
| `analyse.m`, `analyse_prep.m` | Offline analysis of a single flight log. Run `analyse_prep` first. |

Supporting packages: `RisLib/` (logging, radio), `jumping/` (estimators, UDP
rigid-body receiver), `Library/`.

## Data

`DataExchange/` holds the raw flight logs, roughly 1300 `.mat` files. It is
gitignored: these are working data, not source. The curated cases the thesis
actually cites live outside this repository, in
`test results raw material/test_results_designs/`.

`matlab_figures*/` are regenerated plot outputs and are likewise gitignored. The
figures that reach the paper are committed in the thesis repository instead.

## archive/

Superseded work, kept for reference rather than deleted.

- `archive/scripts/` — earlier controllers and one-off MATLAB analyses, last
  touched June and July 2026. Superseded by the live files above.
- `archive/ab-model-data/` — identified A/B state-space models. This line of
  work (the `K_AB` advancing/retreating-blade term) was removed from the thesis;
  the data is kept only so the decision can be revisited.

Nothing in `archive/` is imported by the live code.

## Firmware

The firmware that flew the August 2026 experiments is commit `4293c2fe` on
`waterskip-fw`, tagged `as-flown-2026-07-29`. Its distinguishing feature is
`powerDistributionTangentialSpin`, which holds the two tangential rotors at a
fixed thrust independent of the lift channel:

```c
m2 = m4 = spinMotorThrust;          // fixed; independent of control->thrust
m1 = control->thrust - r + p;       // no yaw term
m3 = control->thrust + r - p;
```

Verified against the live parameter table read off the board: `powerDist`
exposes `idleThrust`, `mixerType`, `spinThrust` and `thrustCap`. `mixerType` and
`spinThrust` are non-persistent and are set on every connection by
`revolvingdario.py`, so they read `0` on a freshly booted board.
