# Water-Skipping MAV — Ground Truth and Handoff

Written 2026-10-01, at the close of Stage 1 (the CityU MPhil thesis). Intended
to be read by whoever picks this up for the IROS submission, human or Claude.

`CLAUDE.md` beside this file carries the full working context. This document is
the shorter thing: what is true, what is merely believed, and what will bite.

---

## 1. The vehicle, as built

Only one vehicle was ever built and flown. Every number here describes it.

| | |
|---|---|
| All-up mass | 90 g, including mocap markers and glue |
| Flight controller | Crazyflie **Bolt** |
| Battery | Coddar CD4S35090HV, 4S HV LiPo, 15.2 V, 350 mAh, 5.32 Wh, 90C |
| Motors | 4x Sparis 1204, 4500 KV |
| Propellers | Flash 2540-3, clear polycarbonate, 3-blade, T-mount |
| Core diameter | 150 mm, i.e. 75 mm motor-mount radius |
| Hydrofoil span | 70 mm radial, from r_in = 75 mm to r_tip = 145 mm |
| Overall | 290 mm tip to tip |
| Hydrofoil section | flat plate, 15 mm chord, 3 mm thick, beta = 30 deg |

The hydrofoils are not bolt-on parts. Each is one integral moulded leg per
motor mount: it drops vertically from the mount, then kinks outward into the
foil, like a mosquito's leg resting on water.

**There are no drag plates.** They were removed from the design. Airframe drag
is neither modelled nor used as a design lever.

**Only beta = 30 deg was ever fabricated and flown.** The 5-30 deg sweep in the
thesis is simulation only, so the model's ordering across beta is untested.

---

## 2. How it spins, which is the thing most often got wrong

Yaw torque comes from **two sources acting in the same sense**:

1. **Dominant.** Two of the four rotors are mounted **horizontally**, tangential
   to the frame, driven at a fixed thrust setpoint (~40000/65535) commanded
   independently of the lift channel.
2. **Secondary.** The two vertical lift rotors all turn the same physical
   direction, so their reaction torque **adds** rather than cancelling.

    tau_z = l*(f2 + f4)  +  c*(f1 + f3)

Consequences that matter:

- The spin rate is a **commanded, tunable operating parameter**, not an
  equilibrium. Do not describe this vehicle as relaxed-hovering and do not
  derive an equilibrium omega_z* from a torque/drag balance. There is none.
- **Cutting the lift does not stop the spin.** This is what makes the unpowered
  descent possible, and it is the real distinction from a conventional
  quadrotor, which makes yaw torque by unbalancing its counter-rotating pairs
  and therefore loses yaw authority when lift is removed.
- The ~800 deg/s step from 8800 (hover) to 8000 (lift cut) is the secondary
  reaction term vanishing. It is direct empirical confirmation of the
  two-source mechanism.

Being *able to choose* a yaw rate is NOT the distinction; any multirotor can be
commanded to a yaw rate. The supervisor caught the thesis making this error.

---

## 3. The firmware that actually flew

    repo    xuyilian/Water-skip-controller   (remote `fork`)
    branch  waterskip-fw
    tag     flown-2026-08  ->  commit 4293c2fe
    base    72b47d84  (safety checks disabled, explicit motor arming)

Verified 2026-10-01: the tag resolves to 4293c2fe, which is HEAD, and the
working clone is in sync with the fork.

The mixer, `powerDistributionTangentialSpin` in
`src/modules/src/power_distribution_quadrotor.c`:

    m2 = m4 = spinMotorThrust          // fixed, independent of control->thrust
    m1 = control->thrust - r + p       // legacy M1/M3 roll/pitch coefficients
    m3 = control->thrust + r - p       // no yaw term at all

Parameter surface, matching a live cfclient read of the board:
`powerDist.idleThrust` (persistent, 7000), `powerDist.mixerType`,
`powerDist.spinThrust`, `powerDist.thrustCap`. **There is no `m24Thrust`.**
`mixerType` and `spinThrust` are non-persistent, so they read 0 on a freshly
booted board and are set on connection by `revolvingdario.py` (~line 813).
Reading 0 in cfclient before running the flight script is expected.

`abandoned-m24thrust` (tip 53eca3c2) was **never flown**. Reasoning from it, or
from the stale gitignored `build/cf2.bin`, produced a string of wrong
conclusions in September 2026. Do not.

---

## 4. What is actually proven

Everything below is measured, from six final-build flights in a motion-capture
volume over a 0.5 m square tank filled to 70 mm.

| Claim | Evidence |
|---|---|
| The hop works | 8 water contacts with unambiguous exits |
| Entry velocity | -1.53 to -1.85 m/s |
| Exit velocity | +0.57 to +1.57 m/s, mean +1.01 |
| Contact duration | 74-222 ms, median ~118 ms |
| Spin retained | 39-47% across five flights, mean 43% |
| The cycle closes | two-hop sequences returning to within 8 mm of the previous release height |
| Controllable while spinning | flown to ~10,000 deg/s, no ceiling encountered |
| The descent is unpowered | lift command is exactly zero throughout, and through the contact |

**The reversal is the water's work.** On the clean single hop the lift command
returns within one sample of the foils leaving the water. Establishing this
required undoing a filter-lag error; see section 6.

---

## 5. What is NOT proven, and must not be claimed

- **That hopping is energetically cheaper than hovering or than a conventional
  water take-off.** Power was never logged. The thesis says so explicitly. The
  contact converts rotational energy into height at an efficiency of order 1-3%,
  so on the numbers available it is *not* an efficient converter. Its value is
  that the vehicle leaves the surface already climbing.
- **That reduced latency enabled the high spin rates.** Latency was reduced (a
  compressed 9-byte setpoint, a decoupled sender thread) but no before/after
  measurement was kept. Mention phase lag only as hedged future work.
- **The descent acceleration.** Mocap says ~4 m/s^2, about 40% of g, consistent
  across five cases, and the distance fallen agrees. The onboard `acc_z`
  disagrees and is unusable (it reads nothing like 1 g in hover). The author's
  own read of the footage doubts 40% of g. **Do not write "ballistic" and do
  not quote a descent acceleration.** The deciding test, counting video frames
  from release to splash, was never done.
- **The moment of inertia.** Never measured. `simu.m` uses a hoop at the
  motor-mount circle, I = m * r_hoop^2 = 5.06e-4 kg m^2. Predicted spin loss
  scales as 1/I and so does the conversion efficiency.
- **The model's ordering across beta.** Only one foil angle was built.

---

## 6. The traps

These each cost real time. They are listed in the order they are likely to bite.

**The filter-lag trap.** `mocap_vz_filt` is filtered twice in
`revolvingdario.py` (EMA on position at alpha 0.3, then EMA on the derivative
at alpha 0.5), giving ~35 ms of group delay and attenuated extrema.
`cmd_thrust` has no delay at all. Comparing one against the other invents an
offset that does not exist, and that is how the thesis came to claim the rotors
returned 42 ms into the contact. Durations survive the lag because both
boundaries shift together; absolute timings and peak values do not.
For any new timing work: use raw `mocap_z_raw`, mask dropout without disturbing
the uniform time base, differentiate zero-phase, and take contact as the
velocity trough to the **first** peak after it. A global maximum runs into the
powered climb and yields restitution > 1, which is impossible.

**`tab:hop_summary` was never re-cut.** It is internally consistent but, being
built on the lagged signal, understates exit speed and peak acceleration. On
the clean hop the lag-free values are +1.39 m/s and 7.6 g against the tabulated
+0.92 and 4.9. **Re-cutting that table is the first job for anyone extending
this work**, and it will move the restitution, the energy figures in Ch.1 and
Ch.2, and the abstract.

**`mocap_yawrate_deg_filt` is not filtered.** Its EMA coefficient is 1.0, a
pass-through, and it is byte-identical to the raw channel in all seven logs.

**int64 integer division.** `cmd_thrust` is int64 in the logs. In MATLAB,
`int64(32747)/65535` is 0, not 0.4997, so a percent-of-full-scale trace
silently collapses to a flat zero line while everything else looks right. Cast
to double first. `figs_results.m` does this in `trim_common` and says why.

**Disk versus hoop.** `I = m*R^2` is a hoop, not a disk; a uniform disk is
half that. The code once carried a comment saying "uniform disk" over a hoop
formula, at the tip radius rather than the motor circle, which overstated the
inertia by about 3.7x.

**Mocap dropout at the splash.** Water across the markers freezes the reported
position at its last value and drives the logged yaw rate to exactly zero. Drawn
raw this looks like a vehicle that has stopped spinning. Mask it.

**The excluded flight.** `20260730_214857_2.5hops` plus `C0228 2.5hops.MP4` used
an **earlier build** and is the one case where drop spin thrust was enabled. It
is renamed with an `olderbuild` marker. Never cite or plot it.

**`drop_spin_thrust`** exists in `revolvingdario.py` but was 0 for every
reported case. Do not mention it in any paper.

---

## 7. Where things live

Two project repositories, which are never merged:

- `xuyilian/CityUHKThesis-main` -> cloned at `paper/`. TeXPage is the source of
  truth; collaborators edit live in the browser, so pull before and push after,
  keep edits localized, and never commit build artefacts.
- `xuyilian/Water-skip-controller` -> cloned at `code/`. Two unrelated
  histories on separate branches: `main` (Python/MATLAB) and `waterskip-fw`
  (firmware C). `merge-base` between them is empty, deliberately. Do not
  "fix" this by splitting the firmware into its own repository.

`crazyflie-firmware/` at the workspace root is a working clone of upstream
bitcraze with `fork` pointing at Water-skip-controller; it is not a third
project repo.

Key files: `revolvingdario.py` (flight controller), `simu.m` (hop model and
Ch.2 figures), `figs_results.m` (Ch.5 figures from flight logs), `analyse.m`
and `analyse_prep.m` (offline analysis, including the IMU-versus-mocap
acceleration comparison), and three Python figure generators for the
schematics. The Python ones need the `cf` conda environment.

`learning/` holds 17 lessons as paired .md and .pdf. The .md files are the
source; `make_lesson_pdf.py` renders them. **Do not run
`make_lesson_pdf_LEGACY.py.bak`** — it carries the lesson text hard-coded and
would overwrite the corrected PDFs with pre-correction content.

Flight logs and video: `test results raw material/`. The curated pairs are the
ones to use; do not trawl `DataExchange/`.

---

## 8. For the IROS paper

The thesis is 17,900 words; a conference paper is roughly a fifth of that and
should be **rewritten, not trimmed**.

Keep: the hop mechanism and why the density contrast makes it work; the
two-tangential/two-vertical airframe and the decoupling of spin from lift; one
model figure; the strongest experimental result.

Drop: most of the model's design-space discussion, the control derivation
(it is inherited and the thesis says so), and the instrumentation detail.

Redo first: re-cut `tab:hop_summary` with the lag-free method. Everything
quantitative in a new paper should rest on that rather than on the thesis
table.

The honest framing, which survived review: the contribution is the **locomotion
mode**, not the control law, and not an efficiency claim.
