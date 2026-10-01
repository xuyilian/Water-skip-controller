# Project: Water-Skipping MAV Paper — Context for Claude

## START HERE (updated 2026-10-01) -- STAGE 1 IS CLOSED
The MPhil thesis is finished and the supervisor's first-round review is fully
answered: all 63 annotations closed. The thesis runs ~17,900 words over six
chapters, 12 figures, 3 tables, no unresolved citations or cross-references.

**Work on the thesis has STOPPED.** The project now moves to an IROS
submission in a NEW repository and a new conversation. This file, together
with `HANDOFF.md` beside it, is the ground truth to carry across.

Read in this order:
  1. `HANDOFF.md` (workspace root) -- what was built, what is proven, what is
     not, and the traps. Written specifically for the next conversation.
  2. This file, especially "AS-BUILT VEHICLE", "YAW GENERATION",
     "THE FILTER-LAG TRAP", and "Known gaps / cautions".
  3. `paper/CityUHKThesis-main/REVIEW-ROUND-1.md` -- the review and how each
     annotation was resolved. Historical record now, not a work list.
  4. `Water-Skipping-MAV-Guide.pdf` (workspace root) -- repository orientation.

What is still OPEN on the thesis, all of it needing the author rather than
analysis: front matter (panel names, acknowledgements, funding, publications,
department as registered), two photo slots (hydrofoil close-up in Ch.3,
experimental setup in Ch.5), and two `TODO(setup)` comments in Ch.5 (mocap
camera count / tracked volume / frame rate, and the video camera model).

If shown the supervisor's marked-up PDF: it is a build that PREDATES the
Chapter 5 results, the citations and every fix. Essentially all of it is
resolved. `REVIEW-ROUND-1.md` records what was done.

## What this is
This is a paper on a water-skipping micro aerial vehicle (MAV) with rotating
hydrofoils. The vehicle spins continuously about its yaw axis; four hydrofoils
rotate rigidly with it. Locomotion cycle: hover → drop into freefall onto a
still water surface → the spinning hydrofoils briefly submerge → eject the MAV
back into the air → recover under control → repeat (a "hop").

## Scope / focus — THREE STAGES, in order (user's plan, 2026-08-19)
Experiments are DONE (concluded ~2026-08-14). No new data is coming. Everything
below is written from results already recorded.

**STAGE 1 (NOW) — the MPhil thesis, in `paper/CityUHKThesis-main/`.**
The priority is COMPLETION, not polish. Fill every gap, close every stub, get a
whole document that stands end to end. Prefer a finished section written plainly
over a perfect paragraph, and prefer stating a limitation honestly over spending
a day removing it. Ship it.

**STAGE 2 (NEXT) — ICRA/IROS conference paper, a SEPARATE new repo.**
6-7 pages, IEEE conference template pulled fresh from the official site. NOT a
trim of the thesis file: content is rewritten from the thesis, to a higher
standard, at roughly a fifth of the length. Expect to keep the hop mechanism,
one model figure, the vehicle, and the strongest experimental result, and to
drop nearly everything else. Do not start this until Stage 1 is done.

**STAGE 3 (LATER) — journal paper, RAL or better.**
Builds on the conference paper. This is where the open items earn their keep:
free-surface/ventilation modelling, a measured moment of inertia, a foil design
study on hardware, and hops onto disturbed water.

Implication for how to work NOW: when something is imperfect but adequate, note
it and move on. The place to fix it is Stage 2 or 3, not here.
Work proceeds incrementally — section by section, not full drafts dumped at once.

## AS-BUILT VEHICLE (authoritative — supersedes anything older)
- Mass 90 g all-up (incl. mocap markers + glue). Crazyflie **Bolt** board.
- Battery: Coddar CD4S35090HV — 4S HV LiPo, 15.2 V nominal, 350 mAh, 5.32 Wh, 90C.
- Motors: 4× Sparis 1204 4500KV. Props: Flash 2540-3, clear PC, 3-blade, T-mount.
- Geometry: 150 mm core diameter (75 mm motor-mount radius) + 70 mm hydrofoil
  horizontal span = 145 mm tip radius, 290 mm tip-to-tip.
- Hydrofoils: 70 mm radial span, 15 mm chord, 3 mm thick, β = 30°. ONLY 30° was
  ever fabricated and flown; the 5–30° sweep is simulation-only.
- Each hydrofoil is ONE integral moulded leg per motor mount: drops vertically,
  then kinks outward into the foil ("mosquito leg"). Confirmed from photos in
  `test results raw material/videos/photos/`.
- **NO drag plates.** They were removed from the design entirely. Airframe drag
  is NOT modelled and NOT a design lever — we do not need the drone to spin slower.

## YAW GENERATION (the big correction -- the paper was wrong about this)
NOT relaxed hover. NOT four same-direction props with uncancelled reaction torque.
Two additive torque sources in the same rotational sense:
1. **DOMINANT**: two HORIZONTALLY mounted rotors (tangential to the frame) driven
   at a fixed thrust setpoint (~40000/65535 PWM), commanded independently of the
   lift channel. Firmware: `powerDistributionTangentialSpin`, param
   `powerDist.spinThrust` (`mixer_type: 1` in MACHINES).
2. **SECONDARY**: the two VERTICAL lift rotors still spin the same physical
   direction, so their reaction torque ADDS to (does not cancel) the tangential
   torque. This is the old tau_z = c*F coupling, now a minor contributor.
Consequence: spin rate is a **commanded, tunable operating parameter**, not a
passive equilibrium. This is why no spin-rate ceiling was found (see below).

## FIRMWARE: FOUND AND VERIFIED (2026-09-17)
The flown source was recovered from the old machine and pushed. It is:

    branch  waterskip-fw     (tracks fork/waterskip-fw)
    tag     flown-2026-08
    commit  4293c2fe  "Tangential-spin mixer for the water-hop Bolt board"
    base    72b47d84  (safety checks disabled, explicit motor arming)

**This is the branch checked out on disk. Local `waterskip-fw` now matches
GitHub, so the name means the same thing everywhere.**

Verified against the live parameter table read off the Bolt in cfclient:
    powerDist.idleThrust  uint32  RW  Persistent:Yes   (7000)
    powerDist.mixerType   uint8   RW  Persistent:No    (0 at boot)
    powerDist.spinThrust  uint32  RW  Persistent:No    (0 at boot)
    powerDist.thrustCap   uint32  RW  Persistent:No    (65535)
The board has NO m24Thrust. This branch's parameter surface matches exactly;
no other ref in any repo does.

The mixer itself (`powerDistributionTangentialSpin`,
`src/modules/src/power_distribution_quadrotor.c`):

    m2 = m4 = spinMotorThrust            // fixed; INDEPENDENT of control->thrust
    m1 = control->thrust - r + p         // legacy M1/M3 roll/pitch coefficients
    m3 = control->thrust + r - p         // no yaw term at all

So yaw authority comes entirely from m2/m4's tangential thrust, and the spin
really is commanded independently of the lift channel. Ch.4's wording ("held
at a fixed setpoint independent of the lift channel") is exactly right, and
the spin survives the lift command being cut to zero, which is what makes the
unpowered descent possible.

The abandoned `m24Thrust` line (old tip `53eca3c2`) is preserved only as the
tag `abandoned-m24thrust`. It was never flown. `build/cf2.bin` (2026-07-26,
built from `72b47d84`) is also stale and gitignored. Reasoning from either of
those produced a string of wrong conclusions in Sept 2026; do not repeat that.

mixerType and spinThrust are NON-PERSISTENT, so they read 0 on a freshly
booted board and are set on every connection by `revolvingdario.py` (~line
813). Reading 0 in cfclient before running the flight script is expected and
is not evidence that they are unused.

To re-check the board: `conda activate cf`, then `cfclient`, Parameters tab,
`powerDist` group.

## Spin rate: what we can and cannot claim
- CAN claim: flight-tested to ~10,000 °/s with **no ceiling encountered**. The
  old paper claim of a 20π rad/s ≈ 3600 °/s ceiling (from a naive "N control
  updates per revolution" argument) and "settles at 3000 °/s" are both FALSE.
- CANNOT claim: that reducing closed-loop latency is what enabled this. Latency
  was reduced (compressed 9-byte `send_setpoint_revolving` packet, decoupled
  SetpointSender thread) but NO before/after latency data was kept. Mention
  phase lag ONLY as a brief, hedged future-work observation. Do not build a
  narrative on it or invest effort defending it. (User's explicit instruction.)

## The novel contribution (the heart of the paper)
The water-skipping hop via hydrofoils. Because water is ~700× denser than air,
the spinning hydrofoils generate little useful lift in air, but during the brief
submersion on water contact they generate substantially more lift, ejecting the
MAV back upward. This conserves a good portion of the vehicle's mechanical energy
across each hop. This locomotion mode — NOT the control law — is the original
contribution distinguishing this work from the SplitFlyer papers.

## Control mechanics: what carries over from SplitFlyer, and what does NOT
The flight control is NOT the novelty. The *fast-spin machinery* carries over:
gyroscopic reduced attitude dynamics, the non-cascaded 4th-order controller,
inertial angular-momentum formulation, input allocation. All of that only needs
ω_z to be LARGE and ROUGHLY CONSTANT — it does not care why the vehicle spins.

**Relaxed hover does NOT carry over.** SplitFlyer (and Mueller & D'Andrea) spin
because yaw torque cannot be cancelled and settle wherever residual torque
balances drag. Our vehicle spins because tangential rotors drive it, at a rate we
command. Do not describe this vehicle as relaxed-hovering, or derive an
equilibrium ω_z* from a torque/drag balance — there are no drag plates and the
balance no longer exists. Where the old text had ω_z*, the corrected treatment is
simply: ω_z is commanded, held constant, and measured.

Framing: credit SplitFlyer/Mueller for the regime and the reduced-attitude
dynamics we genuinely inherit; draw the distinction on WHY the vehicle spins,
since a settable spin rate is a design variable (it sets hydrofoil tangential
speed at water contact) rather than a constraint to work around.

## Source papers (Bai, Tan, Chirarattananon, CityU)
1. "SplitFlyer..." IROS 2020 (arXiv:2007.14862): bicopter dynamics, relaxed
   hovering condition, gyroscopic reduced attitude dynamics, cascaded controller,
   cyclic torque generation for the underactuated bicopter.
2. "SplitFlyer Air..." IEEE/ASME TMECH 2022: unified quad+bicopter framework,
   inertial-frame angular-momentum formulation, non-cascaded controller,
   mode-specific input mappings, SMA catapult undocking mechanism.

## Workspace layout
TWO project repos. Do not merge them.
  - `xuyilian/CityUHKThesis-main`  -> the thesis      (cloned at `paper/`)
  - `xuyilian/Water-skip-controller` -> ALL the code  (cloned at `code/`)

`crazyflie-firmware/` at workspace root is NOT a third project repo. It is a
working clone of upstream `bitcraze/crazyflie-firmware`, kept so the firmware
can be built. Our custom firmware is a BRANCH of the code repo, pushed to the
`fork` remote:

    origin -> bitcraze/crazyflie-firmware       (upstream, for rebasing)
    fork   -> xuyilian/Water-skip-controller    (ours: branch `waterskip-fw`)

So Water-skip-controller holds two unrelated histories on separate branches:
`main` (Python/MATLAB) and `waterskip-fw` (firmware C). That is deliberate --
all the code in one repo -- and is why `merge-base main waterskip-fw` is empty.
They are never merged; each is checked out in its own directory.
Do NOT "fix" this by splitting the firmware into its own repository.
- paper/ — local clone of the TeXPage project (paid git bridge), shared with
  collaborators editing live in the browser. TeXPage is the source of truth.
    - Main file: paper/CityUHKThesis-main/thesis.tex
    - Bibliography: paper/CityUHKThesis-main/References/references.bib
    - Figures: per-chapter, e.g. Chapters/paperone/figures/, Chapters/papertwo/figures/, etc.
    - Build: compiled in TeXPage browser — no local build needed
- code/ — the project codebase (own git repo at code/Water-skip-controller/).
  Flight controller: `revolvingdario.py`. Offline analysis: `analyse.m` +
  `analyse_prep.m`. Hop simulation: `simu.m`.
  `simu.m` was brought to as-built values on 2026-08-17 (90 g, 15 mm chord,
  β 30°, r_in 75 mm → r_tip 145 mm, ω₀ 8000 °/s) and its blade-element integral
  now runs r_in → r_tip. Ch.2's figures were regenerated from it. Still a hop
  *study*, not a spec sheet: the moment of inertia remains a placeholder (see
  "Known gaps" below). `simu.m` also now regenerates Ch.2's Fig 2.2 directly
  into the thesis Assets via `plot_ch2_hop_trajectories` (flag
  `RUN_CH2_TRAJECTORY_OVERLAY`), and prints the ranges Ch.2 quotes.
  `figs_results.m` builds the Ch.5 figures from the flight logs, styled to
  match simu.m. FIG 1 now draws four panels including spin rate.
  Three Python figure generators were added for figures that are schematics or
  annotations rather than data, all writing PDFs/JPEGs straight into
  `paper/CityUHKThesis-main/Assets/`:
    - `fig_hop_cycle.py`            -> Ch.1 Fig 1.1, the four stages of a hop
    - `fig_nonrevolving_frame.py`   -> Ch.4 Fig 4.1, the non-revolving frame
    - `fig_composite_annotate.py`   -> Ch.5 Fig 5.3, time stamps on the composite
  They need matplotlib/PIL, so run them in the `cf` conda env, not system
  python. They deliberately avoid TikZ so the thesis preamble stays untouched
  and cannot conflict with collaborators editing live in TeXPage.
- test results raw material/ — the experimental evidence for the results chapter.
    - `videos/*.MP4` — named by outcome, e.g. `C0286_2hop.MP4`,
      `C0284_stable2hop_propflyoff.MP4`, `C0264 highhopfinalday.MP4`
    - `videos/photos/DSC009*.JPG` — vehicle photos (authoritative for geometry)
    - `test_results_designs/*.mat` — the matching flight logs, same naming
  These video/log pairs are the curated notable cases. Do NOT go trawling the
  ~1400 files in DataExchange/ for results — use these.
  **EXCLUDED — do not cite or plot**: `20260730_214857_2.5hops.mat` +
  `C0228 2.5hops.MP4` (the "2.5hops"/oldbuild pair). That run used an EARLIER
  BUILD and is the one case where drop spin thrust was still enabled. The user
  is renaming it with an `oldbuild` marker. All other cases are the final build.

## Sync & collaboration safety
- The human pulls (in paper/) before each session and pushes after. Do NOT run
  `git push` / `git pull` yourself unless explicitly asked — sync is kept in the
  human's hands so collaborators' live browser edits aren't clobbered.
- Keep edits minimal and localized to reduce merge conflicts; don't reflow or
  reformat whole files unless asked.
- Never commit build artifacts to paper/ (.aux, .log, .pdf, etc.) — they're
  gitignored.
- Figures generated in code/ must be copied into paper/figures/ and committed in
  paper/ to reach TeXPage.

## How to work (important)
- One section at a time. No full-paper or full-section drafts dumped upfront.
- Always FLAG whether content is directly sourced from a reference paper vs.
  extrapolated. Any quadcopter yaw-rate regulator content is an extrapolation,
  not directly sourced — handle with caution.
- Never invent citations, hardware numbers, or results. Leave an explicit
  `% TODO(...)` placeholder instead — a fabricated reference in a submitted
  thesis is far worse than a visible gap.
- When editing `revolvingdario.py`: small, individually-testable increments only.
  A large multi-site refactor previously introduced bugs the user had to unwind
  by hand. If the user says they rewound/reverted, the on-disk state is ground
  truth — do NOT reapply the earlier changes unless asked again.
- Ground migrated theory in ACTUAL hardware geometry (from code/) before
  finalizing prose; validate against the source material.
- **Framing rule**: Present the vehicle's dynamics and properties in their own
  voice. Do NOT open paragraphs with "follows the formulation of X", "unlike X",
  or "direct counterpart of X". Cite sources for specific equations/concepts
  where academic attribution is needed, but the prose should read as describing
  this vehicle — not as a comparison or migration exercise. The reader should
  encounter our work on its own terms.

## Tools
- LaTeX, with companion .bib files.

## The descent is unpowered; the CONTACT is only partly so (corrected 2026-09-09)
The two lift rotors are commanded to EXACTLY ZERO for the whole descent, and
the descent is ARRESTED with them still off: the vehicle's downward velocity
reaches zero entirely by hydrofoil action. That is the claim to make.

Do NOT claim the rotors are off through the whole contact. With the corrected
contact boundaries, the WHOLE contact is unpowered: the lift command returns
within one sample of the foils leaving the water. See "THE FILTER-LAG TRAP"
below, which is the single most important methodological point in this
project. Superseded claims, do NOT reinstate any of them: "control restored
42 ms into the contact", "peak thrust 0.64 of hover during contact", "49% of
the reversal unpowered", "78% unpowered", "T/W below 0.35".

`drop_spin_thrust` exists in `revolvingdario.py` as a knob (added in case the
hop needed help) but was set to **0** for all cases we report — it turned out
to be unnecessary. **Do NOT mention drop spin thrust in the paper.** Write the
descent as: lift rotors off, vehicle falls unpowered, tangential rotors keep
driving the spin so the foils are at full sweep speed on arrival. This is a
STRONGER result than assisted ejection, so state it plainly. (Do NOT write
"ballistically" -- see the descent-rate gap under "Known gaps".)

Measured from the logs (final-build cases only, see EXCLUDED note above), as
tabulated in Ch.5 `tab:hop_summary`: entry −1.53 to −1.85 m/s, exit +0.57 to
+1.57 m/s across 8 contacts, mean exit +1.01. (An older note here said exit
"+0.17 to +0.92"; that never matched the table and is wrong.)
CONTACT DURATION depends on how the contact boundary is defined, so quote it
only from `figs_results.m`, never from memory. Current rule (2026-08-19):
entry = where dv/dt first exceeds 15% of its peak (the knee of the steep
rise), exit = the velocity maximum. Under that rule the single clean hop is
75 ms. An earlier rule that took entry at the velocity minimum gave 106 ms
for the same hop; it started too early, catching a noisy plateau before the
foils bit.

## THE FILTER-LAG TRAP (2026-10-01) -- read before touching any timing claim
`mocap_vz_filt` is filtered TWICE in `revolvingdario.py`: an EMA on position
(`MOCAP_Z_FILTER_ALPHA = 0.3`), then the derivative of that, then a second EMA
(`DERIV_FILTER_ALPHA = 0.5`). Combined group delay is about 3.3 samples, some
35 ms at 95 Hz, and it also ATTENUATES extrema.

`cmd_thrust` is the command as issued and has NO delay.

Comparing an event derived from `mocap_vz_filt` against `cmd_thrust` timing
therefore manufactures a ~35 ms offset that does not exist. This is exactly
how the thesis came to claim the rotors returned 42 ms into the contact. They
do not: differentiating the RAW position with a zero-phase filter
(Savitzky-Golay) puts the velocity peak of the clean hop at 22.861 s, the same
sample at which `cmd_thrust` first goes non-zero.

What survives the lag and what does not:
  - DURATION survives. Both boundaries shift together, so trough-to-peak is
    74 ms lag-free against the 75 ms reported. The thesis duration is right.
  - ABSOLUTE TIMING does not. Never compare a filtered-velocity event against
    an unfiltered one.
  - PEAK VALUES do not. The lag-free exit velocity of the clean hop is
    +1.39 m/s against the +0.92 in `tab:hop_summary`, and peak acceleration
    7.6 g against 4.9 g. The table was NOT re-cut; it is internally consistent
    but understates exit speed and peak g. Re-cutting it is the first job for
    anyone extending this work.

Correct recipe for any new timing work: raw `mocap_z_raw`; mask dropout
(repeated identical z) WITHOUT disturbing the uniform time base; zero-phase
differentiate; contact = velocity trough to the FIRST peak after it,
equivalently the interval where upward acceleration exceeds gravity. Taking a
global maximum instead of the first peak runs past the contact into the
powered climb and yields restitution > 1, which is impossible.
SPIN: hover ~8800 °/s, freefall (lift rotors off) ~8000 °/s, dropping to
~3100–3700 °/s on water contact. Measured across the five final-build flights
(2026-10-01): retention 39, 40, 43, 44, 47%, so **~40% retained (mean 43%),
about 60% of the spin spent driving the hop**. That retention is a second validation target for
`simu.m`'s ω_exit alongside contact duration and exit velocity.
The ~800 °/s hover→freefall drop is the lift rotors' reaction-torque
contribution vanishing when they are cut — empirical confirmation of the
two-source yaw mechanism in Ch.4, and worth stating in the paper.
**Methodology note**: `mocap_yawrate_deg_filt` is NOT filtered. It is written
from an EMA whose coefficient `R_LMS_YAW_RATE_ALPHA = 1.0`, which is a
pass-through, and it is byte-identical to `mocap_yawrate_deg` in all seven
logs (verified 2026-10-01). Both oscillate hard at the spin frequency, raw
~7900–12400 °/s in hover, so apply your own median filter and read values off
the traces rather than taking a median at a sample point —
pinpoint sampling underestimates. Differential readings (hover vs freefall)
are robust even where absolute values are noisy. Rates are genuine, not
aliased: raw yaw steps ~88°/sample at 95 Hz against a 17,000 °/s limit.
Consistent with Ch.2's simulated 55–185 ms and 0.2–0.5 m/s.
NOTE: `cmd_thrust` in the logs is the LIFT channel only; tangential spin thrust
is a firmware param and is NOT in the logs, so the logs alone cannot prove
drop-spin was off — that rests on the user's account.

## WHICH CASE TO USE FOR WHAT (user's decision, 2026-08-19)
- **Primary analysis case = `20260801_003920_stable2hop_propflyoff`**
  (video `C0284_stable2hop_propflyoff.MP4`). Best 2-hop demonstration; Ch.5's
  quantitative results should come from this one.
- **Second-best 2-hop = `20260731_234133_2.5fuh`** (video `C0279_2.5fuh.MP4`).
  Use for the video montage alongside stable2hop, not for the numbers.
- **Single clean hop = `20260731_191119 highhoplastday`**
  (video `C0264 highhopfinalday.MP4`). Control was not re-engaged, so it shows
  raw hop capability. Already used for Ch.5 fig.1 (`ch5fig1_single_hop.pdf`).
- Display plan: link an edited video covering stable2hop + 2.5fuh.

## Results we have, and how to frame them
Best evidence: multiple **2-hop sequences** with controlled return to set height,
plus one **clean single hop** where control was not re-engaged (shows raw hop
capability unobscured by subsequent control). Also several 3-hop attempts that
did not return to height cleanly.

Framing rule for the 3-hop gap: the cause was **mocap losing tracking at the
splash** — an instrumentation limitation of the test environment, NOT a vehicle
or control limitation. State it plainly, once, in a short honest paragraph;
mention the mitigation built for it (dropout detection + blind boost with ramped
cutoff). Do NOT over-apologise, do NOT treat it as a failed result, and do NOT
let it grow into a section. A 2-hop sequence already proves the full cycle closes
(hover → fall → contact → eject → recover under control → repeat); a third hop
demonstrates no new mechanism.

Landing into waves / disturbed water is a genuinely interesting follow-up, but
belongs in Conclusion/Future Work as 1–2 sentences only — we have no wave data.

## Chapter status (as of 2026-10-01) — STAGE 1 CLOSED
~17,900 words, 12 figures, 3 tables. No unresolved citations or
cross-references, no em-dashes, no todoboxes. All 11 bibliography entries are
cited and all cited keys resolve.

1. `introduction.tex` — Ch.1, 2977 w. Related Work carries the physics of why
   the problem is hard, not just a list. Fig 1.1 is the hop cycle.
2. `dynamic.tex` — Ch.2, 3971 w. §2.1 rewritten in a technical register, §2.2
   and §2.3 merged, the three-variable design-space study CUT, §2.5 rebuilt
   around a parameter table and a validation against the measured contacts,
   plus §2.5.1 energy accounting. Fig 2.2 is three stacked panels at the
   as-built beta = 30 deg.
3. `paperone/paperone.tex` — Ch.3, 1398 w. The invented 20*pi spin ceiling and
   the latency speculation are DELETED. One photo slot open.
4. `papertwo/papertwo.tex` — Ch.4, 4519 w. Verified line by line against the
   flown firmware 4293c2fe. Fig 4.1 is the non-revolving frame. Closes with an
   explicit "relation to prior work" that states the control law is inherited
   and is not the contribution.
5. `paperthree/paperthree.tex` — Ch.5, 3843 w. Fig 5.2 has four panels
   including spin rate; Fig 5.3 composite carries time stamps. One photo slot
   open, two `TODO(setup)` comments needing author-only facts.
6. `conclusion.tex` — Ch.6, 1151 w. The limitation is stated as "predicts the
   impulse but not the contact", which is the accurate form.

Figures: 12 placed, 3 `\fbox` placeholders (hydrofoil close-up, experimental
setup, and one in Ch.5), all safe to build.

`REVIEW-ROUND-1.md` records the supervisor's 63 annotations and how each was
resolved. It is a historical record now; nothing in it is outstanding.

Front matter still carries template content: panel names (SMITH John, ZHANG
San, DOE John, LI Si, WANG Wu), acknowledgements, publications page.
Department is set to Mechanical Engineering with a TODO to confirm as
registered. The abstract reports real numbers and the validation result.

## Known gaps / cautions
- **OPEN, UNRESOLVED: how fast does the vehicle actually fall?** Mocap says the
  unpowered descent runs at ~4 m/s², about 40% of g, consistent across 5 cases
  by quadratic fit to raw position, and the distance fallen agrees (it covers
  ~300 mm where free fall would give ~600 mm). If true, something aerodynamic
  supports over half the vehicle's weight during the drop, and "ballistic
  freefall" is the wrong phrase. BUT: the onboard `acc_z` disagrees, and that
  channel reads 0.343 g in hover where it should read 1.0, so it is unusable at
  8000 °/s. The user's own read of the footage also doubts 40% of g.
  DECIDING TEST, not yet done: count video frames from release to splash. In
  `C0284_stable2hop_propflyoff.MP4` (25 fps) the hop is at frames ~364-390 with
  contact at 374. Free fall over the ~0.49 m drop is ~8 frames; 4.3 m/s² is
  ~12 frames. Until this is settled the paper says "unpowered descent" and
  "lift rotors commanded to zero", which is true either way. Do NOT write
  "ballistic" or quote a descent acceleration anywhere.
  Ch.4 also still calls the foils "aerodynamically inert in air"; that sentence
  needs revisiting once this is settled.
- Moments of inertia have never been measured. `simu.m` uses a HOOP of the
  vehicle mass at the MOTOR-MOUNT circle: `I = m * r_hoop^2`, r_hoop = 75 mm,
  giving I = 5.06e-4 kg m^2. Changed 2026-09-24; it previously read as a
  "uniform disk" at the 145 mm tip radius, but `I = m*R^2` is a hoop not a
  disk, and 145 mm credited the vehicle with ~3.7x too much inertia. (The old
  note here said R_disk = 70 mm, which matched neither the code nor the
  geometry.) Still an explicit placeholder. Predicted spin loss scales as 1/I,
  and so does the conversion efficiency in Ch.2; the FRACTION of rotational
  energy spent, 1 - (w_exit/w_0)^2, is independent of I and is the figure to
  trust.
- No latency measurements were kept (see spin-rate section above).
- **Blade-element integrals run r_in → r_tip, NOT 0 → r_tip** (corrected
  2026-08-17). The foil does not reach the spin axis: each leg drops from its
  motor mount at r_in = 75 mm and kinks outward to r_tip = 145 mm, so the span
  is 70 mm sitting in an annulus. Integrating from 0 credits the vehicle with
  75 mm of foil it does not have, and since element force goes as (ωr)² that
  is not a small error. Fixed in `dynamic.tex` (eqs. total_thrust_c,
  total_torque_c, T/Q parameter lists, surrounding prose) and in `simu.m`
  (`rinner` threaded through run_case → simulate_water_skipping →
  compute_thrust_and_torque).
- Ch.2's simulation results were redone at as-built values (2026-08-17) and its
  figures regenerated; the chapter is complete. What remains open is that the
  model over-predicts the contact by a large factor at these sweep speeds. The
  measured hop is much longer and gentler than predicted, consistent with the
  free surface deforming rather than staying flat. Ch.2 states the flat-surface
  assumption explicitly and Ch.6 lists the discrepancy as a limitation; that is
  the honest treatment for Stage 1. Fixing it properly is Stage 3 work.
- Under consideration: swapping Ch.2 and Ch.3 so the vehicle is introduced before
  the model that sizes it (design → model → control → results reads better for a
  conference paper). Not yet decided.
