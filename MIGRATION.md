# What to carry to the IROS repository

Dependency closure traced 2026-10-15 by parsing imports and grepping call
sites, not from memory. The whole working set is **361 KB**; this repository is
1.5 GB, nearly all of it data and git history.

---

## 1. `revolvingdario.py` — the flight controller

Transitive import closure. These seven files are all of it:

    revolvingdario.py
    RisLib/cflog.py                       # LoggingCore
    RisLib/savemat.py                     # DataSaver
    rislab_lib/Joystick/joystick_3.py     # PygameJoystick
    jumping/jumping_estimator_v2.py       # RiseDetect, UdpRigidBodies3,
                                          #   RealTimeProcessor, BiController,
                                          #   JoyBiController, FittedBiController,
                                          #   RProjectionLmsEstimator
    jumping/jumping_model.py              # RealTimeSleeper, saturation
    Library/IIR2Filter/IIR2Filter.py      # pulled in by jumping_estimator_v2

Everything else under `RisLib/`, `rislab_lib/`, `jumping/` and `Library/` is
unreachable from this entry point. `jumping_estimator_v3.py` in particular is
NOT used; `v2` is.

Sibling-import layout matters: `revolvingdario.py` must sit at the repository
root with `RisLib/`, `rislab_lib/`, `jumping/` and `Library/` beside it, or the
imports break.

Third-party: `cflib`, `numpy`, `scipy`, `pygame`, `tqdm`.

### The one thing that will silently break flight

`cflib` is **hand-patched**. `revolvingdario.py` calls
`cf.commander.send_setpoint_revolving(...)`, which does not exist in any
released cflib. The patch adds one method to `cflib/crazyflie/commander.py`
and nothing else; verified 2026-10-15 that this is the only modified file.

The method is saved here as `cflib_patch/send_setpoint_revolving.py`. Paste it
into `class Commander` after installing cflib. It packs `<BhhhH` with
`TYPE_REVOLVING = 12`, which matches `revolvingType = 12` in the firmware's
`crtp_commander_generic.c`.

**A pip upgrade of cflib removes the patch and the vehicle stops responding to
setpoints, with no error at import time.** Pin the version and re-apply after
any reinstall.

---

## 2. `analyse.m` / `analyse_prep.m` — offline analysis

    analyse.m
    analyse_prep.m

`analyse.m` calls `analyse_prep` to build `t`, `dt`, `Fs` and crop the record;
each section bootstraps it if the workspace is empty. Both expect a log to be
loaded first:

    load('DataExchange/20260713_140141.mat'); analyse

No other files, no `addpath`. `analyse.m` contains the onboard-versus-mocap
vertical acceleration comparison, which is the evidence that the IMU is
unusable at these spin rates.

---

## 3. `figs_results.m`, `simu.m` — the figure producers

Both are self-contained single files (16 and 17 local functions). `simu.m` is
pure simulation and reads nothing. `figs_results.m` expects a curated flight
log already loaded into the workspace:

    clear; load('20260801_003920_stable2hop_propflyoff.mat'); figs_results

The three Python figure generators are independent of all the above and need
only matplotlib and PIL:

    fig_hop_cycle.py  fig_nonrevolving_frame.py  fig_composite_annotate.py

### MATLAB requirements

**No toolboxes.** Everything used is base MATLAB: `movmean`, `movmedian`,
`gradient`, `cumtrapz`, `tiledlayout`, `nexttile`, `xline`, `yline`,
`exportgraphics`, `savefig`. Minimum release **R2020a**, set by
`exportgraphics`.

---

## 4. What to leave behind

| | Size | Why |
|---|---|---|
| `DataExchange/` | 932 MB, 1341 files tracked | The raw dump. The curated pairs are the evidence; CLAUDE.md says explicitly not to trawl this |
| `.git/` | 517 MB | Almost entirely DataExchange history. Start the new repo clean |
| `archive/` | 17 MB | Inherited scripts from the legged-jumping project; `revolvinglian.py` is the ancestor of `revolvingdario.py`, kept for provenance only |
| `cache/`, `tools/` | 1.6 MB | Not reachable from any entry point |
| unused `RisLib`/`rislab_lib`/`jumping`/`Library` modules | ~2 MB | 56 tracked files, 5 of which are used |

Do not `git clone` this repository into the new one. Copy the working set.

---

## 5. Data

The curated evidence lives **outside** this repository, at the workspace root:

    test results raw material/test_results_designs/*.mat     7.2 MB, 7 logs
    test results raw material/videos/*.MP4                   3.1 GB

Six logs are the final build. **Exclude `20260730_214857_2.5hops_olderbuild`**:
earlier build, and the only case with drop spin thrust enabled.

Take the 7.2 MB of logs. Leave the videos where they are and reference them;
only `C0284` (composite) and `C0264` (single clean hop) matter for a paper.

---

## 6. Copy command

    NEW=../iros-waterskip
    mkdir -p $NEW/{RisLib,rislab_lib/Joystick,jumping,Library/IIR2Filter,cflib_patch,data}
    cp revolvingdario.py analyse.m analyse_prep.m figs_results.m simu.m \
       fig_hop_cycle.py fig_nonrevolving_frame.py fig_composite_annotate.py \
       CLAUDE.md HANDOFF.md MIGRATION.md $NEW/
    cp RisLib/cflog.py RisLib/savemat.py                     $NEW/RisLib/
    cp rislab_lib/Joystick/joystick_3.py                     $NEW/rislab_lib/Joystick/
    cp jumping/jumping_estimator_v2.py jumping/jumping_model.py $NEW/jumping/
    cp Library/IIR2Filter/IIR2Filter.py                      $NEW/Library/IIR2Filter/
    cp cflib_patch/send_setpoint_revolving.py                $NEW/cflib_patch/
    cp "../../test results raw material/test_results_designs/"*.mat $NEW/data/

Python packages need `__init__.py` only if you convert them from namespace
packages; as laid out above they work as-is on Python 3.

The firmware is a separate concern: it is branch `waterskip-fw` of this same
repository, tag `flown-2026-08`, commit `4293c2fe`. Do not copy the source
into the new repo; reference the tag.
