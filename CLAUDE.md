# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A Python API for controlling CubeMars/TMotor AK-series actuators (e.g. AK80-9, AK45-36) over CAN or serial from
a Linux host (typically a Raspberry Pi with a CAN hat). This is a fork of the upstream `TMotorCANControl` project
with an added personal workspace (`my_ak45/`) for AK45-36 experiments and a migration from Anaconda to `uv` for
environment management. Everything is hardware-in-the-loop: there is no simulator or mock CAN bus, so most
"testing" here means running a demo/test script against a real motor.

## Repository layout

- `src/TMotorCANControl/` — the installable package. Three independent driver modules, one per control
  protocol (see Architecture below). This is the only code that ships in the wheel/sdist.
- `src/TMotorCANControl/test/` — ad-hoc scripts used against real hardware (connection checks, step/chirp
  response tests, torque-constant derivation, log plotting). Despite the name, these are **not** a pytest
  suite — there is no automated test runner in this repo.
- `demos/` — canonical, documented example scripts for each protocol (`mit_can/`, `servo_can/`, `servo_serial/`),
  intended as copy-paste starting points and as the reference for correct API usage.
- `my_ak45/control_mit_can/` — a personal AK45-36 experimentation workspace built on top of the package: numbered
  templates (`0_template_basic.py`, `1_template_impedance.py`, `2_template_current.py`) meant to be copied into
  `experiments/exp_NNN_description.py` (currently `exp_001_gain_tuning.py` … `exp_004_trajectory.py`,
  `exp_006_thermal_baseline_check.py`, `exp_007_thermal_baseline_multi.py`; `exp_005_sysid_excitation.py` has
  since moved to `my_ak45/Mujoco/data_collection/`, see below), driven by a shared `config.yaml`. Common code
  shared by templates and experiment scripts lives in `lib/`: `config_loader.py` (resolves `config.yaml` relative
  to the module file, not `cwd`, so it works from either `control_mit_can/` or `control_mit_can/experiments/`),
  `motor_setup.py` (single/multi-motor init), `logging_utils.py` (`make_run_dir(name)` creates one
  `logs/{name}_{timestamp}/` folder per script run; `make_log_path(run_dir, filename)` resolves a file inside
  it; `console_log(run_dir)` is a context manager that tees `stdout`/`stderr` — including uncaught-exception
  tracebacks — into `run_dir/console.log` alongside the CSV(s)), `sync_logger.py` (`SyncMultiMotorLogger`,
  records multiple motors on one shared timeline/CSV — `TMotorManager_mit_can`'s own per-motor CSV logging has
  an independent `pi_time` origin per motor, which doesn't line up across motors), and `safety_monitor.py`
  (`SafetyMonitor`, cross-motor position/velocity/torque/temperature checks and `power_off()`-all emergency
  stop — not all templates/experiments use it yet; its temperature check is a backstop since
  `TMotorManager_mit_can.update()` already raises `RuntimeError` on its own `max_temp`, so callers that call
  `update()` per motor before `check()` should wrap `update()` in `try/except RuntimeError` and route into
  `trigger_emergency_stop()`, as `exp_003`/`exp_007` do). `logs/` itself is gitignored (`*.csv`/`*.log` recurse
  into the per-run subfolders). See `my_ak45/control_mit_can/README_ja.md` for the full workflow.
- `my_ak45/Mujoco/` — system-identification work using the MuJoCo sysid toolbox, split across a Raspberry Pi
  (real-hardware data capture) and a separate GPU-equipped PC (optimization), with the repo as the hand-off:
  `data_collection/exp_005_sysid_excitation.py` runs on the Pi against real motors (it reuses
  `control_mit_can/lib/` via a `sys.path` insert rather than duplicating it) and writes multi-sine excitation
  captures into `data/raw/` — unlike `control_mit_can/logs/`, `my_ak45/Mujoco/` has no `.gitignore`, so this
  data *is* tracked and pulled by the PC side. `data_collection/sysid_run_check.py` runs automatically at the end
  of `exp_005_sysid_excitation.py` (exceptions from it are swallowed to a warning, not a script failure) and
  gives a PASS/WARN/FAIL verdict across 11 checks (velocity saturation, torque linearity, sign reversal,
  frequency-response decomposition, startup transient, thermal margin, etc.); a FAIL means the run should not be
  used for sysid. 1kHz control (`dt=0.001`) is now confirmed viable on real Raspberry Pi hardware — the
  `SoftRealtimeLoop` timing report is captured into `console.log` via an explicit `del loop` (it only prints from
  `__del__`, and the loop exits via `break` rather than normal iteration, so without this it never printed), and
  the CSV's `wall_time` column (actual completion time per sample, alongside the nominal/scheduled `t`) lets
  per-sample jitter be evaluated after the fact — CSVs recorded before 2026-08-13 lack this column and fall back
  to a coarser check. Jitter judged as one-off spikes (a few samples per run exceeding ~1ms, traced to ordinary
  Linux/non-RT-OS scheduling noise, not the script) turned out to self-correct within a couple of samples and not
  accumulate, so `sysid_run_check.py`'s FAIL criterion was recalibrated from "any single spike over a fixed
  ceiling" to "sustained drift in `wall_time - t`" — occasional WARN-level spikes are expected and not a problem.
  `duration` in `config.yaml` is `10.25` s (not 10.0) to give the multi-sine excitation's startup transient
  (~0.1–0.14s of unrepresentative high-speed motion right after a cold start) room to be discarded while still
  meeting a 10.0s usable-data target. Known open issue: motor temperature trends upward (~68°C observed) across
  repeated back-to-back runs, with 75°C as the hard limit — space out consecutive captures. `docs_syid/` holds the
  sysid work plan and reference material, including a phase-3 note that the 10s captures likely need to be split
  into ~0.5–1s sub-sequences before fitting, since open-loop trajectory divergence between two nominally-identical
  runs grows large over a full 10s span. Not wired into the main package.
- `my_ak45/wire_mechanism/` — a from-scratch physics module (not just docs) modeling a planned wire/tendon-driven
  quadruped joint, developed in phases per `my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`:
  Phase A (coordinate/sign conventions) is decided; Phase B (`wire_kinematics.py` — pure-function geometry:
  `pulley_polar_from_xy`, `anchor_angle`, `included_angle`, `wire_length`, `moment_arm`, `solve_wire_geometry`)
  and Phase C (`wire_statics.py` — quasi-static `gravity_torque()` / `solve_wire_tension()`) are implemented and
  unit-tested. **The `pulley_polar_from_xy()` sign-convention doubt once tracked here is resolved** (2026-08-07,
  commit `427236c`): re-verification showed the suspected mismatch was a false positive in the comparison test's
  own hand-computed anchor coordinates, not in `pulley_polar_from_xy()` itself; the `xfail` was replaced with a
  passing regression test (`test_l_wire_matches_direct_euclidean_distance_to_pulley_xy`) — see
  `.ai/logs/2026-08-07_01_gravity-torque-sign-fix_01.md`. Phase A-2 (drive-mechanism choice: unidirectional vs.
  antagonistic) was **revisited and walked back** (2026-08-13, `drive_modes.py` — dynamics-aware comparison of
  unidirectional/spring-assisted/antagonistic modes, plus `a2_drive_mode_comparison.py` reproducing the
  documented numbers): the original "single wire suffices for ±90°" conclusion only checked the gravity-torque
  sign at zero acceleration, missing inertia, slack at range-end, and the motor speed limit — it turned out motor
  speed limit (not inertia) is the binding constraint, and A-2 is now conditional on the target swing frequency
  (undecided) — see `.ai/logs/2026-08-13_09_a2-drive-mode-reevaluation_01.md`. **Phase D is now implemented for
  the unidirectional-only case**: `pulley_placement_search.py`'s `search_unidirectional_placement()` grid-searches
  `(x, z)` (2D — antagonistic mode would need a 4D `(x1,z1,x2,z2)` search that isn't implemented, since A-2 hasn't
  settled which mode applies) and enforces the singularity (`l5_min`) and `T>=tension_min` constraints; it does
  **not** enforce wire/link non-interference or physical mountability (no link-shape spec exists yet in this
  repo) — don't treat its `feasible=True` as "buildable in real hardware". `plotting.py` renders Phase B/C curves
  and Phase D placement heatmaps via matplotlib (dev-only dependency). Tests run via `uv run pytest` (see
  Environment & commands below).
- `my_ak45/docs_mechanism/`, `my_ak45/quadruped_prep_ja.md` — Japanese-language design/planning notes for the
  wire-driven quadruped described above; `docs_mechanism/` now tracks an implementation (`wire_mechanism/`) as
  design decisions land, `quadruped_prep_ja.md` remains advisory-only.
- `my_ak45/control_mit_can/docs/`, `my_ak45/control_mit_can/docs_mit_can/` — Notion-exported personal notes on
  real-hardware bring-up (Raspberry Pi 5 + Waveshare 2-CH CAN HAT wiring/setup) and MIT-control theory/API usage.
  **Not authoritative for AK45-36 numeric specs**: the velocity/torque limits quoted in these docs disagree with
  `MIT_Params["AK45-36"]` in `mit_can.py` and with a comment in `demos/mit_can/demo_full_state_feedback_mit_can.py`
  (three mutually inconsistent values for the torque limit alone) — see the `⚠️` notes left at each location and
  `.ai/logs/2026-08-05_01_ak45-36-spec-inconsistency-flags_01.md`. None of these values have been validated against
  real hardware. `docs_mit_can/tutorial.pdf` (previously password-protected) has since been transcribed to
  `docs_mit_can/cubemars_tmotor_control_method.md` — it turns out to be a generic Open-Source Leg project tutorial
  (2022, primarily AK80-9) rather than an AK45-36 datasheet, so it does **not** resolve the V_max/T_max
  inconsistency; see `.ai/logs/2026-08-05_02_tutorial-pdf-transcribed-inconclusive_01.md`. `docs_mit_can/` also
  has `ak40-2410-1a-a1-drive-installation-instructions.md`, an official CubeMars driver-board manual — unclear
  whether the AK40-2410 board it documents is the exact board in the AK45-36, and its power-input pin table
  states polarity (`Pin1=-`/red, `Pin2=+`/black) opposite the usual red=+/black=- convention; verify by multimeter
  before wiring, don't trust the color coding. `docs_mit_can/ak45-36-firmware-and-parameters/` holds an actual
  R-Link export from a real AK45-36 unit (`45-36.McParams.McParams`/`45-36.AppParams.AppParams`, plus firmware
  binaries) — the closest thing to ground truth found so far. Cross-checking it against `MIT_Params["AK45-36"]`
  confirmed `GEAR_RATIO=36.0` and `T_max=32.0` (amps, not Nm) but showed `Kt_TMotor`/`Kt_actual=0.1206` is actually
  the firmware's `foc_current_kp` (a current-loop PI gain, not a torque constant) copied in by mistake; `V_max=30.0`
  still doesn't reproduce from the firmware's ERPM/pole/gear values. `docs_mit_can/公式基本仕様.png` is an actual
  CubeMars official spec sheet for the AK45-36 (peak torque 24 Nm, rated torque 8 Nm, rated/peak current 2A/6.5A,
  no-load speed 52 rpm output-side ≈ 5.45 rad/s, Kt 0.11 Nm/A, 14 pole pairs, 36:1 gearing) — it confirms
  `GEAR_RATIO=36.0` and puts `Kt_TMotor=0.1206` within ~10% of the real value, but shows `V_max=30.0`/Notion's
  `45.0` rad/s and `T_max=32.0`A are both far above the motor's real rated/no-load envelope (protocol encoding
  range, not a safe operating limit — don't command near these in real experiments). This prompted lowering
  `my_ak45/control_mit_can/config.yaml`'s `safety.max_velocity` from 10.0 to 6.0 rad/s. See
  `.ai/logs/2026-08-05_03_ak45-36-firmware-export-crosscheck_01.md` and
  `.ai/logs/2026-08-05_04_official-datasheet-crosscheck_01.md`. **`V_max`/`V_min` is now resolved**: real
  multi-motor hardware logs showed decoded velocity running ~5.5–6x faster than a sensor-independent
  finite-difference estimate from position, converging with the manual value and the datasheet's no-load speed;
  `MIT_Params["AK45-36"]["V_max"]`/`["V_min"]` were changed from `30.0`/`-30.0` to `6.0`/`-6.0` accordingly (see
  `.ai/logs/2026-08-11_05_v_max_correction_01.md`) — this also tightens the speed-mode RuntimeError threshold in
  `set_output_velocity_radians_per_second()`. `T_max`/`Kt_TMotor` remain unresolved; don't copy numeric AK45-36
  specs from these docs into code/config without cross-checking `mit_can.py`.
- `docs/` — Sphinx docs. `docs/source/` is the source (autodoc against the three driver modules); `docs/build/`
  is the generated, committed HTML output — regenerate it rather than hand-editing.
- `dist/` — a committed built wheel/sdist snapshot. `__pycache__/` at the repo root is also committed (legacy
  artifacts). Don't worry about matching these on every change; only rebuild `dist/` if the user asks for a
  release artifact.
- `README.md` / `README.ja.md` — user-facing API usage guide (English/Japanese); keep both in sync if the
  public API changes.
- `.github/copilot-instructions.md` — Japanese-language AI assistant instructions; this CLAUDE.md supersedes it
  but does not contradict it.

## Environment & commands

Dependency management uses `uv` (source of truth is `pyproject.toml`; `uv.lock` pins versions). `requirements.txt`
and `setup.cfg` are legacy/compatibility exports only — never edit metadata or dependencies there.

```bash
# install/sync the dev environment
uv sync

# install the package editable (alternative to uv sync, e.g. inside my_ak45 workflows)
python -m pip install -e .

# lint
ruff check .

# format
ruff format .

# regenerate requirements.txt from pyproject.toml (do this instead of hand-editing it)
uv export --no-dev --format requirements-txt -o requirements.txt

# build Sphinx docs
cd docs && make html   # Windows: make.bat html

# run the wire_mechanism unit tests (the only pytest suite in this repo)
uv run pytest -v
```

Ruff config (`pyproject.toml` `[tool.ruff]`): target `py311`, rules `E`, `F`, `I` enabled, `E501` (line length)
ignored, double-quote string style. `requires-python = ">=3.9,<3.14"` (raised from `>=3.8` for `matplotlib`
compatibility); dev dependencies are `pytest`, `ruff`, `matplotlib`.

The installable `TMotorCANControl` package itself has **no automated test suite** (no CI test job — hardware is
required). "Testing" a change to `src/TMotorCANControl/` means:
1. `ruff check .` passes.
2. The package still imports (`python -c "import TMotorCANControl"`).
3. Where feasible, the relevant script under `demos/` or `src/TMotorCANControl/test/` runs against real hardware.
   Since CI/sandboxed environments have no CAN bus or serial device attached, hardware verification usually can't
   be done here — say so explicitly rather than claiming a control-mode change was "tested."

The one exception is `my_ak45/wire_mechanism/` (pure math, no hardware): it has a real `pytest` suite under
`my_ak45/wire_mechanism/tests/`, wired up via `[tool.pytest.ini_options]` (`pythonpath = ["my_ak45"]`,
`testpaths = ["my_ak45/wire_mechanism/tests"]`) so tests import as `from wire_mechanism import wire_kinematics`.
Run it with `uv run pytest -v` before changing anything under `wire_mechanism/`; note the deliberate
`xfail(strict=True)` covering the known `pulley_polar_from_xy` sign bug mentioned above.

## Architecture

The package exposes three independent, protocol-specific manager classes — there is no shared base class between
them, so a fix in one module does not automatically apply to the others:

| Module | Class | Transport | Protocol |
|---|---|---|---|
| `mit_can.py` | `TMotorManager_mit_can` | CAN (`python-can`, socketcan) | MIT mode |
| `servo_can.py` | `TMotorManager_servo_can` | CAN (`python-can`, socketcan) | Servo (VESC-style) mode |
| `servo_serial.py` | `TMotorManager_servo_serial` | Serial (`pyserial`) | Servo mode over UART |

Each module follows the same internal shape:

- A `*_Params` dict (`MIT_Params`, `Servo_Params`, `Servo_Params_Serial`) holding per-motor-type physical
  constants (gear ratio, torque constant, position/velocity/current limits, pole pairs) and protocol constants
  (e.g. `CAN_PACKET_ID` opcodes). Adding support for a new motor variant means adding an entry here, not new code.
- A low-level `*_state` class holding raw feedback (position, velocity, current, temperature, error, acceleration)
  and a `*_command` class holding the outgoing command payload.
- A `CAN_Manager*` class implemented as a **singleton via `__new__`** (one shared `python-can` bus/`Notifier` per
  process, per protocol) — it owns raw frame encode/decode (`float_to_uint`/`uint_to_float`, buffer packing) and
  brings the `can0` interface up/down via `os.system("sudo ip link ...")`. Do not attempt to subclass these
  singletons (documented gotcha: `__init__` would run twice).
- A `motorListener`/notifier callback that routes incoming CAN frames to the right motor instance's async state.
- The public `TMotorManager_*` class: a context manager (`__enter__`/`__exit__`) that powers the motor on/off and
  is the only supported way to drive a motor — control code should always run inside a `with TMotorManager_*(...) as dev:` block so the motor is guaranteed to power off on exception or exit. `TMotorManager_mit_can.__enter__()`
  retries its connection check up to 3 times (0.5s apart) before raising — observed on real hardware, a motor
  that hasn't received any CAN traffic yet ("cold") is slow to answer the first control-mode-start command, so a
  first-run failure right after a fresh `ip link up` is expected and self-heals on retry rather than indicating a
  wiring problem.
- An internal `_TMotorManState` (or `_TMotorManState_Servo` / `SERVO_SERIAL_CONTROL_STATE`) enum tracks which
  control mode is currently active (IDLE / IMPEDANCE / CURRENT / FULL_STATE / SPEED, or servo's
  duty/current/RPM/position/position-velocity modes); `update()` asserts the manager is in a mode consistent with
  whichever `set_*`/`enter_*_control` call was last made, and raises if state limits (position/velocity/current
  thresholds derived from `*_Params`) are exceeded.

Unit conventions to preserve when touching getters/setters: `output_*` values are post-gearbox (user/joint side);
`motor_*` values are pre-gearbox (motor side). Torque/current conversions go through `Kt`/`GEAR_RATIO`/
`Current_Factor` from the relevant `*_Params` entry — don't hardcode conversion factors inline.

`TMotorManager_mit_can`/`_servo_can`/`_servo_serial` all support optional CSV logging via a `log_vars` list and a
`LOG_FUNCTIONS`/getter dispatch table; if you add a new loggable quantity, add both the getter and its entry in
that table.

## Conventions for changes

- Keep API compatibility across the three manager classes and with the demo scripts in `demos/` — those scripts
  are the primary usage documentation and are referenced from `README.md`/ReadTheDocs.
- Make control-mode / command-state-machine changes narrowly; these directly affect motor safety behavior
  (thermal limits, position/velocity/current thresholds, power-on/off sequencing).
- Recent docstrings in `mit_can.py`/`servo_can.py`/`servo_serial.py` have been translated to Japanese; match the
  surrounding language/style when editing nearby docstrings rather than mixing languages within one docstring.
- If public API or examples change, update `README.md`, `README.ja.md`, and `docs/source/` together.
- This targets headless Raspberry Pi/Linux CAN and serial setups — don't introduce GUI dependencies.

## Change log rule (important)

After completing a feature addition, bug fix, or refactoring, write a change-log entry at
`.ai/logs/YYYY-MM-DD_XX_<slug>_YY.md`, following the format defined in
[`.claude/rules/change-log-format.md`](.claude/rules/change-log-format.md). Check
[`.ai/logs/`](.ai/logs/) for past entries to follow as examples.

Required sections: leading metadata (date/time, target files, change type) / design decisions and rationale
(alternatives considered, why they were rejected, trade-offs) / unresolved and known issues / test status
(checklist).
