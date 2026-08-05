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
  `experiments/exp_NNN_description.py`, driven by a shared `config.yaml`, logging CSVs to `logs/` (gitignored).
  Common code shared by templates and experiment scripts lives in `lib/`: `config_loader.py` (resolves
  `config.yaml` relative to the module file, not `cwd`, so it works from either `control_mit_can/` or
  `control_mit_can/experiments/`), `motor_setup.py` (single/multi-motor init), `logging_utils.py` (log path
  naming, `SoftRealtimeLoop` control-loop setup), `sync_logger.py` (`SyncMultiMotorLogger`, records multiple
  motors on one shared timeline/CSV — `TMotorManager_mit_can`'s own per-motor CSV logging has an independent
  `pi_time` origin per motor, which doesn't line up across motors), and `safety_monitor.py` (`SafetyMonitor`,
  cross-motor position/velocity/torque limit checks and `power_off()`-all emergency stop — not all
  templates/experiments use it yet). See `my_ak45/control_mit_can/README_ja.md` for the full workflow.
- `my_ak45/Mujoco/` — separate, early-stage system-identification work using MuJoCo; not wired into the main
  package.
- `my_ak45/docs_mechanism/`, `my_ak45/quadruped_prep_ja.md` — Japanese-language design notes for a planned
  wire-driven quadruped built on this stack; advisory/planning documents only, nothing here is implemented yet.
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
  `.ai/logs/2026-08-05_04_official-datasheet-crosscheck_01.md`. Don't copy numeric AK45-36 specs from these
  docs into code/config without cross-checking `mit_can.py`.
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
```

Ruff config (`pyproject.toml` `[tool.ruff]`): target `py311`, rules `E`, `F`, `I` enabled, `E501` (line length)
ignored, double-quote string style.

There is no automated test suite (no pytest, no CI test job). "Testing" a change means:
1. `ruff check .` passes.
2. The package still imports (`python -c "import TMotorCANControl"`).
3. Where feasible, the relevant script under `demos/` or `src/TMotorCANControl/test/` runs against real hardware.
   Since CI/sandboxed environments have no CAN bus or serial device attached, hardware verification usually can't
   be done here — say so explicitly rather than claiming a control-mode change was "tested."

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
  is the only supported way to drive a motor — control code should always run inside a `with TMotorManager_*(...) as dev:` block so the motor is guaranteed to power off on exception or exit.
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
