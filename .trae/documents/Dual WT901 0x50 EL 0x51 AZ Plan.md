# Dual WT901 0x50 EL + 0x51 AZ Plan

## Summary
- Goal: enable dual WT901 sensors on one RS485 bus with fixed roles:
  - `0x51` = AZ source (yaw/compass fused azimuth)
  - `0x50` = EL source (roll-to-elevation mapping)
- Scope: implement in both runtimes:
  - `src/motorPID/main/az_el_controller.py`
  - `src/motorPID/main/fix-compas.py`
- Startup policy: **no automatic zero-reset**.
- Failure policy: **fail fast** if either address is unreadable.

## Current State Analysis
- `az_el_controller.py` currently uses one `WT901AxisReader` instance (`addr=0x50`) for both AZ and EL.
- `WT901AxisReader.read()` computes both outputs from a single packet:
  - AZ from yaw/compass blending
  - EL from roll mapping
- `main()` in `az_el_controller.py` currently performs `wt.reset_zero_point()` at startup.
- `fix-compas.py` currently uses one sensor (`ADDR=0x50`) and prints AZ/EL from the same packet.
- Existing helper scripts now exist:
  - `set_wt901_address.py` (write address)
  - `check_wt901_address.py` (scan addresses)

## Proposed Changes

### 1) `az_el_controller.py`: Split sensor responsibilities by address
- Add a dual-reader composition for AZ/EL, preserving current output schema used by controllers.
- New structure:
  - `wt_az = WT901AxisReader(label="AZ", addr=0x51, ...)`
  - `wt_el = WT901AxisReader(label="EL", addr=0x50, ...)`
- Add a thin adapter class (or helper function) that returns a merged packet:
  - AZ fields from `wt_az.read_with_retry(...)`:
    - `az`, `yaw_cw`, `compass_cw`, `src`, and optionally `roll/pitch` diagnostics from AZ sensor
  - EL fields from `wt_el.read_with_retry(...)`:
    - `el`, `el_roll`
- Update controller constructors to consume this merged source instead of single-reader `wt`.
- Update startup sync:
  - motor AZ sync from `merged["az"]`
  - motor EL sync from `merged["el"]`
- Remove automatic startup `reset_zero_point()` call in rotctl/target runtime path.

### 2) `az_el_controller.py`: Fail-fast startup validation
- Before control loop starts:
  - open both readers
  - attempt retry-read on each address
  - if either fails, raise startup error and stop run
- Error message must explicitly include failing address (`0x50` or `0x51`) for troubleshooting.

### 3) `fix-compas.py`: Add dual-address mode with fixed roles
- Replace single-device flow with two device-model readers (or reuse the same reader class pattern as controller):
  - AZ reader at `0x51` for blended azimuth
  - EL reader at `0x50` for roll-to-EL mapping
- No auto reset at startup.
- Terminal output remains same columns, but values come from different addresses:
  - `AZ/SRC/YAW/COMPASS` from `0x51`
  - `EL` from `0x50` roll mapping
- If either side missing, fail fast with clear message.

### 4) Runtime behavior and logging
- Add startup info logs:
  - `AZ sensor addr=0x51`, `EL sensor addr=0x50`
  - `zero_reset=disabled`
- Add periodic debug fields (existing logs) to show source split clearly:
  - include `az_addr`/`el_addr` in startup logs only (avoid noisy per-loop logs).

### 5) Compatibility and CLI
- Keep current control and rotctl interfaces unchanged (no mandatory new flags).
- Optional future extension (not in this scope): make addresses configurable via CLI.

## Assumptions & Decisions
- Decision: implement in **both** `az_el_controller.py` and `fix-compas.py`.
- Decision: **no automatic zero reset** during startup.
- Decision: **fail fast** if either address is unavailable.
- Assumption: physical bus already has unique addresses programmed (`0x50`, `0x51`) and stable wiring.
- Assumption: one process owns `/dev/ttyUSB0` at runtime.

## Data Flow (Post-change)
- RS485 `/dev/ttyUSB0` shared bus
  - Poll `0x51` -> compute fused AZ packet
  - Poll `0x50` -> compute EL from roll
  - Merge into one control packet `{az, el, src, yaw_cw, compass_cw, el_roll, ...}`
  - Controllers consume merged packet exactly as before (minimal surface change).

## Edge Cases / Failure Modes
- Address collision (both devices same addr): startup read for one role fails/inconsistent -> fail fast.
- Only one sensor connected: fail fast with explicit role/address error.
- Temporary read timeout:
  - retry policy preserved (`read_with_retry`)
  - startup still fails if retries exhausted.
- Sensor drift while stopped:
  - existing hold-follow/stop behavior remains unchanged; only data source split changes.

## Verification Steps
- Static checks:
  - `python -m py_compile src/motorPID/main/az_el_controller.py`
  - `python -m py_compile src/motorPID/main/fix-compas.py`
- Functional checks:
  1. Run `fix-compas.py` with both sensors connected:
     - confirm AZ reacts to `0x51`, EL to `0x50`.
  2. Run `az_el_controller.py --mode rotctl ...`:
     - startup logs show dual addresses and reset disabled.
     - fail-fast when disconnecting one sensor.
  3. Disconnect `0x51` only:
     - AZ role fails at startup with explicit address error.
  4. Disconnect `0x50` only:
     - EL role fails at startup with explicit address error.

## Out of Scope
- Auto address assignment logic in runtime controllers.
- Changing PID tuning logic unrelated to dual-address acquisition.
- Modifying service unit behavior beyond current launcher compatibility.
