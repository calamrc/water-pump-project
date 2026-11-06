---
name: verify-pump-driver
description: >
  Formally checks the pump driver in pump_controller.c. Use when a change
  touches the relay, turn_on, turn_off, emergency_stop, the safety timer, or
  the transition table, or when the user runs /verify-pump-driver.
---

The relay must be energized exactly while the driver state is `RUNNING`. If a stop stores `OFF` while the relay write failed, the pump keeps running and the state says it is off.

The check is a TLA+ model of the live calls in `drivers/pump/pump_controller.c`. Read every result with `/statechart`. The spec is `Pump.tla` in this directory. The config is `Pump.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

## Already broken

`RelayMatchesRun` fails. The shortest path is a plateau start, then the stop whose relay write fails. `execute_state_transition` drives the relay off in the `RUNNING` exit and then stores the new state even when `set_relay_state` returns an error. The `OFF` entry does not drive the relay. `TypeOK`, `OnlyLivePhases`, and `NoDeadlock` hold.

## What the live calls do

`pump_controller_turn_on_impl` accepts only `OFF` or `STARTING`, then submits `PUMP_EVENT_PLATEAU_DETECTED`. From `OFF` the table cell is `RUNNING`, so `STARTING` is never stored. `turn_off` and `reset` submit `PUMP_EVENT_RESET`, whose cell is `OFF` in every row. `emergency_stop` and `safety_work_handler` call `execute_state_transition` straight to `OFF`. They do not submit `PUMP_EVENT_SAFETY_TIMEOUT`. A failed relay-on write stores `ERROR` and leaves the relay off. `is_on` is true only in `RUNNING`.

`TIMEOUT`, `SAFETY_TIMEOUT`, `ERROR_DETECTED`, `MAINTENANCE_ENTER`, and `MAINTENANCE_EXIT` have no caller. `TypeOK` allows `STARTING`, `TIMEOUT`, and `MAINTENANCE`. No step stores them. `OnlyLivePhases` fails if a later change stores one of them.

## Left out

The mutex drop between the mode check and `process_event` in `turn_on`. GPIO polarity. `safety_systems_active`, which init sets and this file never clears. `set_config`, which returns `-ENOTSUP`. The public header `include/app/drivers/pump_controller.h` has a different `struct pump_config` than `drivers/pump/pump_controller.h`. The device is built from the internal struct.

## Coverage

File is `drivers/pump/pump_controller.c`. Functions are `execute_state_transition`, `process_event`, `pump_controller_turn_on_impl`, `pump_controller_turn_off_impl`, `pump_controller_reset_impl`, `pump_controller_emergency_stop_impl`, `safety_work_handler`. Sync commit is `22c1d37280064eabc7f60925729d4c303c908de8`.

Move that commit forward only after you confirm the spec still describes those functions. The known failure is the stuck relay on the way to `OFF`. Any other path is a new bug or a modeling error. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
