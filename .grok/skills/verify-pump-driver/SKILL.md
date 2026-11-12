---
name: verify-pump-driver
description: >
  Formally checks the pump driver in pump_controller.c. Use when a change
  touches the relay, turn_on, turn_off, emergency_stop, the safety timer, or
  the transition table, or when the user runs /verify-pump-driver.
---

The relay must be energized exactly while the driver state is `RUNNING`. A stop whose relay write fails stays in `RUNNING`, and the next stop tries the coil again.

The check is a TLA+ model of the live calls in `drivers/pump/pump_controller.c`. Read every result with `/statechart`. The spec is `Pump.tla` in this directory. The config is `Pump.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

## What holds

`RelayMatchesRun` holds. On the way out of `RUNNING`, `execute_state_transition` returns the relay error before it stores the new state, and it leaves the safety timer armed. `TypeOK`, `OnlyLivePhases`, and `NoDeadlock` hold. TLC reached 3 states. The chart hides the failed coil write because that step stays in `RUNNING`.

A scratch that stores `OFF` and leaves the relay energized fails `RelayMatchesRun`. The path is a plateau start, then that stop. That copy is not the committed spec.

## What the live calls do

`pump_controller_turn_on_impl` accepts only `OFF` or `STARTING`, then submits `PUMP_EVENT_PLATEAU_DETECTED`. From `OFF` the table cell is `RUNNING`, so `STARTING` is never stored. `turn_off` and `reset` submit `PUMP_EVENT_RESET`, whose cell is `OFF` in every row. `emergency_stop` and `safety_work_handler` call `execute_state_transition` straight to `OFF`. They do not submit `PUMP_EVENT_SAFETY_TIMEOUT`. A failed relay-on write stores `ERROR` and leaves the relay off. A failed relay-off write returns before the state changes, so the phase stays `RUNNING` and the relay stays energized. `is_on` is true only in `RUNNING`. The `OFF` entry still does not drive the relay.

`TIMEOUT`, `SAFETY_TIMEOUT`, `ERROR_DETECTED`, `MAINTENANCE_ENTER`, and `MAINTENANCE_EXIT` have no caller. `TypeOK` allows `STARTING`, `TIMEOUT`, and `MAINTENANCE`. No step stores them. `OnlyLivePhases` fails if a later change stores one of them.

## Left out

The mutex drop between the mode check and `process_event` in `turn_on`. GPIO polarity. `safety_systems_active`, which init sets and this file never clears. `set_config`, which returns `-ENOTSUP`. The public header `include/app/drivers/pump_controller.h` has a different `struct pump_config` than `drivers/pump/pump_controller.h`. The device is built from the internal struct.

## Coverage

File is `drivers/pump/pump_controller.c`. Functions are `execute_state_transition`, `process_event`, `pump_controller_turn_on_impl`, `pump_controller_turn_off_impl`, `pump_controller_reset_impl`, `pump_controller_emergency_stop_impl`, `safety_work_handler`. Sync commit is `7b0abd4700b42f6a8f90e29a1b76a8c651a5c8bc`.

Move that commit forward only after you confirm the spec still describes those functions. A path that stores `OFF` while the relay write failed is a regression. Any other failing path is a new bug or a modeling error. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
