---
name: verify-safety-monitor
description: >
  Formally checks safety_monitor_thread in thread_manager.c. Use when a
  change touches the max runtime, the sensor stall stop, or the emergency
  latch, or when the user runs /verify-safety-monitor.
---

Once the monitor decides the pump has run too long, or the sensor has gone quiet, a successful stop leaves the pump off. A failed stop leaves the latch clear, so the next check interval tries again.

The check is a TLA+ model of `safety_monitor_thread` in `app/src/thread_manager.c`. Read every result with `/statechart`. The spec is `Safety.tla` in this directory. The config is `Safety.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

## What holds

`TypeOK`, `OneStartPerRun`, `LatchedMeansOff`, `RunningHasPump`, and `NoDeadlock` hold. TLC reached 16 states. A failed stop is `StopFails`. It changes nothing, so the chart does not draw it. The latch is set only in the success arm of `execute_emergency_stop`, on the runtime limit, the never-saw-data timeout, and the stall timeout.

A scratch where `StopFails` sets the latch and leaves the pump on fails `LatchedMeansOff` and `NoDeadlock`. The path is the pump turns on, the sensor goes quiet, the stop call fails. That copy is not the committed spec. `OneStartPerRun` holds because a second on-message while the pump is already on does not move `pump_on_time`. The chart hides that message because it changes nothing. A model where that message increments the start count fails `OneStartPerRun`. `RunningHasPump` holds. A scratch that enters running with the pump still off fails it on the turn-on step.

## Bounds

`MaxTicks` is 3. One step is one check interval, not `CONFIG_SAFETY_MONITOR_MAX_RUNTIME_MINUTES`. One quiet step stands for `CONFIG_APP_SAFETY_MONITOR_MAX_SENSOR_TIMEOUT_MS`. The order of the checks matches the thread. The durations do not.

## Left out

The warning log before the limit. `pump_controller_safety_check`, which logs and does not stop the pump. `thread_health_check_all`, which can call `pump_controller_emergency_stop` without publishing `PUMP_EMERGENCY_STOP`, so this monitor can keep `pump_on` after the driver is already off. The health-label enum is not a mode machine.

## Coverage

File is `app/src/thread_manager.c`. Functions are `safety_monitor_thread` and `execute_emergency_stop`. Sync commit is `1bf8a262654813efbb6e496fae514b952398aa8f`.

Move that commit forward only after you confirm the spec still describes those functions. A path that latches while the pump is still on is a regression. Any other failing path is a new bug or a modeling error. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
