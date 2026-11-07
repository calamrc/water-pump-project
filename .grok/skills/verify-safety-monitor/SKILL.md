---
name: verify-safety-monitor
description: >
  Formally checks safety_monitor_thread in thread_manager.c. Use when a
  change touches the max runtime, the sensor stall stop, or the emergency
  latch, or when the user runs /verify-safety-monitor.
---

Once the monitor decides the pump has run too long, or the sensor has gone quiet, the pump must end off. A failed stop today sets the latch and does not try again, so the pump stays on.

The check is a TLA+ model of `safety_monitor_thread` in `app/src/thread_manager.c`. Read every result with `/statechart`. The spec is `Safety.tla` in this directory. The config is `Safety.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

## Already broken

`LatchedMeansOff` and `NoDeadlock` fail on the same state, "Stop failed, pump still on". The shortest path is the pump turns on, the sensor goes quiet, the stop call fails. `execute_emergency_stop` returned an error, `emergency_stop_active` stays set, and `pump_on` stays set. The guard `pump_on && !emergency_stop_active` then skips every later check. Nothing in that state has a next step.

`OneStartPerRun` holds. A second on-message while the pump is already on does not move `pump_on_time`. The chart hides that message because it changes nothing. A model where that message increments the start count fails `OneStartPerRun`. `RunningHasPump` holds. A scratch model that enters running with the pump still off fails it on the turn-on step.

## Bounds

`MaxTicks` is 3. One step is one check interval, not `CONFIG_SAFETY_MONITOR_MAX_RUNTIME_MINUTES`. One quiet step stands for `CONFIG_APP_SAFETY_MONITOR_MAX_SENSOR_TIMEOUT_MS`. The order of the checks matches the thread. The durations do not.

## Left out

The warning log before the limit. `pump_controller_safety_check`, which logs and does not stop the pump. `thread_health_check_all`, which can call `pump_controller_emergency_stop` without publishing `PUMP_EMERGENCY_STOP`, so this monitor can keep `pump_on` after the driver is already off. The health-label enum is not a mode machine.

## Coverage

File is `app/src/thread_manager.c`. Functions are `safety_monitor_thread` and `execute_emergency_stop`. Sync commit is `22c1d37280064eabc7f60925729d4c303c908de8`.

Move that commit forward only after you confirm the spec still describes those functions. The known failure is the latch with the pump still on. Any other path is a new bug or a modeling error. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
