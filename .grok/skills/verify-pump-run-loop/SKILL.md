---
name: verify-pump-run-loop
description: >
  Formally checks the on and off decisions in pump_controller_thread. Use when a
  change touches the sensor timeout, the two-second plateau lockout, or which
  period is stored, or when the user runs /verify-pump-run-loop.
---

For two seconds after this thread turns the pump off because sensor data stopped, a plateau must not start it again. A period shorter than the period that started the run must not replace the stored period.

The check is a TLA+ model of `pump_controller_thread` in `app/src/thread_manager.c`. Read every result with `/statechart`. The spec is `RunLoop.tla` in this directory. The config is `RunLoop.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

## What holds

`ShutMeansOff` holds. The timeout stores `pump_off_time`. A plateau with `elapsed_ms < 2000` returns without clearing that stamp and without calling `pump_controller_turn_on`. The chart shows that ignored start as its own state. A model that turns the pump on from that state fails `ShutMeansOff`.

`NoShrink` holds. While the pump is on, a period shorter than `initial_plateau_period` does not replace `latest_plateau_period`. The chart hides that step because the label does not change. A scratch model that stores the shorter period fails `NoShrink`. `NoDeadlock` holds.

## Left out

The queue drain after the timeout. The two-second guard is what the model checks. Emergency stop and the health-check stop do not set `pump_off_time`, so the next plateau may start the pump at once. The real timeout is `latest_plateau_period * 15 / 10`, capped by `CONFIG_APP_MAX_TIMEOUT_US`. The model uses one timeout step. Periods are only "short" and "long". The flow-analyzer confirm count is not this machine. It is the event this thread calls a plateau.

## Coverage

File is `app/src/thread_manager.c`. Function is `pump_controller_thread`. Sync commit is `22c1d37280064eabc7f60925729d4c303c908de8`.

Move that commit forward only after you confirm the spec still describes that function. A new counterexample is a new bug or a modeling error. A modeling error is a step the code cannot take. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
