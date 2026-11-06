---
name: verify-timer-countdown
description: >
  Formally checks the countdown in timer_state_machine.c. Use when a change
  touches timer_sm_update, pause, resume, start, reset, or the tick, or when
  the user runs /verify-timer-countdown.
---

A pause or a reset must freeze the seconds that are left. If the tick writes after that, the screen can say paused while the countdown ends, and the pump can run past the time the user set.

The check is a TLA+ model of `timer_sm_start`, `timer_sm_pause`, `timer_sm_resume`, `timer_sm_reset`, `timer_sm_adjust_time`, and `timer_sm_update`. Read every result with `/statechart`. The spec is `Timer.tla` in this directory. The config is `Timer.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

`TickArm` is the `TIMER_STATE_RUNNING` test in `timer_sm_update`. `TickCommit` is the write of `remaining_seconds` and, at zero, `TIMER_STATE_COMPLETED`. Those are separate steps because `timer_tick_work_handler` and the UI thread do not share a lock.

## Already broken

`OwedOnlyWhileRunning` and `PauseDoesNotCorrupt` fail on the current code. The shortest path is start, a second passes, pause. The tick has passed the `RUNNING` test. Pause does not cancel the write. The write then stores the new remaining value, and it stores `COMPLETED` when that value is 0. `CompletedAtZero` holds. `NoDeadlock` holds.

The same stuck write can still run after reset. Reset does not clear a tick that has already passed the test.

## Bounds

`MinTime` is 1, `MaxTime` is 3, and `DefaultTime` is 2. The firmware clamps to 10..3600 and defaults to 60 (`TIMER_MIN_SECONDS`, `TIMER_MAX_SECONDS`, `TIMER_DEFAULT_SECONDS`). The clamp shape is the same. The model steps by 1. The firmware steps by `TIMER_STEP_SECONDS` (10).

## Left out

Zbus publish timeouts. A caller of `timer_sm_stop_tick` other than pause, reset, and completion. The dial actions are in the model and the chart hides them, because they do not change the phase. The chart also hides every press after the countdown has already moved, except reset.

## Coverage

Files are `app/src/timer/timer_state_machine.c` and `app/include/timer/timer_state_machine.h`. Functions are `timer_sm_update`, `timer_sm_pause`, `timer_sm_resume`, `timer_sm_start`, `timer_sm_reset`, `timer_sm_adjust_time`. Sync commit is `22c1d37280064eabc7f60925729d4c303c908de8`.

Move that commit forward only after you confirm the spec still describes those functions. A path that is not the pause-during-tick path is a new bug or a modeling error. A modeling error is a step the code cannot take. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
