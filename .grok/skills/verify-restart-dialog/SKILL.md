---
name: verify-restart-dialog
description: >
  Formally checks the restart question in ui_manager.c. Use when a change
  touches handle_input_event, the restart dialog, or a button press while
  that dialog is open, or when the user runs /verify-restart-dialog.
---

While the restart question is on screen, a short press must not start, pause, or reset the timer. Yes reboots the controller. The countdown keeps running behind the question.

The check is a TLA+ model of `handle_input_event` in `app/src/ui_manager.c`. Read every result with `/statechart`. The spec is `Restart.tla` in this directory. The config is `Restart.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

The chart has two regions. Question is the dialog. Timer is only enough of the countdown to show that the question does not own it. The full countdown is `/verify-timer-countdown`.

## What holds

`DialogDoesNotEditTimer` holds. `edited` becomes true when a step changes the timer while the question is open, except when a running countdown finishes. Cancel calls `timer_sm_get_time` to redraw the main screen, then returns. It does not call start, pause, resume, reset, or adjust. A cancel that also resets the timer fails this rule. `NoDeadlock` holds because reboot has a step that changes nothing.

Hold opens the question on Yes. The encoder flips Yes and No. Short on Yes reboots. Short on No closes the question. The tick can finish the countdown while the question is open. The chart hides that do-nothing reboot step.

## Left out

`timer_sm_reset` from the main screen. The real seconds. Pump drawing. The brief `input_manager_enable(false)` around the dialog draw. Reboot does not turn the pump off.

## Coverage

File is `app/src/ui_manager.c`. Function is `handle_input_event`. The timer calls it must not make while the question is open are `timer_sm_start`, `timer_sm_pause`, `timer_sm_resume`, `timer_sm_reset`, and `timer_sm_adjust_time`. Sync commit is `22c1d37280064eabc7f60925729d4c303c908de8`.

Move that commit forward only after you confirm the spec still describes that function. A new counterexample is a new bug or a modeling error. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
