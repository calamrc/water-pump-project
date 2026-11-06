---
name: verify-button-press
description: >
  Formally checks the button press classes in input_manager.c. Use when a
  change touches the press timers, key up, or the published press class, or
  when the user runs /verify-button-press.
---

One physical press must publish only one class. Hold is sent at three seconds. Letting go after that must not also publish short or long. If it does, the UI opens the restart question and the same press starts or stops the timer.

The check is a TLA+ model of `input_callback`, the two timers, and `input_publish_work_handler` in `app/src/input/input_manager.c`. Read every result with `/statechart`. The spec is `Button.tla` in this directory. The config is `Button.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

## What holds

`QuietAfterHold` holds. `NoDeadlock` holds. Key down clears the pending bits and starts both timers. The one-second timer sets the long phase only while the key is still down. The three-second timer can fire from the short phase, because it does not read `long_press_pending`. Key up emits long or short only when `hold_press_pending` is clear (`input_manager.c` release guard). The publisher ranks hold, then long, then short.

A broken release that emits short after hold fails `QuietAfterHold`. The path is press, three seconds, let go.

## Left out

`input_enabled`. The UI turns it off and on around the restart dialog draw, and the timers return immediately while it is off. The encoder. The model assumes the flag stays on.

## Coverage

Files are `app/src/input/input_manager.c` and `app/include/input/input_manager.h`. Functions are `input_callback`, `long_press_timer_handler`, `hold_press_timer_handler`, `input_publish_work_handler`. Sync commit is `22c1d37280064eabc7f60925729d4c303c908de8`.

Move that commit forward only after you confirm the spec still describes those functions. A new counterexample is a new bug or a modeling error. A modeling error is a step the code cannot take. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
