---
name: verify-flow-sensor
description: >
  Formally checks the YF-S201C mode enum in yf_s201c.c. Use when a change
  touches the sensor state, a pulse, reset, get_flow_rate, or is_data_valid,
  or when the user runs /verify-flow-sensor.
---

A flow reading must not stay available after the sensor is in `ERROR`. The error arm clears `period_us` in the same locked update that resets the sample buffer.

The check is a TLA+ model of the mode changes in `drivers/sensor/yf_s201c/yf_s201c.c`. Read every result with `/statechart`. The spec is `Sensor.tla` in this directory. The config is `Sensor.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

## Already broken

`NoPositiveRateInError` fails. The shortest path is a valid pulse, one invalid pulse, and a second invalid pulse. The mode becomes `ERROR` and `rateKept` stays true. The error arm in `yf_s201c_work_handler` stores `YF_S201C_ERROR` and does not clear `period_us`. `yf_s201c_is_data_valid` returns false for `INIT` and for `ERROR` by its own mode test. That test is not a second TLC rule. Copying it into the spec cannot fail. `NoDeadlock` holds.

## Bounds

`Threshold` is 2. The firmware reads `consecutive_invalid_threshold` from devicetree. Any threshold of at least 1 has the same shape. One invalid pulse is its own chart state. The arrow into the fault is the pulse that reaches the threshold.

The model starts in `READY`. It does not include `INIT`.

## Left out

The boot race where the interrupt is enabled before init stores `READY`, so a sample can set `RUNNING` or `ERROR` and init can then overwrite that with `READY`. Stale time. The median filter. `get_flow_rate` overflow. The fixed `period_buffer` length of 5 versus `buffer_size` from devicetree.

## Coverage

Files are `drivers/sensor/yf_s201c/yf_s201c.c` and `drivers/sensor/yf_s201c/yf_s201c.h`. Functions are `yf_s201c_work_handler`, `yf_s201c_reset`, `yf_s201c_get_flow_rate`, `yf_s201c_is_data_valid`. Sync commit is `22c1d37280064eabc7f60925729d4c303c908de8`.

Move that commit forward only after you confirm the spec still describes those functions. The known failure is a positive period left behind in `ERROR`. Any other path is a new bug or a modeling error. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
