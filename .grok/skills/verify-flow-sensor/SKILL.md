---
name: verify-flow-sensor
description: >
  Formally checks the YF-S201C mode enum in yf_s201c.c. Use when a change
  touches the sensor state, a pulse, reset, get_flow_rate, or is_data_valid,
  or when the user runs /verify-flow-sensor.
---

A flow reading must not stay available after the sensor is in `ERROR`. The error arm clears `period_us` while it holds the sample lock, and then it stores `ERROR`.

The check is a TLA+ model of the mode changes in `drivers/sensor/yf_s201c/yf_s201c.c`. Read every result with `/statechart`. The spec is `Sensor.tla` in this directory. The config is `Sensor.cfg`. The view is `view.mjs`. `chart.mmd` is the last drawing. It is not the proof. Regenerate it.

## What holds

`TypeOK`, `NoPositiveRateInError`, and `NoDeadlock` hold. TLC reached 5 states. The error arm sets `period_us` to 0 before it stores `YF_S201C_ERROR`. `yf_s201c_get_flow_rate` still refuses only `INIT`. With the period cleared, that call returns 0. `yf_s201c_is_data_valid` still returns false for `ERROR` by its own mode test. That test is not a second TLC rule.

A scratch that leaves `rateKept` unchanged when the invalid count reaches the threshold fails `NoPositiveRateInError`. The path is a valid pulse, one invalid pulse, and the invalid pulse that reaches the threshold. That copy is not the committed spec.

## Bounds

`Threshold` is 2. The firmware reads `consecutive_invalid_threshold` from devicetree. Any threshold of at least 1 has the same shape. One invalid pulse is its own chart state. The arrow into the fault is the pulse that reaches the threshold.

The model starts in `READY`. It does not include `INIT`.

## Left out

The boot race where the interrupt is enabled before init stores `READY`, so a sample can set `RUNNING` or `ERROR` and init can then overwrite that with `READY`. Stale time. The median filter. `get_flow_rate` overflow. The fixed `period_buffer` length of 5 versus `buffer_size` from devicetree.

## Coverage

Files are `drivers/sensor/yf_s201c/yf_s201c.c` and `drivers/sensor/yf_s201c/yf_s201c.h`. Functions are `yf_s201c_work_handler`, `yf_s201c_reset`, `yf_s201c_get_flow_rate`, `yf_s201c_is_data_valid`. Sync commit is `5ca5a2c46fb5f05cc76a222b77f4e7691ed2f500`.

Move that commit forward only after you confirm the spec still describes those functions. A path that keeps a positive period in `ERROR` is a regression. Any other failing path is a new bug or a modeling error. Do not edit product code from this skill. Use `/maintain-formal-verification-skill` when the code and the spec drift.
