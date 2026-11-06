# Introduction

This is the Doxygen documentation for the Zephyr Water Pump Application.

## Application Structure

`main()` initializes the error handler, the UI manager, and the pump controller, then starts the worker threads. Health monitoring stays in the main thread.

### Threads

- **Sensor monitor**: YF-S201C flow samples
- **Flow analyzer**: fixed-point filtering and plateau detection
- **Pump controller**: relay control from analyzer and timer events
- **Safety monitor**: runtime limit and stale-data shutdown
- **UI manager**: SH1106 display through Zephyr's character framebuffer, plus button input

The timer state machine runs inside the UI path and publishes countdown state on zbus. The thread manager creates the threads and watches their health.

Drivers under `drivers/` own the pump relay, the feedback relay, and the flow sensor. Modules publish state on zbus.

## Safety Features

- Pump runtime limit with automatic shutdown
- Stale sensor data stops the pump
- Error reporting through the error handler

## Hardware

- Flow sensor on GPIO 23
- Pump relay on GPIO 22, active low
- SH1106 128x64 display
