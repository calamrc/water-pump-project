# Zephyr Water Pump Application
## Application Overview

The Zephyr Water Pump application implements an intelligent water pump control system using flow rate feedback for demand-based pumping.

### Key Features

#### Smart Flow Control
- **Plateau Detection**: Advanced algorithm that monitors flow rate stability to detect optimal pump operation points
- **Flow Rate Monitoring**: Real-time measurement using YF-S201C flow sensor (450 pulses per liter)
- **Adaptive Thresholding**: Statistical analysis to distinguish between flow changes and noise

#### Safety Mechanisms
- **Runtime Safety Timer**: Automatic pump shutdown after 5 minutes of continuous operation
- **Stale Data Protection**: Pump shutdown on sensor data timeout
- **Relay Control**: Safe pump on/off with error handling

#### Signal Processing
- **Median Filtering**: Robust period measurement filtering for sensor noise rejection
- **Outlier Detection**: Statistical bounds checking for invalid measurements
- **Debouncing**: Minimum period validation and consecutive invalid thresholds

### Architecture

The application uses Zephyr RTOS features for:
- GPIO interrupts for sensor reading
- Semaphore-based task synchronization
- Kernel timers for safety timeout
- Comprehensive logging for debugging and monitoring

### Hardware Interfaces
- **Flow Sensor**: GPIO 23, falling edge interrupt
- **Pump Relay**: GPIO 22, active low (0 = ON, 1 = OFF)
