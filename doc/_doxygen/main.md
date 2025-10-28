# Introduction

This is the Doxygen documentation for the [Zephyr Water Pump Application].

## Application Structure

The application implements intelligent water pump control with the following main components:

### Core Functionality (`main.c`)
- **Flow Measurement**: Real-time processing of YF-S201C flow sensor data
- **Signal Processing**: Advanced filtering and outlier rejection algorithms
- **Plateau Detection**: Statistical analysis for flow stability determination
- **Pump Control**: Safe relay operation with timeout protection

### Key Functions

#### Flow Sensor Processing
- `flow_sensor_isr()`: Interrupt handler for sensor pulses
- `get_median()`: Robust median calculation for noise filtering
- `calibrate_plateau()`: Statistical calibration of flow stability

#### State Management
- `detect_plateau()`: Algorithm for identifying stable flow conditions
- `safety_timer_handler()`: Emergency pump shutdown mechanism
- `main()`: Application initialization and control loop

#### Configuration
- **Sensor Constants**: YF-S201C specifications and thresholds
- **Safety Parameters**: Runtime limits and protection mechanisms
- **Algorithm Tuning**: Plateau detection and filtering coefficients

## Safety Features

- Maximum 5-minute pump runtime with automatic shutdown
- Stale data detection preventing operation on sensor failure
- Statistical outlier rejection for robust measurements
- Comprehensive error logging and state monitoring

## Hardware Abstraction

The application uses Zephyr's GPIO, timer, and semaphore APIs for:
- Interrupt-driven sensor reading (GPIO 23)
- Pump relay control (GPIO 22, active low)
- Safety timeout implementation
- Task synchronization

[Zephyr Water Pump Application]: https://github.com/zephyrproject-rtos/water-pump
