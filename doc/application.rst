Water Pump Application
======================

Overview
--------

The Zephyr Water Pump Application implements an intelligent, demand-based water pumping system that monitors flow rates and controls pump operation to optimize performance and safety.

The application reads flow data from a YF-S201C hall-effect flow sensor and uses sophisticated algorithms to detect stable flow conditions (plateau detection) before activating the pump. This prevents pump activation during turbulent or unstable flow periods.

Key Components
--------------

Hardware Interfaces
~~~~~~~~~~~~~~~~~~~

Flow Sensor (GPIO 23)
  - YF-S201C hall-effect sensor with 450 pulses per liter
  - Connected to GPIO 23 with falling-edge interrupts
  - Measures flow rate by timing intervals between pulses

Pump Relay (GPIO 22)
  - Controls pump power supply
  - Active low: 0 = Pump ON, 1 = Pump OFF
  - Integrated with safety timer for maximum runtime protection

Software Architecture
~~~~~~~~~~~~~~~~~~~~~

The application consists of several integrated subsystems:

ISR-Based Flow Measurement
  Interrupt service routine processes sensor pulses in real-time

Signal Processing Pipeline
  - Median filtering of period measurements
  - Outlier rejection using statistical bounds
  - Flow rate calculation from pulse periods

Plateau Detection Algorithm
  Advanced algorithm determining flow stability for pump activation

Safety and Control System
  - Runtime safety timer (5-minute maximum)
  - Stale data detection and timeout handling
  - Semaphore-based task synchronization

Core Algorithms
---------------

Flow Rate Calculation
~~~~~~~~~~~~~~~~~~~~~~

The flow rate is calculated from the period between sensor pulses:

.. math::

   \text{Flow Rate (L/min)} = \frac{60 \times 10^6}{\text{Period (μs)} \times 450}

Where 450 is the YF-S201C pulses per liter specification.

Median Filtering
~~~~~~~~~~~~~~~~~

Period measurements are stored in a circular buffer of 5 values. When the buffer is full, measurements are validated against the median:

.. code-block:: c

   if (current_period < median / 1.5 || current_period > median * 1.5) {
       use_median;  // Reject outlier
   } else {
       use_current; // Accept measurement
   }

Plateau Detection
~~~~~~~~~~~~~~~~~

The plateau detection algorithm monitors flow rate stability over time:

1. **Sliding Window**: Maintains buffer of most recent flow measurements
2. **Slope Analysis**: Calculates linear trend in recent measurements
3. **Noise Estimation**: Computes standard deviation of residuals from linear fit
4. **Stability Threshold**: Compares flow deltas against noise-adaptive threshold
5. **Confirmation**: Requires consecutive stable measurements before activation

The algorithm distinguishes between:
- Linear flow changes (significant slope)
- Stable plateaus (minimal noise within threshold)
- Noise-induced fluctuations (statistically bounded)

Safety Mechanisms
-----------------

Runtime Safety Timer
~~~~~~~~~~~~~~~~~~~~~

- Maximum pump runtime: 5 minutes
- Automatic shutdown on timeout to prevent overheating/damage
- Resettable on state changes

Stale Data Protection
~~~~~~~~~~~~~~~~~~~~~

- Monitors sensor data freshness (200ms threshold)
- Pump shutdown on sensor failure or disconnection
- Invalid data accumulation tracking

Configuration Parameters
------------------------

Sensor Specifications
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   #define YF_S201C_PULSES_PER_LITER 450
   #define FLOW_THRESHOLD_L_PER_MIN 0.1f
   #define MIN_PERIOD_US 100

Filtering Constants
~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   #define CONSECUTIVE_INVALID_THRESHOLD 5
   #define STALE_PERIOD_THRESHOLD_MS 200
   #define SEM_GIVE_MIN_INTERVAL_MS 10

Plateau Detection Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   #define PLATEAU_CONFIRM_COUNT 2
   #define PLATEAU_WINDOW_SIZE 5
   #define PLATEAU_MIN_SLOPE 0.01f
   #define PLATEAU_K_FACTOR 3.0f

Safety Parameters
~~~~~~~~~~~~~~~~~~

.. code-block:: c

   #define PUMP_ON_DEBOUNCE_MS 3000
   #define PUMP_SAFETY_TIMEOUT_MIN 5
   #define MAX_TIMEOUT_US 1000000LL

Operational Flow
----------------

The application runs in an infinite loop with the following states:

Idle State
  Waiting for sensor data semaphore

Measurement Processing
  - Calculate flow rate from sensor periods
  - Apply filtering and validation
  - Detect plateau conditions

Pump Control
  - Activate pump when plateau detected in pump-off state
  - Allow run-time to stabilize before final plateau assessment
  - Maintain pump state during run-time variations

Timeout Handling
  - Shutdown on stale data
  - Force pump off on semaphore timeout
  - Reset all buffers and state

Logging and Monitoring
-----------------------

The application provides comprehensive logging:

- Flow rate measurements (INFO level)
- Plateau detection events (INFO level)
- Pump state changes (INFO level)
- Calibration values (DEBUG level)
- Error conditions (ERROR level)

Debug logging includes:
- Plateau detection intermediate values
- Buffer states
- Calibration parameters

Deployment
----------

The application is designed for embedded systems with Zephyr RTOS support. It requires:
- GPIO support for sensor and relay
- Interrupt capabilities for real-time sensor reading
- Timer facilities for safety timeouts
- Logging infrastructure for monitoring

Configuration is handled through Kconfig and devicetree overlays for different board targets.
