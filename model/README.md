# Plant Model for Water Pump Project

This directory contains a Python-based continuous plant model for the water pump system.

## Purpose

The main goal is to support **better automated testing and algorithm development** without hardware.

Instead of manually generating raw pulse trains with `gpio_emul_input_set`, you can use a more realistic model of how flow behaves when a faucet is opened/closed and when the pump turns on/off.

## Key Features

- Continuous first-order dynamics (flow does not change instantly)
- Separate time constants for faucet demand and pump effect
- Configurable gains and noise
- Easy conversion from flow rate (L/min) → YF-S201C pulse period (µs)
- Reusable for generating test vectors or future co-simulation

## Quick Start

```python
from model.plant import FlowPlant, simple_watering_session

# Run a pre-defined scenario
pulses = simple_watering_session(duration_s=35.0)

for t, period_us in pulses[::20]:   # every 1 second
    print(f"t={t:.1f}s → period = {period_us} µs")
```

## Generated Files

- `plant/flow_plant.py` — Core continuous plant model
- `plant/generate_pulse_train.py` — Helpers to generate realistic pulse sequences
- `plant/example_simulation.py` — Example script with plotting

## Future Ideas

- Export pulse trains directly consumable by the C emulation tests
- Add more failure modes (leaks, pump weakness, sensor noise patterns)
- Co-simulation bridge (Python model ↔ compiled firmware or Zephyr POSIX build)
- Parameter identification from real hardware data (when available)

## Notes

- This model is **not** meant to replace the existing C unit tests.
- It is primarily a development and test-vector generation tool.
- The model deliberately stays relatively simple so it remains easy to understand and tune.
