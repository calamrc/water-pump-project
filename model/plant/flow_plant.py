"""
Simple continuous plant model for the water pump system.

Models:
- Faucet demand (user opening/closing a faucet)
- Pump effect on flow/pressure
- First-order dynamics (inertia in the system)
- Basic noise

The model outputs instantaneous flow rate (L/min), which can then be
converted into realistic YF-S201C pulse periods.
"""

import math
import random
from dataclasses import dataclass
from typing import Optional


@dataclass
class PlantParams:
    # Time constants (seconds)
    faucet_tau: float = 0.6          # How fast faucet flow builds
    pump_tau: float = 0.9            # How fast pump pressure/flow builds

    # Gains
    faucet_gain: float = 4.0         # Max flow from fully open faucet (L/min)
    pump_gain: float = 3.5           # Additional flow the pump can provide

    # Sensor characteristics
    pulses_per_liter: int = 450
    flow_noise_std: float = 0.08     # Standard deviation of flow noise (L/min)

    # Limits
    max_flow: float = 12.0           # Hard cap on flow rate


class FlowPlant:
    """
    Continuous plant model of the water system.

    Usage:
        plant = FlowPlant()
        for t in range(100):
            flow = plant.step(dt=0.1, faucet_opening=0.7, pump_on=True)
            period_us = plant.flow_to_period_us(flow)
    """

    def __init__(self, params: Optional[PlantParams] = None):
        self.params = params or PlantParams()

        # Internal states (first-order lags)
        self._faucet_flow = 0.0
        self._pump_contribution = 0.0

        self._last_flow = 0.0

    def reset(self):
        self._faucet_flow = 0.0
        self._pump_contribution = 0.0
        self._last_flow = 0.0

    def step(self, dt: float, faucet_opening: float, pump_on: bool) -> float:
        """
        Advance the plant by dt seconds.

        Args:
            dt: Time step in seconds
            faucet_opening: 0.0 (closed) to 1.0 (fully open)
            pump_on: Whether the pump relay is energized

        Returns:
            Current flow rate in L/min (with noise)
        """
        p = self.params
        faucet_opening = max(0.0, min(1.0, faucet_opening))

        # Target flows
        target_faucet = faucet_opening * p.faucet_gain

        if pump_on:
            # Pump helps push more water when faucet is open
            target_pump = faucet_opening * p.pump_gain
        else:
            target_pump = 0.0

        # First-order dynamics
        alpha_faucet = dt / (p.faucet_tau + 1e-9)
        alpha_pump = dt / (p.pump_tau + 1e-9)

        self._faucet_flow += (target_faucet - self._faucet_flow) * alpha_faucet
        self._pump_contribution += (target_pump - self._pump_contribution) * alpha_pump

        # Total flow before limits and noise
        raw_flow = self._faucet_flow + self._pump_contribution

        # Apply hard limit
        raw_flow = min(raw_flow, p.max_flow)

        # Add sensor noise
        noisy_flow = raw_flow + random.gauss(0.0, p.flow_noise_std)

        # Final safety clamp
        flow = max(0.0, min(noisy_flow, p.max_flow))

        self._last_flow = flow
        return flow

    def get_flow_rate(self) -> float:
        """Return the last computed flow rate (with noise)."""
        return self._last_flow

    def flow_to_period_us(self, flow_lpm: Optional[float] = None) -> int:
        """
        Convert flow rate (L/min) to expected pulse period in microseconds
        for the YF-S201C (450 pulses per liter).
        """
        if flow_lpm is None:
            flow_lpm = self._last_flow

        p = self.params

        if flow_lpm < 0.05:
            # Very low flow -> very long period (or we can return a sentinel)
            return 10_000_000  # 10 seconds as "no pulse" marker

        # period (us) = 60e6 / (flow_lpm * pulses_per_liter)
        period = int(60_000_000.0 / (flow_lpm * p.pulses_per_liter))
        return max(period, 1000)  # minimum 1ms period for sanity

    def simulate(self, duration: float, dt: float,
                 faucet_profile, pump_profile) -> list:
        """
        Run a simulation for `duration` seconds.

        faucet_profile and pump_profile can be:
            - a constant (float or bool)
            - a callable(t) -> value

        Returns a list of (time, flow_lpm, period_us) tuples.
        """
        results = []
        t = 0.0

        while t < duration:
            faucet = faucet_profile(t) if callable(faucet_profile) else faucet_profile
            pump_on = pump_profile(t) if callable(pump_profile) else pump_profile

            flow = self.step(dt, faucet, pump_on)
            period = self.flow_to_period_us(flow)
            results.append((t, flow, period))
            t += dt

        return results


# Example usage
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    plant = FlowPlant()

    times = []
    flows = []

    # Simple scenario: faucet opens at t=2s, pump turns on at t=5s, faucet closes at t=15s
    for t in range(300):  # 30 seconds @ 0.1s steps
        tt = t * 0.1
        faucet = 0.0
        pump = False

        if tt > 2.0:
            faucet = 0.65
        if tt > 5.0:
            pump = True
        if tt > 15.0:
            faucet = 0.0

        flow = plant.step(0.1, faucet, pump)
        times.append(tt)
        flows.append(flow)

    plt.figure(figsize=(10, 4))
    plt.plot(times, flows)
    plt.xlabel("Time (s)")
    plt.ylabel("Flow (L/min)")
    plt.title("Flow Plant Model - Example Session")
    plt.grid(True)
    plt.show()
