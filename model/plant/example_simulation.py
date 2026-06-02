"""
Example usage of the FlowPlant model.

This script runs a simulated watering session and prints some statistics
that could be used to generate realistic test vectors.
"""

from .flow_plant import FlowPlant, PlantParams
import matplotlib.pyplot as plt


def main():
    params = PlantParams(
        faucet_tau=0.7,
        pump_tau=1.1,
        faucet_gain=4.2,
        pump_gain=3.8,
        flow_noise_std=0.07,
    )

    plant = FlowPlant(params)

    dt = 0.05  # 50ms simulation step
    duration = 45.0  # seconds

    times = []
    flows = []
    periods = []

    faucet_cmd = 0.0
    pump_on = False

    for step in range(int(duration / dt)):
        t = step * dt

        # Scenario timeline
        if 3.0 < t < 22.0:
            faucet_cmd = 0.72  # faucet opened to ~70%
        else:
            faucet_cmd = 0.0

        if 6.5 < t < 24.0:
            pump_on = True
        else:
            pump_on = False

        flow = plant.step(dt, faucet_cmd, pump_on)
        period_us = plant.flow_to_period_us(flow)

        times.append(t)
        flows.append(flow)
        periods.append(period_us)

    # Print some useful statistics
    print("Simulation finished.")
    print(f"Max flow: {max(flows):.2f} L/min")
    print(f"Average flow (during faucet open): {sum(flows[60:440]) / 380:.2f} L/min")
    print(f"Number of simulated pulses (rough): {len([p for p in periods if p < 5_000_000])}")

    # Plot
    plt.figure(figsize=(12, 5))
    plt.plot(times, flows, label="Flow rate (L/min)")
    plt.xlabel("Time (s)")
    plt.ylabel("Flow (L/min)")
    plt.title("Flow Plant Model - Example Watering Session")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
