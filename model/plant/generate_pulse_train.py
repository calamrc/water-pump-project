"""
Utility to generate realistic pulse period sequences from the FlowPlant model.

Primary purpose (option B):
  Feed the *exact* same Zephyr emulation tests (qemu_x86 / native_sim +
  gpio_emul + real thin driver + flow_processor + full zbus services)
  with continuous-dynamics data produced by the Python plant.

Usage examples:
  python -m model.plant.generate_pulse_train --scenario watering \
         --output-c tests/emulation/test_pump_flow_smoke/src/generated_plant_data.h

  Then rebuild and run the test suite — the new ZTEST will automatically
  replay the plant-generated periods through the real firmware path.
"""

try:
    from .flow_plant import FlowPlant, PlantParams
except ImportError:
    # Allow direct script execution (python generate_pulse_train.py)
    import sys
    import os
    sys.path.insert(0, os.path.dirname(__file__))
    from flow_plant import FlowPlant, PlantParams
from typing import List, Tuple


def generate_pulse_periods(
    duration_s: float,
    dt: float = 0.02,
    faucet_profile=None,
    pump_profile=None,
    params: PlantParams = None,
) -> List[Tuple[float, int]]:
    """
    Simulate the plant and return a list of (time, period_us) tuples.

    This is the most useful output for feeding into your Zephyr emulation tests.
    """
    plant = FlowPlant(params)
    results = []
    t = 0.0

    while t < duration_s:
        faucet = faucet_profile(t) if callable(faucet_profile) else (faucet_profile or 0.0)
        pump_on = pump_profile(t) if callable(pump_profile) else (pump_profile or False)

        flow = plant.step(dt, faucet, pump_on)
        period = plant.flow_to_period_us(flow)
        results.append((t, period))
        t += dt

    return results


def simple_watering_session(duration_s: float = 40.0) -> List[Tuple[float, int]]:
    """
    A convenient pre-defined scenario:
    - Faucet opens at t=3s
    - Pump turns on at t=6s
    - Faucet closes at t=23s
    - Pump turns off at t=25s
    """
    def faucet(t):
        return 0.68 if 3.0 < t < 23.0 else 0.0

    def pump(t):
        return 3.0 < t < 25.0

    return generate_pulse_periods(duration_s, faucet_profile=faucet, pump_profile=pump)


def realistic_20s_watering_session() -> List[Tuple[float, int]]:
    """
    A 20-second realistic watering session focused on flow analysis testing:

    Timeline:
    - 0.0s - 2.5s : Faucet turning on (flow rising from 0)
    - 2.5s - 5.5s : Low flow stabilizes (analyzer should detect first plateau while pump off)
    - 5.5s - 6.5s : Pump turns on (in real system this would be triggered by plateau + demand logic)
    - 6.5s - 8.0s : Flow increases due to pump assistance
    - 8.0s - 17.0s: Higher flow stabilizes (second plateau at higher rate)
    - 17.0s - 20.0s: Faucet turned off (flow drops *significantly* even with pump still on;
                    the firmware's 1.5x pulse watchdog should eventually turn the pump off)
    """
    def faucet(t):
        if t < 2.5:
            # Faucet opening phase - gradual
            return (t / 2.5) * 0.35          # ramps 0 → 0.35 L/min
        elif t < 17.0:
            return 0.35                       # steady low flow from faucet
        else:
            # Faucet closed at t=17.0 — sharp step drop (realistic dead-head behavior)
            # When the faucet closes, flow through the sensor drops very abruptly
            # (almost step-like), even while the pump is still running.
            # This is the behavior needed for the 1.5x watchdog to trigger naturally
            # while the pump is on.
            # Hard step drop at the exact moment the faucet is closed (t >= 17.0)
            # This is as step-like as the model allows for this scenario.
            return 0.01 if t >= 17.0 else 0.35

    def pump(t):
        # In a full simulation the pump "on" time would be decided by the firmware.
        # For this trace we model the physical effect of the pump being on.
        # We keep the pump "on" in the plant a bit after the faucet closes so that
        # the firmware's 1.5x watchdog (not the plant) is what turns the flow off
        # in the simulation. This better matches real usage + the original contract.
        if 6.0 < t < 20.5:
            return True
        return False

    raw_pulses = generate_pulse_periods(20.0, dt=0.02, faucet_profile=faucet, pump_profile=pump)

    # --- Physics override for realistic quick faucet close ---
    # When you quickly close the faucet (even with the pump still running),
    # flow at the sensor collapses much faster than the simple first-order plant
    # can produce. We apply a faster exponential decay starting at t=17.0
    # to match real dead-head behavior. This is necessary for the 1.5x
    # pulse watchdog to have a realistic chance to trigger.
    close_t = 17.0
    decay_tau = 0.40          # fast decay constant (realistic for quick faucet close)
    residual_flow = 0.012     # very low residual flow after close

    import math

    # First pass: find approximate flow at close time from the raw periods
    flow_at_close = 0.35
    for t, period in raw_pulses:
        if t >= close_t:
            if period > 1000:
                flow_at_close = 60_000_000.0 / (period * 450.0)
            break

    new_pulses = []
    for t, period in raw_pulses:
        if t >= close_t:
            # Fast physical collapse of flow
            elapsed = t - close_t
            decayed_flow = residual_flow + (flow_at_close - residual_flow) * math.exp(-elapsed / decay_tau)

            # Convert corrected flow back to period
            if decayed_flow < 0.04:
                new_period = 6_000_000   # very long period = almost no pulses
            else:
                new_period = int(60_000_000 / (decayed_flow * 450))
            new_pulses.append((t, new_period))
        else:
            new_pulses.append((t, period))

    return new_pulses


def export_as_c_array(pulses: List[Tuple[float, int]],
                      var_name: str = "plant_pulse_periods_us",
                      filename: str = "generated_plant_data.h") -> None:
    """
    Export the pulse periods as a C header with a const array.
    Produces the exact filename and layout expected by
    tests/emulation/test_pump_flow_smoke (via __has_include).
    """
    periods = [p for (t, p) in pulses]

    with open(filename, "w") as f:
        f.write("/*\n")
        f.write(" * Auto-generated by the FlowPlant model (option B data-driven path).\n")
        f.write(" * Do not edit by hand. Regenerate via model/plant/generate_pulse_train.py\n")
        f.write(" * For 20s realistic: use --scenario realistic (or realistic_20s).\n")
        f.write(" * Note: contains random.gauss noise from FlowPlant; exact periods\n")
        f.write(" * vary per regen (no global seed here) but event timeline (first\n")
        f.write(" * low plateau off->on, goods, hard drop ~t17 for wd) is stable.\n")
        f.write(" */\n")
        f.write("#ifndef GENERATED_PLANT_DATA_H_\n")
        f.write("#define GENERATED_PLANT_DATA_H_\n\n")
        f.write("#include <stdint.h>\n")
        f.write("#include <stddef.h>\n\n")
        f.write(f"/* Number of samples: {len(periods)} */\n\n")
        f.write(f"static const uint32_t {var_name}[] = {{\n")

        for i in range(0, len(periods), 8):
            chunk = periods[i:i+8]
            line = "    " + ", ".join(str(p) for p in chunk) + ",\n"
            f.write(line)

        f.write("};\n\n")
        f.write(f"static const size_t {var_name}_count = {len(periods)};\n\n")
        f.write("#endif /* GENERATED_PLANT_DATA_H_ */\n")

    print(f"Exported C array header to {filename}")


def export_as_csv(pulses: List[Tuple[float, int]],
                  filename: str = "plant_pulses.csv") -> None:
    """Export as simple CSV: time_s,period_us"""
    with open(filename, "w") as f:
        f.write("time_s,period_us\n")
        for t, p in pulses:
            f.write(f"{t},{p}\n")
    print(f"Exported CSV to {filename}")


if __name__ == "__main__":
    import argparse
    import os

    parser = argparse.ArgumentParser(
        description="Generate realistic pulse data from the FlowPlant model "
                    "for driving the exact Zephyr emulation tests (option B)."
    )
    parser.add_argument("--scenario", choices=["watering", "variable", "pump_react", "realistic", "realistic_20s"],
                        default="watering", help="Predefined scenario (realistic/realistic_20s for 20s faithful watering trace with hard close at t~17)")
    parser.add_argument("--duration", type=float, default=35.0, help="Duration in seconds")
    parser.add_argument("--dt", type=float, default=0.02, help="Simulation step (s)")
    parser.add_argument("--output-c", metavar="PATH",
                        help="Write generated_plant_data.h compatible header to this path "
                             "(e.g. tests/emulation/test_pump_flow_smoke/src/generated_plant_data.h)")
    parser.add_argument("--output-csv", metavar="PATH",
                        help="Also write a CSV (time_s,period_us)")
    args = parser.parse_args()

    if args.scenario == "watering":
        pulses = simple_watering_session(args.duration)
    elif args.scenario in ("realistic", "realistic_20s"):
        # 20s faithful scenario for host_sim/emul "I want B" + 20s tests (hard
        # close at ~17s for wd; uses internal decay override)
        pulses = realistic_20s_watering_session()
    else:
        # Fallback to basic generation for other scenarios
        def faucet(t): return 0.65 if 4.0 < t < 20.0 else 0.0
        def pump(t): return 5.0 < t < 22.0
        pulses = generate_pulse_periods(args.duration, dt=args.dt,
                                        faucet_profile=faucet, pump_profile=pump)

    print(f"Generated {len(pulses)} plant samples for scenario '{args.scenario}'")

    if args.output_c:
        export_as_c_array(pulses, filename=args.output_c)
        # Also drop a small README note next to it for discoverability
        note_dir = os.path.dirname(args.output_c) or "."
        with open(os.path.join(note_dir, "README_plant_data.txt"), "w") as nf:
            nf.write("This header was generated by model/plant/generate_pulse_train.py\n")
            nf.write("Run the script with --output-c to refresh after changing plant parameters.\n")
            nf.write("The emulation test (test_pump_flow_smoke) will automatically pick it up\n")
            nf.write("via __has_include when the file is present in its include path.\n")

    # Friendly hint for host_sim users
    if args.output_csv and not args.output_c:
        print("\nTip: You can now run the host simulator with:")
        print(f"   cd ../sim && make && ./host_sim {os.path.basename(args.output_csv)}")

    if args.output_csv:
        export_as_csv(pulses, filename=args.output_csv)

    if not args.output_c and not args.output_csv:
        # Legacy default behavior when invoked without args
        pulses = simple_watering_session(35.0)
        print(f"Generated {len(pulses)} samples (default)")
        print("First 8 periods (us):")
        for t, p in pulses[:8]:
            print(f"  t={t:5.1f}s -> {p:8d} us")
        export_as_csv(pulses, filename="watering_session_pulses.csv")
