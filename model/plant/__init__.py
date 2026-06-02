"""
Plant model package for the Zephyr Water Pump project.

Provides a continuous dynamics simulation of the physical system
(faucet + pipes + pump effect) that generates realistic flow rates
and corresponding YF-S201C pulse periods.

Intended primarily for:
- Algorithm development
- Generating realistic test vectors
- Future co-simulation work
"""

from .flow_plant import FlowPlant, PlantParams
from .generate_pulse_train import generate_pulse_periods, simple_watering_session

__all__ = [
    "FlowPlant",
    "PlantParams",
    "generate_pulse_periods",
    "simple_watering_session",
]
