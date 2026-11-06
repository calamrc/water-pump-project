# Sensor

TLC reached 6 states and 16 steps. The chart draws 6 states and 14 arrows.
The chart does not draw 2 steps that stay in one chart state.

## NoPositiveRateInError breaks
The shortest path has 3 steps:
0. Ready
1. a pulse looks valid: Measuring
2. a pulse looks invalid: Measuring, one bad pulse
3. a pulse looks invalid: Faulted, last rate still readable

## Rules that hold
TypeOK, NoDeadlock
