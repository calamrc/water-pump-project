# Safety

TLC reached 21 states and 57 steps. The chart draws 6 states and 14 arrows.
The chart does not draw 19 steps that stay in one chart state.

## LatchedMeansOff breaks
The shortest path has 3 steps:
0. Pump off
1. the pump turns on: Running
2. the sensor goes quiet: Running, sensor has gone quiet
3. the stop call fails: Stop failed, pump still on

## NoDeadlock breaks
The shortest path has 3 steps:
0. Pump off
1. the pump turns on: Running
2. the sensor goes quiet: Running, sensor has gone quiet
3. the stop call fails: Stop failed, pump still on

## Rules that hold
TypeOK, OneStartPerRun, RunningHasPump
