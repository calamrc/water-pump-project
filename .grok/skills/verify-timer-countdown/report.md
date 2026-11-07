# Timer

TLC reached 93 states and 342 steps. The chart draws 11 states and 20 arrows.
The chart does not draw 15 steps that stay in one chart state.
The view hides 177 steps, so the chart does not draw them.

## OwedOnlyWhileRunning breaks
The shortest path has 3 steps:
0. Setting the time
1. you press start: Counting down
2. a second passes: Counting down, a second is in flight
3. you press pause: Paused, a tick is still writing

## PauseDoesNotCorrupt breaks
The shortest path has 4 steps:
0. Setting the time
1. you press start: Counting down
2. a second passes: Counting down, a second is in flight
3. you press pause: Paused, a tick is still writing
4. the tick writes the new second: Paused, countdown moved after it should have stopped

## Rules that hold
TypeOK, CompletedAtZero, NoDeadlock
