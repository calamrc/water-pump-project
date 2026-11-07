---- MODULE Timer ----
\* Countdown in app/src/timer/timer_state_machine.c.
\* One tick is two steps because timer_tick_work_handler and the UI thread
\* do not share a lock. TickArm is the RUNNING test in timer_sm_update.
EXTENDS Integers

CONSTANTS MinTime, MaxTime, DefaultTime

VARIABLES phase, total, remaining, tick, corrupted
vars == <<phase, total, remaining, tick, corrupted>>

Clamp(n) ==
  IF n < MinTime THEN MinTime ELSE IF n > MaxTime THEN MaxTime ELSE n

TypeOK ==
  /\ phase \in {"setting", "running", "paused", "completed"}
  /\ total \in MinTime..MaxTime
  /\ remaining \in 0..MaxTime
  /\ tick \in {"idle", "owed"}
  /\ corrupted \in BOOLEAN

Init ==
  /\ phase = "setting"
  /\ total = DefaultTime
  /\ remaining = DefaultTime
  /\ tick = "idle"
  /\ corrupted = FALSE

AdjustUp ==
  \/ /\ phase = "setting"
     /\ total' = Clamp(total + 1)
     /\ UNCHANGED <<phase, remaining, tick, corrupted>>
  \/ /\ phase = "paused"
     /\ remaining' = Clamp(remaining + 1)
     /\ UNCHANGED <<phase, total, tick, corrupted>>

AdjustDown ==
  \/ /\ phase = "setting"
     /\ total' = Clamp(total - 1)
     /\ UNCHANGED <<phase, remaining, tick, corrupted>>
  \/ /\ phase = "paused"
     /\ remaining' = Clamp(remaining - 1)
     /\ UNCHANGED <<phase, total, tick, corrupted>>

Start ==
  /\ phase = "setting"
  /\ phase' = "running"
  /\ remaining' = total
  /\ UNCHANGED <<total, tick, corrupted>>

Pause ==
  /\ phase = "running"
  /\ phase' = "paused"
  /\ UNCHANGED <<total, remaining, tick, corrupted>>

Resume ==
  /\ phase = "paused"
  /\ phase' = "running"
  /\ UNCHANGED <<total, remaining, tick, corrupted>>

Reset ==
  /\ phase' = "setting"
  /\ total' = DefaultTime
  /\ remaining' = DefaultTime
  /\ UNCHANGED <<tick, corrupted>>

TickArm ==
  /\ phase = "running"
  /\ tick = "idle"
  /\ remaining > 0
  /\ tick' = "owed"
  /\ UNCHANGED <<phase, total, remaining, corrupted>>

TickCommit ==
  /\ tick = "owed"
  /\ remaining > 0
  /\ remaining' = remaining - 1
  /\ tick' = "idle"
  /\ phase' = IF remaining - 1 = 0 THEN "completed" ELSE phase
  /\ corrupted' = IF phase /= "running" THEN TRUE ELSE corrupted
  /\ UNCHANGED total

Next ==
  \/ AdjustUp \/ AdjustDown
  \/ Start \/ Pause \/ Resume \/ Reset
  \/ TickArm \/ TickCommit

Spec == Init /\ [][Next]_vars

OwedOnlyWhileRunning == tick = "owed" => phase = "running"

PauseDoesNotCorrupt == corrupted = FALSE

CompletedAtZero == phase = "completed" => remaining = 0

====
