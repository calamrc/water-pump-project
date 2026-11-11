---- MODULE Safety ----
\* safety_monitor_thread in app/src/thread_manager.c.
\* One quiet interval stands for the sensor timeout. MaxTicks stands for
\* the configured max runtime. Both are bounds, not the Kconfig numbers.
EXTENDS Integers

CONSTANTS MaxTicks

VARIABLES mode, runtime, sensor, pump, starts
vars == <<mode, runtime, sensor, pump, starts>>

TypeOK ==
  /\ mode \in {"idle", "running", "latched"}
  /\ runtime \in 0..MaxTicks
  /\ sensor \in {"never", "fresh", "old"}
  /\ pump \in BOOLEAN
  /\ starts \in 0..2

Init ==
  /\ mode = "idle"
  /\ runtime = 0
  /\ sensor = "never"
  /\ pump = FALSE
  /\ starts = 0

TurnedOn ==
  /\ mode = "idle"
  /\ mode' = "running"
  /\ runtime' = 0
  /\ sensor' = "never"
  /\ pump' = TRUE
  /\ starts' = starts + 1

TurnedOnAgain ==
  /\ mode = "running"
  /\ UNCHANGED vars

Sample ==
  /\ mode = "running"
  /\ sensor' = "fresh"
  /\ UNCHANGED <<mode, runtime, pump, starts>>

GoQuiet ==
  /\ mode = "running"
  /\ sensor \in {"never", "fresh"}
  /\ sensor' = "old"
  /\ UNCHANGED <<mode, runtime, pump, starts>>

Advance ==
  /\ mode = "running"
  /\ runtime < MaxTicks
  /\ runtime' = runtime + 1
  /\ mode' = IF runtime + 1 >= MaxTicks THEN "latched" ELSE mode
  /\ pump' = IF runtime + 1 >= MaxTicks THEN FALSE ELSE pump
  /\ UNCHANGED <<sensor, starts>>

Stall ==
  /\ mode = "running"
  /\ sensor = "old"
  /\ mode' = "latched"
  /\ pump' = FALSE
  /\ UNCHANGED <<runtime, sensor, starts>>

StopFails ==
  /\ mode = "running"
  /\ \/ runtime + 1 >= MaxTicks
     \/ sensor = "old"
  /\ UNCHANGED vars

TurnedOff ==
  /\ mode = "running"
  /\ mode' = "idle"
  /\ runtime' = 0
  /\ sensor' = "never"
  /\ pump' = FALSE
  /\ starts' = 0

StopSeen ==
  /\ mode = "latched"
  /\ pump = FALSE
  /\ mode' = "idle"
  /\ runtime' = 0
  /\ sensor' = "never"
  /\ pump' = FALSE
  /\ starts' = 0

Next ==
  \/ TurnedOn \/ TurnedOnAgain
  \/ Sample \/ GoQuiet
  \/ Advance \/ Stall \/ StopFails
  \/ TurnedOff \/ StopSeen

Spec == Init /\ [][Next]_vars

OneStartPerRun == starts <= 1

LatchedMeansOff == (mode = "latched") => (pump = FALSE)

RunningHasPump == (mode = "running") => (pump = TRUE)

====
