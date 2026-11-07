---- MODULE RunLoop ----
EXTENDS Integers
\* pump_controller_thread in app/src/thread_manager.c.
VARIABLES hw, lock, age, initial, latest, refused
vars == <<hw, lock, age, initial, latest, refused>>

TypeOK ==
  /\ hw \in {"off", "on"}
  /\ lock \in {"open", "shut"}
  /\ age \in 0..2
  /\ initial \in {"none", "short", "long"}
  /\ latest \in {"none", "short", "long"}
  /\ refused \in BOOLEAN

Init ==
  /\ hw = "off"
  /\ lock = "open"
  /\ age = 0
  /\ initial = "none"
  /\ latest = "none"
  /\ refused = FALSE

StartLong ==
  /\ hw = "off"
  /\ lock = "open"
  /\ hw' = "on"
  /\ initial' = "long"
  /\ latest' = "long"
  /\ refused' = FALSE
  /\ UNCHANGED <<lock, age>>

StartShort ==
  /\ hw = "off"
  /\ lock = "open"
  /\ hw' = "on"
  /\ initial' = "short"
  /\ latest' = "short"
  /\ refused' = FALSE
  /\ UNCHANGED <<lock, age>>

ShorterWhileOn ==
  /\ hw = "on"
  /\ initial = "long"
  /\ UNCHANGED <<hw, lock, age, initial, latest, refused>>

LongerWhileOn ==
  /\ hw = "on"
  /\ latest' = "long"
  /\ UNCHANGED <<hw, lock, age, initial, refused>>

Timeout ==
  /\ hw = "on"
  /\ hw' = "off"
  /\ lock' = "shut"
  /\ age' = 0
  /\ initial' = "none"
  /\ latest' = "none"
  /\ refused' = FALSE

AgeOneSecond ==
  /\ lock = "shut"
  /\ hw = "off"
  /\ age < 2
  /\ age' = age + 1
  /\ refused' = FALSE
  /\ UNCHANGED <<hw, lock, initial, latest>>

PlateauWhileShut ==
  /\ hw = "off"
  /\ lock = "shut"
  /\ age < 2
  /\ refused' = TRUE
  /\ UNCHANGED <<hw, lock, age, initial, latest>>

StartAfterWait ==
  /\ hw = "off"
  /\ lock = "shut"
  /\ age >= 2
  /\ hw' = "on"
  /\ lock' = "open"
  /\ age' = 0
  /\ initial' = "long"
  /\ latest' = "long"
  /\ refused' = FALSE

Emergency ==
  /\ hw = "on"
  /\ hw' = "off"
  /\ lock' = "open"
  /\ initial' = "none"
  /\ latest' = "none"
  /\ refused' = FALSE
  /\ UNCHANGED age

Next ==
  \/ StartLong \/ StartShort
  \/ ShorterWhileOn \/ LongerWhileOn
  \/ Timeout \/ AgeOneSecond \/ PlateauWhileShut
  \/ StartAfterWait \/ Emergency

Spec == Init /\ [][Next]_vars

ShutMeansOff == (lock = "shut") => (hw = "off")

NoShrink == (hw = "on" /\ initial = "long") => (latest = "long")

====
