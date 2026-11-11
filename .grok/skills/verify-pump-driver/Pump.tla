---- MODULE Pump ----
\* Live calls in drivers/pump/pump_controller.c.
\* The transition table is wider than the calls. TIMEOUT, SAFETY_TIMEOUT,
\* ERROR_DETECTED, MAINTENANCE_ENTER, and MAINTENANCE_EXIT have no caller.
\* emergency_stop and the safety work item force OFF. They do not use the
\* SAFETY_TIMEOUT column.
VARIABLES phase, relay
vars == <<phase, relay>>

TypeOK ==
  /\ phase \in {"off", "starting", "running", "timeout", "error", "maintenance"}
  /\ relay \in BOOLEAN

Init == phase = "off" /\ relay = FALSE

TurnOn ==
  /\ phase = "off"
  /\ phase' = "running"
  /\ relay' = TRUE

RelayOnFails ==
  /\ phase = "off"
  /\ phase' = "error"
  /\ relay' = FALSE

TurnOff ==
  /\ phase = "running"
  /\ phase' = "off"
  /\ relay' = FALSE

\* A failed coil write leaves the phase and the relay alone.
StopRelayFails ==
  /\ phase = "running"
  /\ UNCHANGED <<phase, relay>>

SafetyExpiry ==
  /\ phase = "running"
  /\ phase' = "off"
  /\ relay' = FALSE

EmergencyStop ==
  /\ phase = "running"
  /\ phase' = "off"
  /\ relay' = FALSE

ResetFromError ==
  /\ phase = "error"
  /\ phase' = "off"
  /\ UNCHANGED relay

Next ==
  \/ TurnOn \/ RelayOnFails
  \/ TurnOff \/ StopRelayFails
  \/ SafetyExpiry \/ EmergencyStop
  \/ ResetFromError

Spec == Init /\ [][Next]_vars

\* STARTING, TIMEOUT, and MAINTENANCE have no live assignment.
OnlyLivePhases == phase \in {"off", "running", "error"}

RelayMatchesRun == (relay = TRUE) <=> (phase = "running")

====
