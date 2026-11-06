---- MODULE Button ----
\* Press classes in app/src/input/input_manager.c.
\* Key down starts the 1s and 3s timers. The 3s timer emits hold and stops
\* the 1s timer. Key up emits long or short only when hold was not sent.
VARIABLES phase, emitted, holdArmed
vars == <<phase, emitted, holdArmed>>

TypeOK ==
  /\ phase \in {"up", "downShort", "downLong", "holdSent"}
  /\ emitted \in {"none", "short", "long", "hold"}
  /\ holdArmed \in BOOLEAN

Init == phase = "up" /\ emitted = "none" /\ holdArmed = FALSE

Press ==
  /\ phase = "up"
  /\ phase' = "downShort"
  /\ emitted' = "none"
  /\ holdArmed' = FALSE

OneSecond ==
  /\ phase = "downShort"
  /\ phase' = "downLong"
  /\ UNCHANGED <<emitted, holdArmed>>

HoldFires ==
  /\ phase \in {"downShort", "downLong"}
  /\ phase' = "holdSent"
  /\ emitted' = "hold"
  /\ holdArmed' = TRUE

Release ==
  \/ /\ phase = "downShort"
     /\ phase' = "up"
     /\ emitted' = "short"
     /\ UNCHANGED holdArmed
  \/ /\ phase = "downLong"
     /\ phase' = "up"
     /\ emitted' = "long"
     /\ UNCHANGED holdArmed
  \/ /\ phase = "holdSent"
     /\ phase' = "up"
     /\ emitted' = "none"
     /\ UNCHANGED holdArmed

Next == Press \/ OneSecond \/ HoldFires \/ Release

Spec == Init /\ [][Next]_vars

QuietAfterHold == holdArmed => emitted \in {"hold", "none"}

====
