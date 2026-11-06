---- MODULE Restart ----
\* Restart question in app/src/ui_manager.c handle_input_event.
VARIABLES dialog, timer, edited
vars == <<dialog, timer, edited>>

TypeOK ==
  /\ dialog \in {"off", "yes", "no", "rebooted"}
  /\ timer \in {"setting", "running", "paused", "done"}
  /\ edited \in BOOLEAN

Init == dialog = "off" /\ timer = "setting" /\ edited = FALSE

MarkEdit(d, t, t2, e) ==
  e \/ (d \in {"yes", "no"} /\ t2 /= t /\ ~(t = "running" /\ t2 = "done"))

Hold ==
  /\ dialog = "off"
  /\ dialog' = "yes"
  /\ edited' = MarkEdit(dialog, timer, timer, edited)
  /\ UNCHANGED timer

Flip ==
  \/ /\ dialog = "yes" /\ dialog' = "no"
     /\ edited' = MarkEdit(dialog, timer, timer, edited)
     /\ UNCHANGED timer
  \/ /\ dialog = "no" /\ dialog' = "yes"
     /\ edited' = MarkEdit(dialog, timer, timer, edited)
     /\ UNCHANGED timer

Confirm ==
  /\ dialog = "yes"
  /\ dialog' = "rebooted"
  /\ edited' = MarkEdit(dialog, timer, timer, edited)
  /\ UNCHANGED timer

Cancel ==
  /\ dialog = "no"
  /\ dialog' = "off"
  /\ edited' = MarkEdit(dialog, timer, timer, edited)
  /\ UNCHANGED timer

Start ==
  /\ dialog = "off"
  /\ timer = "setting"
  /\ timer' = "running"
  /\ edited' = MarkEdit(dialog, timer, "running", edited)
  /\ UNCHANGED dialog

Pause ==
  /\ dialog = "off"
  /\ timer = "running"
  /\ timer' = "paused"
  /\ edited' = MarkEdit(dialog, timer, "paused", edited)
  /\ UNCHANGED dialog

Resume ==
  /\ dialog = "off"
  /\ timer = "paused"
  /\ timer' = "running"
  /\ edited' = MarkEdit(dialog, timer, "running", edited)
  /\ UNCHANGED dialog

Tick ==
  /\ timer = "running"
  /\ timer' = "done"
  /\ edited' = MarkEdit(dialog, timer, "done", edited)
  /\ UNCHANGED dialog

Done ==
  /\ dialog = "rebooted"
  /\ UNCHANGED vars

Next == Hold \/ Flip \/ Confirm \/ Cancel \/ Start \/ Pause \/ Resume \/ Tick \/ Done

Spec == Init /\ [][Next]_vars

DialogDoesNotEditTimer == edited = FALSE

====
