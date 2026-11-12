---- MODULE Sensor ----
\* drivers/sensor/yf_s201c/yf_s201c.c
\* get_flow_rate refuses only INIT. is_data_valid refuses INIT and ERROR.
EXTENDS Integers

CONSTANTS Threshold

VARIABLES mode, invalid, rateKept
vars == <<mode, invalid, rateKept>>

TypeOK ==
  /\ mode \in {"ready", "running", "error"}
  /\ invalid \in 0..Threshold
  /\ rateKept \in BOOLEAN

Init == mode = "ready" /\ invalid = 0 /\ rateKept = FALSE

ValidSample ==
  /\ invalid' = 0
  /\ rateKept' = TRUE
  /\ mode' = "running"

InvalidSample ==
  /\ invalid < Threshold
  /\ invalid' = invalid + 1
  /\ mode' = IF invalid + 1 >= Threshold THEN "error" ELSE mode
  /\ rateKept' = IF invalid + 1 >= Threshold THEN FALSE ELSE rateKept

Reset ==
  /\ mode' = "ready"
  /\ invalid' = 0
  /\ rateKept' = FALSE

Next == ValidSample \/ InvalidSample \/ Reset

Spec == Init /\ [][Next]_vars

NoPositiveRateInError == (mode = "error") => (rateKept = FALSE)

====
