export const state = (v) => {
  const refused = v.refused === true || v.refused === "TRUE";
  if (v.hw === "on" && v.latest === "long" && v.initial === "long") return "Pump running";
  if (v.hw === "on" && v.initial === "short" && v.latest === "long") return "Pump running, period grew";
  if (v.hw === "on") return "Pump running, short period";
  if (v.lock === "shut" && v.age >= 2) return "Two seconds since the stop";
  if (v.lock === "shut" && refused && v.age === 1) return "A new start was ignored, one second gone";
  if (v.lock === "shut" && refused) return "A new start was ignored";
  if (v.lock === "shut" && v.age === 1) return "One second since the stop";
  if (v.lock === "shut") return "Just stopped for no flow";
  return "Pump off";
};

export const step = (action) => {
  const labels = {
    StartLong: "a plateau starts the pump",
    StartShort: "a short plateau starts the pump",
    ShorterWhileOn: null,
    LongerWhileOn: "a longer plateau arrives",
    Timeout: "sensor data times out",
    AgeOneSecond: "a second passes",
    PlateauWhileShut: "a plateau arrives too soon",
    StartAfterWait: "a plateau starts the pump",
    Emergency: "emergency stop",
  };
  return labels[action] ?? null;
};

export const group = (v) => {
  if (v.hw === "on") return "running";
  if (v.lock === "shut") return "lockout";
  return "off";
};

export const colors = {
  off: "tan",
  running: "teal",
  lockout: "orange",
};

export const title = "Pump run loop";
