const names = {
  off: "Off",
  starting: "Starting",
  running: "Running",
  timeout: "Timed out",
  error: "Relay write failed",
  maintenance: "Maintenance",
};

export const state = (v) => {
  const on = v.relay === true || v.relay === "TRUE";
  if (v.phase === "off" && on) return "Off, relay still energized";
  if (v.phase === "running" && !on) return "Running, relay is open";
  return names[v.phase];
};

export const step = (action) => {
  const labels = {
    TurnOn: "a plateau starts the pump",
    RelayOnFails: "the relay refuses to close",
    TurnOff: "the run stops",
    StopRelayFails: "the relay refuses to open",
    SafetyExpiry: "the safety timer fires",
    EmergencyStop: "emergency stop",
    ResetFromError: "reset after a failed relay write",
  };
  return labels[action] ?? null;
};

export const group = (v) => {
  const on = v.relay === true || v.relay === "TRUE";
  if (v.phase === "off" && on) return "wrong";
  if (v.phase === "running" && !on) return "wrong";
  if (v.phase === "error") return "fault";
  if (v.phase === "running") return "running";
  return "off";
};

export const colors = {
  off: "tan",
  running: "teal",
  fault: "orange",
  wrong: "pink",
};

export const title = "Pump driver";
