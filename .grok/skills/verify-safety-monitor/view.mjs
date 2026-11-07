export const state = (v) => {
  const on = v.pump === true || v.pump === "TRUE";
  if (v.mode === "latched" && on) return "Stop failed, pump still on";
  if (v.mode === "latched") return "Stopped by the monitor";
  if (v.mode === "idle") return "Pump off";
  if (v.sensor === "old") return "Running, sensor has gone quiet";
  if (v.runtime >= 2) return "Running, near the time limit";
  return "Running";
};

export const step = (action) => {
  const labels = {
    TurnedOn: "the pump turns on",
    TurnedOnAgain: "the pump reports on again",
    Sample: "a sensor sample arrives",
    GoQuiet: "the sensor goes quiet",
    Advance: "time passes",
    Stall: "the quiet sensor trips the stop",
    StopFails: "the stop call fails",
    TurnedOff: "the pump turns off",
    StopSeen: "the stop is confirmed",
  };
  return labels[action] ?? null;
};

export const group = (v) => {
  const on = v.pump === true || v.pump === "TRUE";
  if (v.mode === "latched" && on) return "wrong";
  if (v.mode === "latched") return "stopped";
  if (v.mode === "running") return "running";
  return "idle";
};

export const colors = {
  idle: "tan",
  running: "teal",
  stopped: "orange",
  wrong: "pink",
};

export const title = "Safety monitor";
