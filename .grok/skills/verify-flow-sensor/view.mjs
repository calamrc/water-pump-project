export const state = (v) => {
  const kept = v.rateKept === true || v.rateKept === "TRUE";
  if (v.mode === "error" && kept) return "Faulted, last rate still readable";
  if (v.mode === "error") return "Faulted";
  if (v.mode === "running" && v.invalid > 0) return "Measuring, one bad pulse";
  if (v.mode === "running") return "Measuring";
  if (v.invalid > 0) return "Ready, one bad pulse";
  return "Ready";
};

export const step = (action) => {
  const labels = {
    ValidSample: "a pulse looks valid",
    InvalidSample: "a pulse looks invalid",
    Reset: "the sensor resets",
  };
  return labels[action] ?? null;
};

export const group = (v) => {
  const kept = v.rateKept === true || v.rateKept === "TRUE";
  if (v.mode === "error" && kept) return "wrong";
  if (v.mode === "error") return "fault";
  if (v.mode === "running") return "running";
  return "ready";
};

export const colors = {
  ready: "yellow",
  running: "teal",
  fault: "orange",
  wrong: "pink",
};

export const title = "Flow sensor";
