const phaseName = {
  setting: "Setting the time",
  running: "Counting down",
  paused: "Paused",
  completed: "Finished",
};

export const state = (v) => {
  const owed = v.tick === "owed";
  const bad = v.corrupted === true || v.corrupted === "TRUE";
  if (bad) return `${phaseName[v.phase]}, countdown moved after it should have stopped`;
  if (owed && v.phase !== "running") return `${phaseName[v.phase]}, a tick is still writing`;
  if (owed) return "Counting down, a second is in flight";
  return phaseName[v.phase];
};

export const step = (action, from) => {
  const bad = from.corrupted === true || from.corrupted === "TRUE";
  if (bad) return action === "Reset" ? "you press reset" : null;
  const labels = {
    AdjustUp: null,
    AdjustDown: null,
    Start: "you press start",
    Pause: "you press pause",
    Resume: "you press resume",
    Reset: "you press reset",
    TickArm: "a second passes",
    TickCommit: "the tick writes the new second",
  };
  return labels[action] ?? null;
};

export const group = (v) => {
  const bad = v.corrupted === true || v.corrupted === "TRUE";
  if (bad || (v.tick === "owed" && v.phase !== "running")) return "wrong";
  return v.phase;
};

export const colors = {
  setting: "yellow",
  running: "teal",
  paused: "orange",
  completed: "purple",
  wrong: "pink",
};

export const title = "Countdown timer";
