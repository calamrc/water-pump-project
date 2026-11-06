const question = {
  off: "Timer screen",
  yes: "Restart, Yes",
  no: "Restart, No",
  rebooted: "Rebooting",
};

const clock = {
  setting: "Setting the time",
  running: "Counting down",
  paused: "Paused",
  done: "Finished",
};

export const state = (v) => ({
  Question: question[v.dialog],
  Timer: clock[v.timer],
});

export const step = (action) => {
  const labels = {
    Hold: "you hold the button",
    Flip: "you turn the dial",
    Confirm: "you confirm restart",
    Cancel: "you cancel restart",
    Start: "you press start",
    Pause: "you press pause",
    Resume: "you press resume",
    Tick: "the countdown finishes",
    Done: null,
  };
  return labels[action] ?? null;
};

export const group = (v) => {
  if (v.dialog === "rebooted") return "reboot";
  if (v.dialog === "off") return "main";
  return "asking";
};

export const colors = {
  main: "teal",
  asking: "yellow",
  reboot: "orange",
};

export const title = "Restart question";
