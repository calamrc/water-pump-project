const names = {
  up: "Button up",
  downShort: "Held, under one second",
  downLong: "Held, past one second",
  holdSent: "Hold already sent",
};

export const state = (v) => names[v.phase];

export const step = (action, from, to) => {
  if (action === "Press") return "you press the button";
  if (action === "OneSecond") return "one second passes";
  if (action === "HoldFires") return "three seconds pass";
  if (action === "Release" && from.phase === "downShort") return "you let go";
  if (action === "Release" && from.phase === "downLong") return "you let go after one second";
  if (action === "Release" && from.phase === "holdSent") return "you let go after the hold";
  return null;
};

export const group = (v) => {
  if (v.phase === "up") return "up";
  if (v.phase === "holdSent") return "hold";
  return "down";
};

export const colors = {
  up: "tan",
  down: "yellow",
  hold: "orange",
};

export const title = "Button press";
