export const chapters = [
  ["01", "なぜ二足立位は難しいのか", "01_why_biped_is_hard.md"],
  ["02", "倒立振子", "02_inverted_pendulum.md"],
  ["03", "PID / PDフィードバック", "03_pid_feedback.md"],
  ["04", "状態空間と状態フィードバック", "04_state_space.md"],
  ["05", "LQR", "05_lqr.md"],
  ["06", "支持多角形とZMP", "06_zmp_support_polygon.md"],
  ["07", "LIPMとステップ制御", "07_lipm_step_control.md"]
];

const chapterIdByFilename = new Map(
  chapters.map(([id, , filename]) => [filename, id])
);

function isUntouchedReference(value) {
  return !value
    || value.startsWith("#")
    || value.startsWith("//")
    || /^[a-z][a-z0-9+.-]*:/i.test(value);
}

export function resolveContentReference(value, baseUrl) {
  if (isUntouchedReference(value)) return value;

  const resolved = new URL(value, baseUrl);
  const filename = decodeURIComponent(resolved.pathname.split("/").pop() || "");
  const chapterId = chapterIdByFilename.get(filename);

  if (chapterId && resolved.pathname.includes("/theory/")) {
    return `#${chapterId}`;
  }

  return resolved.href;
}
