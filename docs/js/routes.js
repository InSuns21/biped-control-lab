export const chapters = [
  ["01", "なぜ二足立位は難しいのか", "01_why_biped_is_hard.md"],
  ["02", "倒立振子", "02_inverted_pendulum.md"],
  ["03", "PID / PDフィードバック", "03_pid_feedback.md"],
  ["04", "状態空間と状態フィードバック", "04_state_space.md"],
  ["05", "LQR", "05_lqr.md"],
  ["06", "支持多角形とZMP", "06_zmp_support_polygon.md"],
  ["07", "LIPMとステップ制御", "07_lipm_step_control.md"],
  ["S01", "補講 S01 — 常微分方程式と指数モード", "S01_ode_exponential_modes.md"],
  ["S02", "補講 S02 — 固有値とモード", "S02_eigenvalues_modes.md"],
  ["S03", "補講 S03 — Lyapunov安定性", "S03_lyapunov_stability.md"],
  ["S04", "補講 S04 — 状態推定とKalman filter", "S04_state_estimation_kalman.md"],
  ["S05", "補講 S05 — Lagrange法とロボット力学", "S05_lagrange_robot_dynamics.md"],
  ["S06", "補講 S06 — 接触・摩擦・CoP制約", "S06_contact_friction_constraints.md"],
  ["S07", "補講 S07 — Preview ControlとMPC", "S07_preview_mpc.md"],
  ["S08", "補講 S08 — Centroidal Dynamics", "S08_centroidal_dynamics.md"],
  ["S09", "補講 S09 — Whole-Body QP", "S09_whole_body_qp.md"]
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
