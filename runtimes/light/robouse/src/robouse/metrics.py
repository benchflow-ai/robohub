"""Episode metrics from the trusted per-step physical trace, and aggregate statistics with 95% confidence intervals.

Per episode (written to episode/result.json under `metrics` and to episode/physics.jsonl step by step):
  success               the trusted verdict (0/1)
  progress              fraction of the task's stages reached in order (backends with stages), else success
  execution_time_s      active duration: control steps used x control period (successful episodes only in aggregates)
  execution_speed       stages (or 1 task) completed per active minute
  smoothness_sparc      spectral arc length of the end-effector (or base) speed profile, fc = 10 Hz (<= 0; closer to 0
                        is smoother; unit-free; Balasubramanian et al. 2015)
  avg_jerk              mean norm of the end-effector jerk (m/s^3) after a 10 Hz low-pass
  path_length_m         end-effector (or base) path length
  max_contact_force_n   largest normal contact force on the robot's own geoms (MuJoCo backends)
  safety_events         count of safety events (falls, crashes, harm, e-stops, over-temperature stops)
  safe_failure          1 if the episode failed without any safety event (aggregated over failed episodes)
  excellent             success with no safety event and no vetoed motion (the automatic stand-in for a human's
                        "excellent" rating; aggregated as execution quality over successful episodes)
Navigation backends add spl, soft_spl, arrived, collisions, time_to_target_s, path_ratio, excess_turning_rad_per_m.

Aggregates (`robouse report`): proportions with Wilson score intervals (well defined at 0 % and 100 %), means with
percentile bootstrap intervals (2000 resamples, fixed seed), pass@k (unbiased estimator) over repeated seeds.
"""
from __future__ import annotations

import math

import numpy as np


# ------------------------------------------------------------------------------------------------------------------
# per-episode signal metrics
# ------------------------------------------------------------------------------------------------------------------

def _lowpass(x: np.ndarray, fs: float, fc: float = 10.0) -> np.ndarray:
    """Zero-phase FFT low-pass along axis 0 (no-op when the sampling rate cannot resolve fc)."""
    if len(x) < 4 or fs <= 2 * fc:
        return x
    X = np.fft.rfft(x, axis=0)
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    X[f > fc] = 0
    return np.fft.irfft(X, n=len(x), axis=0)


def sparc(speed: np.ndarray, fs: float, padlevel: int = 4, fc: float = 10.0, amp_th: float = 0.05) -> float | None:
    """Spectral arc length of a speed profile (Balasubramanian, Melendez-Calderon, Roby-Brami, Burdet 2015)."""
    v = np.asarray(speed, dtype=float)
    if len(v) < 4 or not np.any(v > 1e-9):
        return None
    nfft = int(2 ** (math.ceil(math.log2(len(v))) + padlevel))
    f = np.arange(0, fs, fs / nfft)
    M = np.abs(np.fft.fft(v, nfft))
    M = M / M.max()
    sel = f <= fc
    f_sel, M_sel = f[sel], M[sel]
    idx = np.nonzero(M_sel >= amp_th)[0]
    if len(idx) < 2:
        return -1.0
    f_sel, M_sel = f_sel[idx[0]:idx[-1] + 1], M_sel[idx[0]:idx[-1] + 1]
    span = f_sel[-1] - f_sel[0]
    if span <= 0:
        return -1.0
    return float(-np.sum(np.sqrt((np.diff(f_sel) / span) ** 2 + np.diff(M_sel) ** 2)))


def trajectory_metrics(points: list, dt: float) -> dict:
    """SPARC, mean jerk and path length of a sampled 3-D (or 2-D) point trajectory."""
    if not points or len(points) < 4 or not dt or dt <= 0:
        return {}
    p = np.asarray(points, dtype=float)
    fs = 1.0 / dt
    p = _lowpass(p, fs)
    vel = np.gradient(p, dt, axis=0)
    speed = np.linalg.norm(vel, axis=1)
    jerk = np.gradient(np.gradient(vel, dt, axis=0), dt, axis=0)
    s = sparc(speed, fs)
    return {"smoothness_sparc": None if s is None else round(s, 4),
            "avg_jerk": round(float(np.mean(np.linalg.norm(jerk, axis=1))), 4),
            "path_length_m": round(float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))), 4)}


def episode_metrics(result: dict, physics: list[dict], dt: float | None, progress: float | None, stages: int | None,
                    extra: dict | None = None) -> dict:
    ok = bool(result.get("success"))
    steps = int(result.get("steps_used") or 0)
    n_safety = len(result.get("safety_events") or [])
    vetoes = int(((result.get("session") or {}).get("vetoes")) or 0)
    m: dict = {"success": int(ok), "progress": progress if progress is not None else float(ok),
               "safety_events": n_safety, "safe_failure": None if ok else int(n_safety == 0),
               "excellent": int(ok and n_safety == 0 and vetoes == 0) if ok else None}
    if dt:
        t = steps * dt
        m["execution_time_s"] = round(t, 3)
        done_units = (progress or 0.0) * (stages or 1) if stages else float(ok)
        m["execution_speed"] = round(done_units / (t / 60.0), 4) if t > 0 else None
        m["control_dt_s"] = round(dt, 5)
    pts = [r["ee"] for r in physics if r.get("ee") is not None]
    if dt and pts:
        m.update(trajectory_metrics(pts, dt))
    forces = [r["f"] for r in physics if r.get("f") is not None]
    if forces:
        m["max_contact_force_n"] = round(float(max(forces)), 3)
    m.update(extra or {})
    return m


# ------------------------------------------------------------------------------------------------------------------
# contact force on the robot (MuJoCo)
# ------------------------------------------------------------------------------------------------------------------

ROBOT_WORDS = ("robot", "gripper", "hand", "finger", "pad", "panda", "wrist", "claw", "link", "arm", "leftpad", "rightpad",
               "tool", "jaw", "palm", "thumb", "foot", "calf", "thigh", "hip", "trunk", "torso", "pelvis", "base_link")


def robot_geoms(model) -> set[int]:
    """Geoms on the robot: bodies whose name (or an ancestor's) contains a robot part word."""
    import mujoco

    robot_bodies = set()
    for b in range(model.nbody):
        name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "").lower()
        if any(w in name for w in ROBOT_WORDS):
            robot_bodies.add(b)
    changed = True
    while changed:  # children of robot bodies are robot bodies
        changed = False
        for b in range(1, model.nbody):
            if b not in robot_bodies and int(model.body_parentid[b]) in robot_bodies and int(model.body_parentid[b]) != 0:
                robot_bodies.add(b)
                changed = True
    return {g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in robot_bodies}


def max_robot_contact_force(model, data, geoms: set[int]) -> float:
    import mujoco

    f6 = np.zeros(6)
    best = 0.0
    for i in range(int(data.ncon)):
        c = data.contact[i]
        if int(c.geom1) in geoms or int(c.geom2) in geoms:
            mujoco.mj_contactForce(model, data, i, f6)
            best = max(best, abs(float(f6[0])))
    return best


# ------------------------------------------------------------------------------------------------------------------
# aggregate statistics
# ------------------------------------------------------------------------------------------------------------------

def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float] | None:
    if n <= 0:
        return None
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


def bootstrap_mean(x: list[float], n_boot: int = 2000, seed: int = 0) -> tuple[float, float] | None:
    a = np.asarray([v for v in x if v is not None and not (isinstance(v, float) and math.isnan(v))], dtype=float)
    if len(a) == 0:
        return None
    if len(a) == 1:
        return (float(a[0]), float(a[0]))
    rng = np.random.default_rng(seed)
    means = a[rng.integers(0, len(a), size=(n_boot, len(a)))].mean(axis=1)
    return (float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5)))


def pass_at_k(n: int, c: int, k: int) -> float:
    """Unbiased pass@k (Chen et al. 2021): 1 - C(n-c, k) / C(n, k)."""
    if n - c < k:
        return 1.0
    return 1.0 - math.comb(n - c, k) / math.comb(n, k)


PROPORTIONS = ("success", "safe_failure", "excellent", "arrived", "vlm_agree")
MEANS = ("progress", "execution_time_s", "execution_speed", "smoothness_sparc", "avg_jerk", "max_contact_force_n",
         "path_length_m", "spl", "soft_spl", "collisions", "time_to_target_s", "path_ratio", "excess_turning_rad_per_m",
         "cost_usd", "prompt_tokens", "completion_tokens", "wall_time_s")
SUCCESS_ONLY = ("execution_time_s", "execution_speed")  # PAW: averaged over successful episodes


def aggregate(rows: list[dict]) -> dict:
    """rows: one dict of episode metrics per trial (plus cost and wall time). Returns {metric: {value, ci, n}}."""
    out: dict = {"n": len(rows)}
    for key in PROPORTIONS:
        vals = [r[key] for r in rows if r.get(key) is not None]
        if vals:
            k, n = int(sum(vals)), len(vals)
            lo_hi = wilson(k, n)
            out[key] = {"value": round(k / n, 4), "ci95": [round(lo_hi[0], 4), round(lo_hi[1], 4)], "n": n, "k": k}
    for key in MEANS:
        src = [r for r in rows if r.get("success")] if key in SUCCESS_ONLY else rows
        vals = [r[key] for r in src if r.get(key) is not None]
        if vals:
            ci = bootstrap_mean(vals)
            out[key] = {"value": round(float(np.mean(vals)), 4), "ci95": [round(ci[0], 4), round(ci[1], 4)], "n": len(vals)}
    if "safe_failure" in out:
        out["safe_failure"]["note"] = "share of failed episodes with no safety event"
    if "excellent" in out:
        out["execution_quality"] = {**out.pop("excellent"), "note": "share of successful episodes rated excellent"}
    return out
