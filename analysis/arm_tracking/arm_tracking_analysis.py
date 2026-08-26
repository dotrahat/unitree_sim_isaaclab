#!/usr/bin/env python3
"""
Quantitative arm-tracking analysis for G1-23 + BrainCo teleoperation episodes.

Question answered: "If my hand is at this x,y,z, does the robot's palm go to that
x,y,z -- and what is the error between desired and achieved?"

The error is reported as a three-way decomposition, POSITION (mm) and ORIENTATION (deg)
always reported SEPARATELY:

    DESIRED          the operator's requested wrist pose (actions.<side>_wrist_target_SE3),
                     a 4x4 SE3 already in the robot's waist frame in Unitree URDF
                     convention -- the exact matrix handed to solve_ik.
    IK-ACHIEVABLE    FK(actions.<side>_arm.qpos) = FK(sol_q), the best the 5-DoF arm's
                     IK could do with that request.
    ACHIEVED         FK(states.<side>_arm.qpos), where the arm actually ended up.

    TOTAL         = DESIRED       -> ACHIEVED       (the headline: hand vs palm)
    REACHABILITY  = DESIRED       -> IK-ACHIEVABLE  (5 DoF cannot represent the request)
    SERVO         = IK-ACHIEVABLE -> ACHIEVED       (the arm missed its own target)

Data source : xr_teleoperate EpisodeWriter JSON (teleop/utils/data/<task>/episode_XXXX/data.json)
              states.<side>_arm.qpos  = MEASURED joint position [rad], read back over DDS
              actions.<side>_arm.qpos = COMMANDED joint target  [rad] (sol_q)
              actions.<side>_wrist_target_SE3 = DESIRED wrist pose, flat row-major 4x4
                     (only present from episode_0025 onward)
FK model    : xr_teleoperate/assets/g1/mode10/g1_23dof_mode_10_with_brainco.urdf, reduced
              exactly as teleop/robot_control/robot_arm_ik.py:G1_23_ArmIK reduces it
              (legs + waist_yaw + all BrainCo finger joints locked -> nq=10, five per arm),
              with the L_ee/R_ee operational frames at <side>_wrist_roll_joint + [0.20,0,0] m.
              Same model and frame the IK solved against. G1_23_ArmIK does NOT apply
              scale_arms (commented out, robot_arm_ik.py:564), so DESIRED needs no unscaling.

Run:  conda activate tv-2 && python arm_tracking_analysis.py
"""
import os, json, glob, csv, datetime
import numpy as np
import pinocchio as pin
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_ROOT = "/home/cvl/xr_teleoperate/teleop/utils/data/pick cube"
URDF_PATH = "/home/cvl/xr_teleoperate/assets/g1/mode10/g1_23dof_mode_10_with_brainco.urdf"
URDF_DIR  = "/home/cvl/xr_teleoperate/assets/g1/mode10"
OUT_DIR   = os.path.dirname(os.path.abspath(__file__))

# INCLUSION RULE: only episodes whose dual-arm vector was stored natively as a correct
# 5 + 5 split. Every episode in this dataset is 5-DoF-per-arm G1-23 data, but episodes
# 0000-0021 stored it as two OVERLAPPING 7-element slices (q[:7] and q[-7:] of the
# 10-vector), so the stored right_arm[0:4] is a copy of left_arm[3:7]. That is losslessly
# reconstructible, but no reported number should rest on a repair step, so they are
# excluded. They stay in sessions.csv with included=False so the exclusion is auditable.

# Nominal record rate. There are no per-sample timestamps. The sim episodes carry a
# whole-episode sim_state._timestamp at 1-SECOND resolution; its slope over the long
# episodes is 29.8-29.9 Hz, which validates 30 Hz as the sample spacing.
FS_HZ = 30.0
DT    = 1.0 / FS_HZ

# Steady state: commanded joint speed below this, sustained for SETTLE_S beforehand, so a
# velocity zero-crossing at the top of a swing is not counted as "arrived".
STEADY_VEL_DEG_S = 2.0
SETTLE_S         = 0.5
SETTLE_N         = int(round(SETTLE_S * FS_HZ))

MAX_LAG_N = 60   # cross-correlation lag search, samples (0..2000 ms)

JOINTS  = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"]
Q_NAMES = [f"left_{j}" for j in JOINTS] + [f"right_{j}" for j in JOINTS]
TERMS   = ["total", "reachability", "servo"]


# ----------------------------------------------------------------------------- FK
def build_fk():
    robot = pin.RobotWrapper.BuildFromURDF(URDF_PATH, URDF_DIR)
    lock = [f"{s}_{j}_joint" for s in ("left", "right") for j in
            ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")] \
         + ["waist_yaw_joint"] \
         + [f"{s}_{f}_joint" for s in ("left", "right") for f in
            ("thumb_metacarpal", "thumb_proximal", "thumb_distal", "index_proximal",
             "index_distal", "middle_proximal", "middle_distal", "ring_proximal",
             "ring_distal", "pinky_proximal", "pinky_distal")]
    red = robot.buildReducedRobot([robot.model.getJointId(n) for n in lock],
                                  np.zeros(robot.model.nq))
    for tag, jn in (("L_ee", "left_wrist_roll_joint"), ("R_ee", "right_wrist_roll_joint")):
        red.model.addFrame(pin.Frame(tag, red.model.getJointId(jn),
                                     pin.SE3(np.eye(3), np.array([0.20, 0.0, 0.0])),
                                     pin.FrameType.OP_FRAME))
    order = [red.model.names[i] for i in range(1, red.model.njoints)]
    assert order == [f"{n}_joint" for n in Q_NAMES], f"joint order mismatch: {order}"
    return red.model, red.model.createData(), \
           {"left": red.model.getFrameId("L_ee"), "right": red.model.getFrameId("R_ee")}


def fk_palm(model, data, fid, Q):
    P = np.empty((len(Q), 3)); R = np.empty((len(Q), 3, 3))
    for i, q in enumerate(Q):
        pin.forwardKinematics(model, data, q); pin.updateFramePlacement(model, data, fid)
        P[i] = data.oMf[fid].translation; R[i] = data.oMf[fid].rotation
    return P, R


def geodesic_deg(Ra, Rb):
    """Angle of the relative rotation Ra^T Rb, per sample, in degrees."""
    Rrel = np.einsum("nji,njk->nik", Ra, Rb)
    return np.degrees(np.arccos(np.clip((np.trace(Rrel, axis1=1, axis2=2) - 1.0) / 2.0, -1, 1)))


# ----------------------------------------------------------------------------- loading
class IncompleteEpisode(Exception):
    """data.json is truncated -- almost always an episode still being recorded.

    The writer streams each item to the file as it arrives and only appends the closing
    "]}" when the episode is saved, so a live recording is valid JSON prefix but not
    valid JSON. Skip it rather than taking down the whole analysis.
    """


def load_episode(ep_dir):
    path = os.path.join(ep_dir, "data.json")
    try:
        blob = json.load(open(path))
    except json.JSONDecodeError as exc:
        raise IncompleteEpisode(f"{os.path.basename(ep_dir)}: {exc}") from exc
    items = blob["data"]; n = len(items)
    legacy = len(items[0]["states"]["left_arm"]["qpos"]) == 7

    S = np.array([it["states"]["left_arm"]["qpos"] + it["states"]["right_arm"]["qpos"]
                  for it in items], float) if not legacy else np.zeros((n, 10))
    A = np.array([it["actions"]["left_arm"]["qpos"] + it["actions"]["right_arm"]["qpos"]
                  for it in items], float) if not legacy else np.zeros((n, 10))

    # DESIRED wrist poses, only present from episode_0025 onward
    has_target = "left_wrist_target_SE3" in items[0].get("actions", {})
    T = None
    if has_target:
        T = {s: np.array([it["actions"][f"{s}_wrist_target_SE3"] for it in items],
                         float).reshape(n, 4, 4) for s in ("left", "right")}
        for s in ("left", "right"):   # integrity: must be valid SE3
            R = T[s][:, :3, :3]
            assert np.allclose(np.einsum("nij,nkj->nik", R, R), np.eye(3), atol=1e-4), \
                f"{ep_dir}: non-orthonormal rotation in {s} wrist target"

    return dict(name=os.path.basename(ep_dir), path=ep_dir, n=n, legacy=legacy,
                q_meas=S, q_cmd=A, T_des=T, has_target=has_target,
                is_sim=isinstance(items[0].get("sim_state"), dict),
                date=datetime.date.fromisoformat(blob["info"]["date"]),
                arm_reference_mode=items[0].get("actions", {}).get("arm_reference_mode", ""),
                stuck_meas=0 if legacy else int(np.sum(np.all(np.diff(S, axis=0) == 0, axis=1))),
                stuck_cmd=0 if legacy else int(np.sum(np.all(np.diff(A, axis=0) == 0, axis=1))))


# ----------------------------------------------------------------------------- stats
def stats(v, unit):
    """Descriptive stats for a 1-D magnitude array, keyed with the unit in the name."""
    return {f"median_{unit}": float(np.median(v)), f"mean_{unit}": float(v.mean()),
            f"rms_{unit}": float(np.sqrt((v ** 2).mean())),
            f"p95_{unit}": float(np.percentile(v, 95)), f"max_{unit}": float(v.max())}


def err_stats(e_deg):
    a = np.abs(e_deg)
    return dict(mae_deg=float(a.mean()), rms_deg=float(np.sqrt((e_deg ** 2).mean())),
                p95_deg=float(np.percentile(a, 95)), max_deg=float(a.max()), n=int(a.size))


def best_lag(cmd, meas, max_lag=MAX_LAG_N):
    """k >= 0 (samples) maximising corr(cmd[t-k], meas[t]). Returns (k, r)."""
    if cmd.std() < 1e-9 or meas.std() < 1e-9:
        return 0, float("nan")
    max_lag = min(max_lag, max(0, len(cmd) - 30))
    best_k, best_r = 0, -np.inf
    for k in range(0, max_lag + 1):
        a = cmd[:len(cmd) - k] if k else cmd
        b = meas[k:]
        a = a - a.mean(); b = b - b.mean()
        if a.std() < 1e-9 or b.std() < 1e-9:
            continue
        r = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
        if r > best_r:
            best_r, best_k = r, k
    return best_k, best_r


def steady_mask(cmd_deg):
    v = np.abs(np.gradient(cmd_deg, DT)); slow = v < STEADY_VEL_DEG_S
    m = slow.copy()
    for s in range(1, SETTLE_N + 1):
        m[s:] &= slow[:-s]
    m[:SETTLE_N] = False
    return m


# ----------------------------------------------------------------------------- main
def main():
    model, data, FID = build_fk()
    eps, skipped = [], []
    for d in sorted(glob.glob(os.path.join(DATA_ROOT, "episode_*"))):
        if not os.path.exists(os.path.join(d, "data.json")):
            continue
        try:
            eps.append(load_episode(d))
        except IncompleteEpisode as exc:
            skipped.append(os.path.basename(d))
            print(f"  SKIPPING {exc} -- truncated data.json (episode still recording?)")
    if skipped:
        print(f"  {len(skipped)} episode(s) skipped as incomplete: {', '.join(skipped)}\n")

    sessions, joint_rows, task_rows, reach_rows = [], [], [], []
    pooled_joint, pooled_task = {}, {}
    # decomposition-only pool: restricted to episodes carrying wrist targets so that
    # TOTAL, REACHABILITY and SERVO are always over IDENTICAL samples and the split
    # is internally consistent. pooled_task keeps every episode for the servo term.
    pooled_decomp = {}

    for ep in eps:
        env = "sim" if ep["is_sim"] else "physical"
        included = not ep["legacy"]
        ep["included"] = included
        sessions.append(dict(
            session=ep["name"], date=ep["date"].isoformat(), env=env, included=included,
            n_samples=ep["n"], duration_s=round(ep["n"] * DT, 1),
            has_wrist_target=ep["has_target"], arm_reference_mode=ep["arm_reference_mode"],
            stuck_measured_frames=ep["stuck_meas"], stuck_cmd_frames=ep["stuck_cmd"],
            has_per_sample_timestamps=False,
            excluded_reason="" if included else
                "stored as overlapping 7+7 slices of the 10-vector; not natively 5+5"))
        if not included:
            continue

        cmd_deg, meas_deg = np.degrees(ep["q_cmd"]), np.degrees(ep["q_meas"])

        # ---------- A. joint space
        for j in range(10):
            side, jname = ("left" if j < 5 else "right"), JOINTS[j % 5]
            c, m = cmd_deg[:, j], meas_deg[:, j]
            e0 = m - c
            k, r = best_lag(c, m)
            ek = (m[k:] - c[:len(c) - k]) if k else e0
            sm = steady_mask(c); ess = e0[sm]
            s0, sk = err_stats(e0), err_stats(ek)
            row = dict(session=ep["name"], env=env, arm=side, joint=jname, n_samples=ep["n"],
                       zerolag_mae_deg=s0["mae_deg"], zerolag_rms_deg=s0["rms_deg"],
                       zerolag_p95_deg=s0["p95_deg"], zerolag_max_deg=s0["max_deg"],
                       lag_k_ms=round(k * DT * 1000.0, 1), lag_xcorr_r=round(r, 4),
                       lag_censored=(k == min(MAX_LAG_N, max(0, ep["n"] - 30))),
                       lagcomp_mae_deg=sk["mae_deg"], lagcomp_rms_deg=sk["rms_deg"],
                       lagcomp_p95_deg=sk["p95_deg"], lagcomp_max_deg=sk["max_deg"],
                       steady_n_samples=int(sm.sum()))
            if sm.sum() >= 10:
                ss = err_stats(ess)
                row.update(steady_mae_deg=ss["mae_deg"], steady_rms_deg=ss["rms_deg"],
                           steady_p95_deg=ss["p95_deg"], steady_max_deg=ss["max_deg"],
                           steady_bias_deg=float(ess.mean()))
            else:
                row.update({k2: None for k2 in ("steady_mae_deg", "steady_rms_deg",
                                                "steady_p95_deg", "steady_max_deg",
                                                "steady_bias_deg")})
            joint_rows.append(row)
            pooled_joint.setdefault((env, side, jname), []).append(e0)
            pooled_joint.setdefault((env, side, jname, "steady"), []).append(ess)

        # ---------- B. task space, three-way, position and orientation SEPARATELY
        ep["task"] = {}
        for side in ("left", "right"):
            Pi, Ri = fk_palm(model, data, FID[side], ep["q_cmd"])    # IK-achievable
            Pa, Ra = fk_palm(model, data, FID[side], ep["q_meas"])   # achieved
            terms = {"servo": (Pi, Ri, Pa, Ra)}
            if ep["has_target"]:
                Pd, Rd = ep["T_des"][side][:, :3, 3], ep["T_des"][side][:, :3, :3]
                terms["total"] = (Pd, Rd, Pa, Ra)
                terms["reachability"] = (Pd, Rd, Pi, Ri)
            ep["task"][side] = dict(Pi=Pi, Pa=Pa,
                                    Pd=(ep["T_des"][side][:, :3, 3] if ep["has_target"] else None))
            for term, (Pf, Rf, Pt, Rt) in terms.items():
                dpos = np.linalg.norm(Pt - Pf, axis=1) * 1000.0   # mm
                dori = geodesic_deg(Rf, Rt)                       # deg
                row = dict(session=ep["name"], env=env, arm=side, error_term=term,
                           n_samples=ep["n"])
                row.update({f"pos_{k2}": v for k2, v in stats(dpos, "mm").items()})
                row.update({f"ori_{k2}": v for k2, v in stats(dori, "deg").items()})
                row["z_shortfall_mean_mm"] = float((Pf[:, 2] - Pt[:, 2]).mean() * 1000)
                row["z_shortfall_max_mm"]  = float((Pf[:, 2] - Pt[:, 2]).max() * 1000)
                task_rows.append(row)
                pooled_task.setdefault((env, side, term), []).append((dpos, dori))
                if ep["has_target"]:
                    pooled_decomp.setdefault((env, side, term), []).append((dpos, dori))

            # ---------- C. is the reachability error a frame offset or a real workspace limit?
            if ep["has_target"]:
                Pd = ep["T_des"][side][:, :3, 3]
                d = (Pi - Pd) * 1000.0
                bias = d.mean(0)
                reach = np.linalg.norm(Pd, axis=1) * 1000.0
                mag = np.linalg.norm(d, axis=1)
                qs = np.quantile(reach, [0, .25, .5, .75, 1.0])
                for i in range(4):
                    sel = (reach >= qs[i]) & (reach <= qs[i + 1])
                    reach_rows.append(dict(
                        session=ep["name"], arm=side, quartile=i + 1,
                        reach_lo_mm=round(qs[i], 1), reach_hi_mm=round(qs[i + 1], 1),
                        n_samples=int(sel.sum()),
                        ik_residual_median_mm=float(np.median(mag[sel])),
                        ik_residual_p95_mm=float(np.percentile(mag[sel], 95)),
                        constant_bias_x_mm=round(float(bias[0]), 2),
                        constant_bias_y_mm=round(float(bias[1]), 2),
                        constant_bias_z_mm=round(float(bias[2]), 2),
                        median_after_bias_removal_mm=float(np.median(
                            np.linalg.norm(d - bias, axis=1)))))

    # ---------- aggregates
    agg_joint = []
    for key, arrs in sorted(pooled_joint.items(), key=lambda kv: str(kv[0])):
        e = np.concatenate(arrs)
        if e.size < 10:
            continue
        s = err_stats(e)
        agg_joint.append(dict(env=key[0], arm=key[1], joint=key[2],
                              window="steady_state" if len(key) == 4 else "all_samples",
                              n_samples=s["n"], mae_deg=s["mae_deg"], rms_deg=s["rms_deg"],
                              p95_deg=s["p95_deg"], max_deg=s["max_deg"],
                              bias_deg=float(e.mean())))
    agg_task = []
    for (env, side, term), lst in sorted(pooled_task.items()):
        dp = np.concatenate([a[0] for a in lst]); do = np.concatenate([a[1] for a in lst])
        row = dict(env=env, arm=side, error_term=term, n_samples=int(dp.size))
        row.update({f"pos_{k}": v for k, v in stats(dp, "mm").items()})
        row.update({f"ori_{k}": v for k, v in stats(do, "deg").items()})
        agg_task.append(row)

    # ---------- write
    def dump(path, rows):
        if rows:
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader(); w.writerows(rows)

    for nm in skipped:
        sessions.append(dict(session=nm, date="", env="", included=False, n_samples=0,
                             duration_s=0.0, has_wrist_target=False, arm_reference_mode="",
                             stuck_measured_frames=0, stuck_cmd_frames=0,
                             has_per_sample_timestamps=False,
                             excluded_reason="data.json truncated -- episode still recording "
                                             "or the run was interrupted before save"))
    sessions.sort(key=lambda r: r["session"])
    dump(os.path.join(OUT_DIR, "sessions.csv"), sessions)
    dump(os.path.join(OUT_DIR, "metrics_joint.csv"), joint_rows)
    dump(os.path.join(OUT_DIR, "metrics_taskspace.csv"), task_rows)
    dump(os.path.join(OUT_DIR, "metrics_aggregate_joint.csv"), agg_joint)
    dump(os.path.join(OUT_DIR, "metrics_aggregate_taskspace.csv"), agg_task)
    dump(os.path.join(OUT_DIR, "metrics_ik_reach.csv"), reach_rows)
    with open(os.path.join(OUT_DIR, "metrics.json"), "w") as f:
        json.dump(dict(
            generated=datetime.datetime.now().isoformat(timespec="seconds"),
            data_root=DATA_ROOT, fk_urdf=URDF_PATH, assumed_sample_rate_hz=FS_HZ,
            steady_state_def=dict(cmd_speed_below_deg_s=STEADY_VEL_DEG_S,
                                  sustained_for_s=SETTLE_S),
            error_terms=dict(
                total="DESIRED (operator wrist pose) -> ACHIEVED (FK of measured joints)",
                reachability="DESIRED -> IK-ACHIEVABLE (FK of sol_q); 5 DoF cannot represent the request",
                servo="IK-ACHIEVABLE -> ACHIEVED; the arm missed its own target"),
            not_available_from_recorded_data=[
                "end-to-end latency: no per-sample timestamps; the only clock is "
                "sim_state._timestamp, sim-only, 1-second resolution, single process.",
                "total/reachability for episodes before 0025: wrist targets were not "
                "recorded, so only the servo term is computable there."],
            skipped_incomplete_episodes=skipped,
            sessions=sessions, per_session_joint=joint_rows, per_session_taskspace=task_rows,
            ik_reach_dependence=reach_rows,
            aggregate_joint=agg_joint, aggregate_taskspace=agg_task), f, indent=2)

    make_plots(eps, agg_joint, pooled_task, pooled_decomp, reach_rows)
    print(f"wrote outputs to {OUT_DIR}")


# ----------------------------------------------------------------------------- plots
COL  = {"total": "#c1121f", "reachability": "#e07a00", "servo": "#0353a4",
        "sim": "#0353a4", "physical": "#2a9d8f"}
LBL  = {"total": "TOTAL  desired → achieved",
        "reachability": "reachability  (5-DoF IK limit)",
        "servo": "servo  (arm vs its own target)"}


def _style():
    plt.rcParams.update({"font.size": 17, "axes.titlesize": 21, "axes.labelsize": 18,
                         "xtick.labelsize": 15, "ytick.labelsize": 15, "legend.fontsize": 15,
                         "figure.dpi": 220, "savefig.dpi": 220, "axes.grid": True,
                         "grid.alpha": 0.3, "lines.linewidth": 2.4})


def make_plots(eps, agg_joint, pooled_task, pooled_decomp, reach_rows):
    _style()
    tgt = [e for e in eps if e["included"] and e["has_target"]]
    inc = [e for e in eps if e["included"]]

    # ---- Plot 1: desired vs IK-achievable vs achieved palm height
    if tgt:
        e = max(tgt, key=lambda x: x["task"]["left"]["Pd"][:, 2].ptp())
        fig, axes = plt.subplots(1, 2, figsize=(21, 8))
        T = e["task"]; t = np.arange(e["n"]) * DT
        for ax, side in zip(axes, ("left", "right")):
            ax.plot(t, T[side]["Pd"][:, 2]*1000, color=COL["total"], ls="--",
                    label="DESIRED  (your hand)")
            ax.plot(t, T[side]["Pi"][:, 2]*1000, color=COL["reachability"],
                    label="IK-ACHIEVABLE  (best 5-DoF can do)")
            ax.plot(t, T[side]["Pa"][:, 2]*1000, color=COL["servo"],
                    label="ACHIEVED  (robot palm)")
            ax.set_title(f"{side} arm"); ax.set_xlabel("time [s]")
            ax.legend(loc="best", framealpha=.93, fontsize=14)
        axes[0].set_ylabel("palm height z [mm]  (waist frame)")
        fig.suptitle(f"Desired vs achieved palm height — {e['name']} "
                     f"(n={e['n']:,}, {e['n']*DT:.0f} s)", fontsize=22)
        fig.tight_layout(); fig.savefig(f"{OUT_DIR}/plot1_palm_height.png"); plt.close(fig)

    # ---- Plot 2: per-joint tracking error, sim vs physical
    fig, axes = plt.subplots(1, 2, figsize=(19, 8), sharey=True)
    x = np.arange(len(JOINTS)); w = 0.36
    for ax, side in zip(axes, ("left", "right")):
        for i, env in enumerate(("sim", "physical")):
            mae, p95, ns = [], [], []
            for j in JOINTS:
                r = [a for a in agg_joint if (a["env"], a["arm"], a["joint"], a["window"])
                     == (env, side, j, "all_samples")]
                mae.append(r[0]["mae_deg"] if r else np.nan)
                p95.append(r[0]["p95_deg"] if r else np.nan)
                ns.append(r[0]["n_samples"] if r else 0)
            err = np.array(p95) - np.array(mae)
            ax.bar(x + (i - .5)*w, mae, w, yerr=[np.zeros_like(err), err], capsize=6,
                   color=COL[env], label=f"{env} (n={max(ns):,}/joint)", error_kw=dict(lw=1.8))
        ax.set_xticks(x); ax.set_xticklabels([j.replace("_", "\n") for j in JOINTS])
        ax.set_title(f"{side} arm"); ax.set_xlabel("joint")
    axes[0].set_ylabel("joint tracking error [deg]\nbar = MAE, whisker to p95")
    axes[0].legend()
    fig.suptitle("Per-joint arm tracking error (commanded vs measured, zero-lag)", fontsize=22)
    fig.tight_layout(); fig.savefig(f"{OUT_DIR}/plot2_per_joint_error.png"); plt.close(fig)

    # ---- Plot 3: CDF of the three error terms, POSITION and ORIENTATION separately
    fig, axes = plt.subplots(1, 2, figsize=(20, 8.5))
    for ax, (idx, unit, lab) in zip(axes, [(0, "mm", "position error [mm]"),
                                           (1, "deg", "orientation error [deg]")]):
        for term in TERMS:
            for side, ls in (("left", "-"), ("right", "--")):
                lst = pooled_decomp.get(("physical", side, term))
                if not lst: continue
                d = np.sort(np.concatenate([a[idx] for a in lst]))
                ax.plot(d, np.arange(1, d.size+1)/d.size*100, ls=ls, color=COL[term],
                        label=f"{LBL[term]} — {side} (n={d.size:,})")
        ax.set_xscale("log"); ax.set_xlabel(f"{lab}  (log scale)")
        for y, nm in ((50, "median"), (95, "p95")):
            ax.axhline(y, color="grey", lw=1.1, ls=":"); ax.text(ax.get_xlim()[0]*1.1, y+1, nm, color="grey")
        ax.legend(fontsize=12, loc="lower right")
    axes[0].set_ylabel("percentage of samples at or below [%]")
    fig.suptitle("Where the error comes from — position and orientation, reported separately",
                 fontsize=22)
    fig.tight_layout(); fig.savefig(f"{OUT_DIR}/plot3_error_cdf.png"); plt.close(fig)

    # ---- Plot 4: the decomposition as bars, position | orientation
    if tgt:
        fig, axes = plt.subplots(1, 2, figsize=(20, 8))
        for ax, (pre, unit, lab) in zip(axes, [("pos", "mm", "position error [mm]"),
                                               ("ori", "deg", "orientation error [deg]")]):
            xs = np.arange(len(TERMS)); w = 0.36
            for i, side in enumerate(("left", "right")):
                med, p95 = [], []
                for term in TERMS:
                    lst = pooled_decomp.get(("physical", side, term))
                    v = np.concatenate([a[0 if pre == "pos" else 1] for a in lst])
                    med.append(np.median(v)); p95.append(np.percentile(v, 95))
                err = np.array(p95) - np.array(med)
                ax.bar(xs + (i-.5)*w, med, w, yerr=[np.zeros_like(err), err], capsize=7,
                       color=[COL[t] for t in TERMS], alpha=1.0 if i == 0 else 0.5,
                       edgecolor="k", linewidth=0.8, error_kw=dict(lw=1.8))
                for xi, mv, pv in zip(xs + (i-.5)*w, med, p95):
                    ax.text(xi, pv, f"  {mv:.1f}", ha="center", va="bottom", fontsize=14)
            n = np.concatenate([a[0] for a in pooled_decomp[("physical","left",TERMS[0])]]).size
            ax.set_xlim(-0.6, len(TERMS)-0.4)
            ax.margins(y=0.13)
            ax.set_xticks(xs); ax.set_xticklabels(["TOTAL\ndesired→achieved",
                                                   "reachability\n(5-DoF IK)", "servo\n(arm)"])
            ax.set_ylabel(lab); ax.set_title(lab.split(" [")[0].capitalize())
            from matplotlib.patches import Patch
            ax.legend(handles=[Patch(facecolor="grey", edgecolor="k", label="left arm"),
                               Patch(facecolor="grey", edgecolor="k", alpha=0.5, label="right arm")],
                      fontsize=14, loc="upper right")
        _n = sum(e["n"] for e in tgt)
        fig.suptitle(f"Desired → achieved error, decomposed  (bar = median, whisker to p95)\n"
                     f"{', '.join(e['name'] for e in tgt)} — n={_n:,} samples per arm, "
                     f"all three terms over identical samples", fontsize=19)
        fig.tight_layout(); fig.savefig(f"{OUT_DIR}/plot4_decomposition.png"); plt.close(fig)

    # ---- Plot 5: IK residual vs reach distance
    if reach_rows:
        fig, ax = plt.subplots(figsize=(13, 8))
        for side, mk in (("left", "o"), ("right", "s")):
            rr = [r for r in reach_rows if r["arm"] == side]
            xc = [(r["reach_lo_mm"] + r["reach_hi_mm"]) / 2 for r in rr]
            ax.plot(xc, [r["ik_residual_median_mm"] for r in rr], marker=mk, ms=11,
                    color=COL["reachability"] if side == "left" else COL["total"],
                    label=f"{side} arm (median)")
        ax.set_xlabel("distance of the requested pose from the waist [mm]")
        ax.set_ylabel("IK reachability residual [mm]")
        ax.set_title("The 5-DoF limit is real: the error grows with reach\n"
                     "(a constant frame offset would be flat)", fontsize=19)
        ax.legend()
        fig.tight_layout(); fig.savefig(f"{OUT_DIR}/plot5_ik_vs_reach.png"); plt.close(fig)


if __name__ == "__main__":
    main()
