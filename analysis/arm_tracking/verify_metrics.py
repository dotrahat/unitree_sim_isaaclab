"""Independent verification of arm_tracking_analysis.py.

Deliberately shares no code with it: rebuilds the pinocchio model from scratch, reloads
the raw JSON, and re-implements every reported statistic, then asserts the results match
the committed CSVs exactly. Also runs FK sanity checks (zero-pose symmetry, mirrored-
configuration symmetry, reach envelope).

Run:  conda activate tv-2 && python verify_metrics.py
Exits non-zero if any aggregate disagrees with the CSV."""
import json, csv, os, sys, numpy as np, pinocchio as pin

D = "/home/cvl/xr_teleoperate/teleop/utils/data/pick cube"
OUT = "/home/cvl/unitree_sim_isaaclab/analysis/arm_tracking"

# ---- raw load, no helper functions
def raw(ep):
    items = json.load(open(f"{D}/{ep}/data.json"))["data"]
    S = np.array([it["states"]["left_arm"]["qpos"] + it["states"]["right_arm"]["qpos"] for it in items])
    A = np.array([it["actions"]["left_arm"]["qpos"] + it["actions"]["right_arm"]["qpos"] for it in items])
    return S, A

# ---- independent FK, built from scratch here
robot = pin.RobotWrapper.BuildFromURDF(
    "/home/cvl/xr_teleoperate/assets/g1/mode10/g1_23dof_mode_10_with_brainco.urdf",
    "/home/cvl/xr_teleoperate/assets/g1/mode10")
lock = [f"{s}_{j}_joint" for s in ("left","right") for j in
        ("hip_pitch","hip_roll","hip_yaw","knee","ankle_pitch","ankle_roll")] + ["waist_yaw_joint"] + \
       [f"{s}_{f}_joint" for s in ("left","right") for f in
        ("thumb_metacarpal","thumb_proximal","thumb_distal","index_proximal","index_distal",
         "middle_proximal","middle_distal","ring_proximal","ring_distal","pinky_proximal","pinky_distal")]
red = robot.buildReducedRobot([robot.model.getJointId(n) for n in lock], np.zeros(robot.model.nq))
for tag, jn in (("L_ee","left_wrist_roll_joint"), ("R_ee","right_wrist_roll_joint")):
    red.model.addFrame(pin.Frame(tag, red.model.getJointId(jn),
                                 pin.SE3(np.eye(3), np.array([0.20,0.,0.])), pin.FrameType.OP_FRAME))
dat = red.model.createData()
FID = {"left": red.model.getFrameId("L_ee"), "right": red.model.getFrameId("R_ee")}

def fk(Q, side):
    P = np.empty((len(Q),3)); R = np.empty((len(Q),3,3))
    for i,q in enumerate(Q):
        pin.forwardKinematics(red.model, dat, q); pin.updateFramePlacement(red.model, dat, FID[side])
        P[i] = dat.oMf[FID[side]].translation; R[i] = dat.oMf[FID[side]].rotation
    return P, R

print("="*78); print("FK SANITY CHECKS"); print("="*78)
print("reduced nq =", red.model.nq, "| joints:", [red.model.names[i] for i in range(1,red.model.njoints)])
z = np.zeros(10)
for s in ("left","right"):
    P,_ = fk(z[None,:], s); print(f"  q=0  {s:5} palm (pelvis frame) = {np.round(P[0],4)} m")
# mirror test: mirrored joint config must give mirrored palm (y flips)
qm = np.array([0.3,0.4,0.2,-0.5,0.1, 0.3,-0.4,-0.2,-0.5,-0.1])
Pl,_ = fk(qm[None,:],"left"); Pr,_ = fk(qm[None,:],"right")
print(f"  mirror test: L={np.round(Pl[0],4)}  R={np.round(Pr[0],4)}  "
      f"-> x,z match={np.allclose(Pl[0][[0,2]],Pr[0][[0,2]],atol=1e-6)}, y sign flip={np.allclose(Pl[0][1],-Pr[0][1],atol=1e-6)}")
# reach envelope
rng = np.random.default_rng(0); Qr = rng.uniform(-1.5,1.5,(400,10))
Pr_,_ = fk(Qr,"left"); d = np.linalg.norm(Pr_,axis=1)
print(f"  palm distance from pelvis over 400 random configs: {d.min()*1000:.0f}-{d.max()*1000:.0f} mm (plausible for a G1 arm+0.2m offset)")

print(); print("="*78); print("RECOMPUTED vs COMMITTED CSV"); print("="*78)
csv_joint = {(r["env"],r["arm"],r["joint"],r["window"]): r for r in csv.DictReader(open(f"{OUT}/metrics_aggregate_joint.csv"))}
csv_palm  = {(r["env"],r["arm"]): r for r in csv.DictReader(open(f"{OUT}/metrics_aggregate_palm.csv"))}
JN = ["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow","wrist_roll"]
GRP = {"physical":["episode_0022"], "sim":["episode_0023","episode_0024"]}

bad = 0
for env, eps in GRP.items():
    Ss = [raw(e) for e in eps]
    for side, off in (("left",0),("right",5)):
        # ---- joint stats, recomputed
        for j in range(5):
            e_all = np.concatenate([np.degrees(S[:,off+j]-A[:,off+j]) for S,A in Ss])
            ref = csv_joint[(env,side,JN[j],"all_samples")]
            for name, mine in (("mae",np.abs(e_all).mean()), ("rms",np.sqrt((e_all**2).mean())),
                               ("p95",np.percentile(np.abs(e_all),95)), ("max",np.abs(e_all).max()),
                               ("bias",e_all.mean())):
                theirs = float(ref[f"{name}_deg"])
                if abs(mine-theirs) > 1e-9: bad += 1; print(f"  MISMATCH {env} {side} {JN[j]} {name}: {mine} vs {theirs}")
            if int(ref["n_samples"]) != e_all.size: bad += 1; print(f"  MISMATCH n {env} {side} {JN[j]}")
        # ---- palm stats, recomputed
        dps, dos, dzs = [], [], []
        for S,A in Ss:
            Pc,Rc = fk(A,side); Pm,Rm = fk(S,side)
            dps.append(np.linalg.norm(Pm-Pc,axis=1)*1000)
            Rrel = np.einsum("nji,njk->nik",Rc,Rm)
            dos.append(np.degrees(np.arccos(np.clip((np.trace(Rrel,axis1=1,axis2=2)-1)/2,-1,1))))
            dzs.append((Pc[:,2]-Pm[:,2])*1000)
        dp=np.concatenate(dps); do=np.concatenate(dos); dz=np.concatenate(dzs)
        ref = csv_palm[(env,side)]
        for name, mine in (("pos_mae_mm",dp.mean()), ("pos_rms_mm",np.sqrt((dp**2).mean())),
                           ("pos_p95_mm",np.percentile(dp,95)), ("pos_max_mm",dp.max()),
                           ("ori_mae_deg",do.mean()), ("ori_p95_deg",np.percentile(do,95)),
                           ("z_shortfall_mean_mm",dz.mean()), ("z_shortfall_max_mm",dz.max())):
            theirs = float(ref[name])
            if abs(mine-theirs) > 1e-9: bad += 1; print(f"  MISMATCH {env} {side} {name}: {mine} vs {theirs}")
        print(f"  {env:9}{side:6} recomputed palm: MAE {dp.mean():6.2f} mm  p95 {np.percentile(dp,95):6.2f}  "
              f"max {dp.max():6.2f}  ori MAE {do.mean():5.2f} deg  n={dp.size}")

print(f"\n  >>> {'ALL AGGREGATE VALUES MATCH THE CSV EXACTLY' if bad==0 else f'{bad} MISMATCHES'} <<<")
if bad: sys.exit(1)

# ---- pooled physical headline, recomputed
S,A = raw("episode_0022"); allp=[]
for side,off in (("left",0),("right",5)):
    Pc,_=fk(A,side); Pm,_=fk(S,side); allp.append(np.linalg.norm(Pm-Pc,axis=1)*1000)
a=np.concatenate(allp)
print(f"\n  physical pooled (n={a.size}): median {np.median(a):.2f}  mean {a.mean():.2f}  "
      f"rms {np.sqrt((a**2).mean()):.2f}  p95 {np.percentile(a,95):.2f}  p99 {np.percentile(a,99):.2f}  max {a.max():.2f} mm")
