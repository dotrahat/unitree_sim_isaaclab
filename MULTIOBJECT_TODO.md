# Multi-object BrainCo env — manual verification TODO

Task: `Isaac-PickPlace-MultiObject-G123-Brainco-Joint`
Config file: `tasks/g1_tasks/pick_place_multiobject_g1_23dof_brainco/pickplace_multiobject_g1_23dof_brainco_env_cfg.py`

Objects added so far (in table order):
1. `object`   — tomato soup can (z=0.83, confirmed correct by you)
2. `object_2` — mustard bottle (z=0.83, **unconfirmed** — likely needs adjusting)
3. `object_3` — cracker box (z=0.83, **unconfirmed** — swapped in for the mac-n-cheese box, see note below)
4. `object_4` — banana (z=0.83, **unconfirmed**; physics was hand-authored by me since the downloaded asset had none — worth double-checking it grasps/behaves like a normal rigid body)
5. `object_5` — deformable tube (soft body — **not yet visually/hands-on tested at all**)

---

## Run commands

**Sim — now requires `--device cuda`, not `--device cpu`**, because the deformable tube (object_5) only works on GPU PhysX:
```bash
python sim_main.py --device cuda --enable_cameras --task Isaac-PickPlace-MultiObject-G123-Brainco-Joint --enable_brainco_dds --robot_type g123
```
(Drop `--headless` for the GUI window.)

**Teleop** (separate terminal, `tv-2` conda env):
```bash
conda activate tv-2
cd ~/xr_teleoperate/teleop
python teleop_hand_and_arm.py --arm=G1_23 --ee=brainco --sim --record
```
Then in the sim/teleop terminal press `r` to start following, `q` to stop. Open the vuer web UI as usual to watch the robot.

---

## [ ] 1. Check z-heights for objects 2–4
For each of mustard bottle / cracker box / banana: see if it's floating above or sinking into the table (same issue the soup can had at first). Adjust `pos=[x, y, z]` in the `init_state` for `object_2`, `object_3`, `object_4` in the env_cfg file above. Tell me the corrected z values and I'll update the file.

## [ ] 2. Visually confirm the banana looks/behaves right
I authored its rigid-body/collision physics myself (the downloaded asset had none) using a convex-hull collider. Check that:
- it doesn't clip through the table or other objects
- it can be grasped normally like the other rigid objects
- (texture should look correct — I fixed a broken-texture-path bug already, but worth a glance)

## [ ] 3. Check the deformable tube (object_5) — first time actually looking at this one
- Does it render/look like a tube, not some default/garbage material?
- Does it actually deform (sag, bend, squish) when the gripper touches it, rather than behaving like a rigid object?
- Any PhysX warnings/errors in the terminal specific to the soft body when you nudge or grasp it?
- Confirm the sim doesn't need `--device cpu` for anything else you rely on (e.g. if you had a reason to prefer CPU physics before, GPU is now mandatory for this specific task while the tube is in it)

## [ ] 4. Full teleop pass with all 5 objects
Grasp each object at least once via teleop (soup can, mustard bottle, cracker box, banana, tube) and confirm the BrainCo hand can pick each one up reasonably. Note any object that's clearly too big/small/slippery for the hand.

## [ ] 5. Decide: keep mac-n-cheese box, or is the cracker box substitute fine?
Your originally-downloaded `Food/mac_n_cheese_centered.usd` has no physics authored at all (visual-only asset), so I swapped in the YCB cracker box (`003_cracker_box.usd`) for the "box" object instead — same shape variety, but physics-ready. If you specifically want the mac-n-cheese box, I can author physics onto it the same way I did for the banana — just say so.

## [ ] 6. Decide: do you want a CPU-only variant of this task (without the tube)?
Since GPU is now mandatory for the full 5-object env, let me know if you also want a lighter/CPU-friendly variant (e.g. everything except the deformable tube) for quicker testing, or if `--device cuda` going forward is fine.

---

## Reference: what changed this session
- New task: `tasks/g1_tasks/pick_place_multiobject_g1_23dof_brainco/`
- Registered in: `tasks/g1_tasks/__init__.py`
- New/derived assets: `assets/objects/multiobject_assets/YCB/Axis_Aligned/011_banana_physics.usd` (physics-authored by me)
- Original cylinder task (`Isaac-PickPlace-Cylinder-G123-Brainco-Joint`) is untouched and still works.
