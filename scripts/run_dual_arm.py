#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys, time, yaml, numpy as np
import pybullet as p, pybullet_data as pd

# repo path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if REPO_ROOT not in sys.path: sys.path.insert(0, REPO_ROOT)

from src.simulation.environment import SimulationEnvironment
from src.simulation.robot_interface import RobotInterface
from src.control.trajectories import circle_lissajous_with_z
from src.vision.vision import (
    get_camera_matrices, render_rgbd, make_view_matrix,
    depth_to_linear, simple_green_seg, centroid_from_mask, Ema
)

CFG = os.path.join(REPO_ROOT, "config", "robots.yaml")

def last_movable_link(body, cid):
    idx = -1
    for j in range(p.getNumJoints(body, physicsClientId=cid)):
        jt = p.getJointInfo(body, j, physicsClientId=cid)[2]
        if jt in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC): idx = j
    return idx

def main():
    cid = p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0,0,-9.81, physicsClientId=cid)
    p.setTimeStep(1.0/240.0, physicsClientId=cid)
    with open(CFG, "r") as f: cfg = yaml.safe_load(f)

    env = SimulationEnvironment(gui=True, client_id=cid)

    # --- Load robots
    ra = env.load_robot_a(cfg["robot_a"]["type"], cfg["robot_a"]["base_xyz"], cfg["robot_a"]["base_rpy"])
    rb = env.load_robot_b_panda(cfg["robot_b"]["base_xyz"], cfg["robot_b"]["base_rpy"])

    robotA = RobotInterface(ra, cid, pos_kp=0.6, pos_kd=0.1, max_force=140.0)
    robotB = RobotInterface(rb, cid, nullspace=cfg["robot_b"]["nullspace"], pos_kp=0.55, pos_kd=0.1, max_force=180.0)

    ee_a  = last_movable_link(ra, cid)
    hand_b= last_movable_link(rb, cid)

    # --- Object and grasp
    oid = env.load_object_cube(xyz=(-0.6,0.0,0.76), rgba=(0.1,0.8,0.1,1.0), size=0.03)
    time.sleep(0.2)

    for tgt in (np.array([-0.6,0.0,0.88]), np.array([-0.6,0.0,0.80])):
        q = robotA.ik_position(ee_a, tgt)
        for _ in range(480): robotA.set_joint_positions(q, rate_limit=0.8); env.step(); time.sleep(1/240)

    p.createConstraint(ra, ee_a, oid, -1, p.JOINT_FIXED, [0,0,0],[0,0,0],[0,0,0], physicsClientId=cid)

    mid = np.array([-0.25,0.0,1.0])
    q_mid = robotA.ik_position(ee_a, mid)
    for _ in range(720): robotA.set_joint_positions(q_mid, rate_limit=0.8); env.step(); time.sleep(1/240)

    # --- Camera intrinsics
    cam = cfg["robot_b"]["camera"]
    W,H = cam["width"], cam["height"]
    proj, fx, fy, cx, cy = get_camera_matrices(W,H, cam["fov_deg"], cam["near"], cam["far"])

    # --- IBVS params (tamed)
    desired_depth = 0.70
    k_xy, k_z = 0.0008, 0.02
    ctr_f, depth_f = Ema(0.8), Ema(0.6)
    last_good_target, last_seen = None, time.time()
    loss_timeout_s = 1.2

    # --- Diagnostics
    t0 = time.time()
    print("[INFO] DOF A,B =", robotA.dof, robotB.dof, "  EE idx A,B =", ee_a, hand_b)

    while True:
        # quit on ESC
        ke = p.getKeyboardEvents()
        if 27 in ke and ke[27] & p.KEY_WAS_TRIGGERED: break

        t = time.time() - t0

        # --- Robot A motion (slow + small)
        obj_xyz = circle_lissajous_with_z(t, center=np.array([-0.25,0.0,1.0]), R=0.10, z_amp=0.03, z_freq=0.3)
        qA = robotA.ik_position(ee_a, obj_xyz)
        robotA.set_joint_positions(qA, rate_limit=0.8)

        # --- Wrist camera pose
        hand_pos, hand_orn = p.getLinkState(rb, hand_b, computeForwardKinematics=True, physicsClientId=cid)[4:6]
        R = np.array(p.getMatrixFromQuaternion(hand_orn)).reshape(3,3)
        cam_pos  = np.array(hand_pos) + R @ np.array([0,0,0.06])
        # IMPORTANT: in PyBullet camera looks along -Z of its frame
        cam_fwd  = R @ np.array([0,0,-1.0])
        cam_up   = R @ np.array([0,1,0])
        view = make_view_matrix(cam_pos.tolist(), (cam_pos+cam_fwd).tolist(), cam_up.tolist())

        # --- Render and detect
        rgb, depth_buf = render_rgbd(cid, view, proj, W, H)
        depth_lin = depth_to_linear(depth_buf, cam["near"], cam["far"])
        mask = simple_green_seg(rgb)
        ctr = centroid_from_mask(mask)
        ctr = ctr_f(ctr)

        target_world = None
        if ctr is not None:
            u, v = float(ctr[0]), float(ctr[1])
            d = float(depth_lin[int(np.clip(v,0,H-1)), int(np.clip(u,0,W-1))])
            if np.isfinite(d) and d > 0:
                d = depth_f(d)
                last_seen = time.time()
                e_u, e_v, e_z = (u-cx), (v-cy), (d-desired_depth)

                # correction in camera frame (small, proportional)
                corr_cam = np.array([
                    -k_xy*e_u * max(d,0.2)/fx,
                    -k_xy*e_v * max(d,0.2)/fy,
                    -k_z * e_z
                ])
                # map to world
                corr_world = R @ corr_cam
                target_world = cam_pos + corr_world + cam_fwd*(-desired_depth)
                last_good_target = target_world.copy()

        # --- Command B safely
        if last_good_target is not None and (time.time()-last_seen) < loss_timeout_s:
            qB = robotB.ik_position(hand_b, last_good_target.tolist())
            robotB.set_joint_positions(qB, rate_limit=0.7)
        else:
            # idle hold straight ahead
            hold = cam_pos + cam_fwd*(-desired_depth)
            q_idle = robotB.ik_position(hand_b, hold.tolist())
            robotB.set_joint_positions(q_idle, rate_limit=0.5)

        # --- occasional logs
        if t < 3.0 and int(t*4)%4==0:
            qa,_ = robotA.joint_states(); qb,_ = robotB.joint_states()
            print(f"[t={t:4.2f}] A_q0={qa[0]:+.2f}  B_q0={qb[0]:+.2f}  ctr={None if ctr is None else (int(ctr[0]),int(ctr[1]))}")

        env.step()
        time.sleep(1/240)

if __name__ == "__main__":
    main()
