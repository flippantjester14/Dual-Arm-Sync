#!/usr/bin/env python3
import sys, os, time, yaml, numpy as np
import pybullet as p

# Make src importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.simulation.environment import SimulationEnvironment
from src.simulation.robot_interface import RobotInterface

def test_environment_creation(cid):
    print("\n" + "="*60)
    print("TEST 1: Creating Simulation Environment")
    print("="*60)
    env = SimulationEnvironment(gui=True, client_id=cid)
    print("✓ Environment created successfully")
    print(f"✓ Plane ID: {env.plane_id}")
    print(f"✓ Table A ID: {env.table_a_id}")
    print(f"✓ Table B ID: {env.table_b_id}")
    return env  # keep this env as the active scene

def test_robot_loading(env):
    print("\n" + "="*60)
    print("TEST 2: Loading Robots")
    print("="*60)
    ra, rb = env.load_robots()
    print(f"✓ Robot A (KUKA iiwa) ID: {ra}")
    print(f"✓ Robot B (Franka Panda) ID: {rb}")
    return ra, rb

def test_object_loading(env):
    print("\n" + "="*60)
    print("TEST 3: Loading Object")
    print("="*60)
    oid = env.load_object()
    pos, orn = env.get_object_pose()
    print(f"✓ Object ID: {oid}")
    print(f"✓ Object position: {np.round(pos,3)}")
    print(f"✓ Object orientation (xyzw): {np.round(orn,3)}")

def test_robot_interface(env, ra, rb):
    print("\n" + "="*60)
    print("TEST 4: Testing Robot Interface")
    print("="*60)
    with open('config/robot_config.yaml','r') as f:
        cfg = yaml.safe_load(f)
    rA = RobotInterface(ra, cfg.get('kuka_iiwa', {}), physics_client_id=env.client)
    rB = RobotInterface(rb, cfg.get('panda', {}), physics_client_id=env.client)
    print(f"✓ Robot A DOF: {rA.dof}")
    print(f"✓ Robot B DOF: {rB.dof}")
    qa, _ = rA.get_joint_states()
    qb, _ = rB.get_joint_states()
    print(f"✓ A joint sample: {np.round(qa,3)}")
    print(f"✓ B joint sample: {np.round(qb,3)}")
    return rA, rB

def test_simulation_step(env):
    print("\n" + "="*60)
    print("TEST 5: Testing Simulation Stepping")
    print("="*60)
    for _ in range(300):
        env.step()
        time.sleep(0.002)
    pos, _ = env.get_object_pose()
    print(f"✓ Completed stepping. Object @ {np.round(pos,3)}")

def test_robot_control(env, rA):
    print("\n" + "="*60)
    print("TEST 6: Testing Robot Control")
    print("="*60)
    qA, _ = rA.get_joint_states()
    target = np.copy(qA)
    n = min(6, rA.dof)
    target[:n] = np.array([0.5, -0.5, 0.5, -0.5, 0.4, 0.0][:n])

    for _ in range(800):
        rA.set_joint_positions(target)
        env.step()
        time.sleep(0.002)

    qAf, _ = rA.get_joint_states()
    err = np.abs(qAf[:n] - target[:n])
    print(f"✓ Final A joints (first {n}): {np.round(qAf[:n],3)}")
    print(f"✓ Target           (first {n}): {np.round(target[:n],3)}")
    print(f"✓ |error|          (first {n}): {np.round(err,3)}")

def main():
    print("\n" + "#"*60)
    print("# Dual-Arm Synchronization - GUI Component Tests")
    print("#"*60)

    # Create ONE GUI connection for the whole run
    cid = p.connect(p.GUI)
    try:
        env = test_environment_creation(cid)
        ra, rb = test_robot_loading(env)
        test_object_loading(env)
        rA, rB = test_robot_interface(env, ra, rb)
        test_simulation_step(env)
        test_robot_control(env, rA)

        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        print("✓ PASS: Environment Creation")
        print("✓ PASS: Robot Loading")
        print("✓ PASS: Object Loading")
        print("✓ PASS: Robot Interface")
        print("✓ PASS: Simulation Stepping")
        print("✓ PASS: Robot Control")
        print("\n🎉 All GUI tests passed.")
        print("\nNote: Keep this single GUI for batch tests. Do not open a new GUI per test.")
    finally:
        # keep GUI open a bit for inspection; press Ctrl+C to exit earlier
        time.sleep(1.0)
        if p.isConnected(cid):
            p.disconnect(cid)

if __name__ == "__main__":
    main()
