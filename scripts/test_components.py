#!/usr/bin/env python3
import sys, os, time, yaml, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.simulation.environment import SimulationEnvironment
from src.simulation.robot_interface import RobotInterface

GUI = False   # keep tests in DIRECT mode

def test_environment_creation():
    print("\n" + "="*60)
    print("TEST 1: Creating Simulation Environment")
    print("="*60)
    try:
        env = SimulationEnvironment(gui=GUI)
        print("✓ Environment created successfully")
        print(f"✓ Plane ID: {env.plane_id}")
        print(f"✓ Table A ID: {env.table_a_id}")
        print(f"✓ Table B ID: {env.table_b_id}")
        env.close()
        return True
    except Exception as e:
        print(f"✗ Failed to create environment: {e}")
        return False

def test_robot_loading():
    print("\n" + "="*60)
    print("TEST 2: Loading Robots")
    print("="*60)
    try:
        env = SimulationEnvironment(gui=GUI)
        ra, rb = env.load_robots()
        print(f"✓ Robot A (KUKA iiwa) ID: {ra}")
        print(f"✓ Robot B (Franka Panda) ID: {rb}")
        env.close()
        return True
    except Exception as e:
        print(f"✗ Failed to load robots: {e}")
        return False

def test_object_loading():
    print("\n" + "="*60)
    print("TEST 3: Loading Object")
    print("="*60)
    try:
        env = SimulationEnvironment(gui=GUI)
        env.load_robots()
        oid = env.load_object()
        pos, orn = env.get_object_pose()
        print(f"✓ Object ID: {oid}")
        print(f"✓ Object position: {np.round(pos,3)}")
        print(f"✓ Object orientation (xyzw): {np.round(orn,3)}")
        env.close()
        return True
    except Exception as e:
        print(f"✗ Failed to load object: {e}")
        return False

def test_robot_interface():
    print("\n" + "="*60)
    print("TEST 4: Testing Robot Interface")
    print("="*60)
    try:
        env = SimulationEnvironment(gui=GUI)
        ra, rb = env.load_robots()
        with open('config/robot_config.yaml','r') as f:
            cfg = yaml.safe_load(f)

        rA = RobotInterface(ra, cfg.get('kuka_iiwa', {}), physics_client_id=env.client)
        rB = RobotInterface(rb, cfg.get('panda', {}), physics_client_id=env.client)

        print(f"✓ Robot A DOF: {rA.dof}")
        print(f"✓ Robot B DOF: {rB.dof}")
        qa, dqa = rA.get_joint_states()
        qb, dqb = rB.get_joint_states()
        print(f"✓ A joint sample: {np.round(qa,3)}")
        print(f"✓ B joint sample: {np.round(qb,3)}")
        env.close()
        return True
    except Exception as e:
        print(f"✗ Failed robot interface test: {e}")
        return False

def test_simulation_step():
    print("\n" + "="*60)
    print("TEST 5: Testing Simulation Stepping")
    print("="*60)
    try:
        env = SimulationEnvironment(gui=GUI)
        env.load_robots()
        env.load_object()
        steps = 0
        for _ in range(300):  # ~1.25s at 240 Hz
            env.step()
            steps += 1
            time.sleep(0.001)
        pos, _ = env.get_object_pose()
        print(f"✓ Completed {steps} steps. Object @ {np.round(pos,3)}")
        env.close()
        return True
    except Exception as e:
        print(f"✗ Failed simulation step test: {e}")
        return False

def test_robot_control():
    print("\n" + "="*60)
    print("TEST 6: Testing Robot Control")
    print("="*60)
    try:
        env = SimulationEnvironment(gui=GUI)
        ra, rb = env.load_robots()
        with open('config/robot_config.yaml','r') as f:
            cfg = yaml.safe_load(f)
        rA = RobotInterface(ra, cfg.get('kuka_iiwa', {}), physics_client_id=env.client)

        # Build a target vector matching A's DOF
        qA, _ = rA.get_joint_states()
        target = np.copy(qA)
        n = min(6, rA.dof)
        target[:n] = np.array([0.5, -0.5, 0.5, -0.5, 0.4, 0.0][:n])

        for _ in range(600):  # ~2.5s
            rA.set_joint_positions(target)
            env.step()
            time.sleep(0.001)

        qAf, _ = rA.get_joint_states()
        err = np.abs(qAf[:n] - target[:n])
        print(f"✓ Final A joints (first {n}): {np.round(qAf[:n],3)}")
        print(f"✓ Target           (first {n}): {np.round(target[:n],3)}")
        print(f"✓ |error|          (first {n}): {np.round(err,3)}")
        env.close()
        return True
    except Exception as e:
        print(f"✗ Failed robot control test: {e}")
        return False

def main():
    print("\n" + "#"*60)
    print("# Dual-Arm Synchronization - Component Tests")
    print("#"*60)

    tests = [
        ("Environment Creation", test_environment_creation),
        ("Robot Loading", test_robot_loading),
        ("Object Loading", test_object_loading),
        ("Robot Interface", test_robot_interface),
        ("Simulation Stepping", test_simulation_step),
        ("Robot Control", test_robot_control),
    ]

    results = [(name, fn()) for name, fn in tests]
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    passed = sum(1 for _, r in results if r)
    total = len(results)
    for name, r in results:
        print(f"{'✓ PASS' if r else '✗ FAIL'}: {name}")
    print(f"\nTotal: {passed}/{total} tests passed")
    print("\n🎯 Use GUI=True only for manual demos, not for batch tests.")

if __name__ == "__main__":
    main()
