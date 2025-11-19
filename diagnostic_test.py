#!/usr/bin/env python3
"""
DIAGNOSTIC: Test camera projection and segmentation independently.
This will help us identify exactly what's failing.
"""

import numpy as np
import pybullet as p
import pybullet_data
import cv2
from scipy.spatial.transform import Rotation as R
import time
import math

# Simple test setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240.0)

p.resetDebugVisualizerCamera(2.0, 0, -15, [0, 0, 0.9])

# Load plane and table
plane = p.loadURDF("plane.urdf")
col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.35, 0.45, 0.3125])
vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.35, 0.45, 0.3125], rgbaColor=[0.4, 0.4, 0.4, 1])
p.createMultiBody(0, col, vis, [0.6, 0.0, 0.3125])

# Load Panda robot
robot_id = p.loadURDF(
    "franka_panda/panda.urdf",
    [0.6, 0.0, 0.625],
    baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi]),
    useFixedBase=True
)

# Create RED cube at KNOWN position
cube_pos = [0.0, 0.0, 0.95]
h = 0.03
col_cube = p.createCollisionShape(p.GEOM_BOX, halfExtents=[h, h, h])
vis_cube = p.createVisualShape(p.GEOM_BOX, halfExtents=[h, h, h], rgbaColor=[1, 0, 0, 1])
cube_id = p.createMultiBody(0.05, col_cube, vis_cube, cube_pos)

print(f"\n{'='*70}")
print(f"DIAGNOSTIC TEST")
print(f"{'='*70}")
print(f"Cube ID: {cube_id}")
print(f"Cube position (ground truth): {cube_pos}")
print(f"Robot ID: {robot_id}")

# Get robot joints
joints = []
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        joints.append(i)
        if len(joints) == 7:
            break

print(f"Movable joints: {joints}")

# Reset robot to look toward cube
for j, pos in zip(joints, [0, -0.5, 0, -1.4, 0, 1.7, 0]):
    p.resetJointState(robot_id, j, pos)

# Wait for settle
for _ in range(100):
    p.stepSimulation()

# Get EE pose
ee_link = 10  # Panda wrist
ee_state = p.getLinkState(robot_id, ee_link, computeForwardKinematics=True)
ee_pos = np.array(ee_state[4])
ee_quat = np.array(ee_state[5])

print(f"\nEnd effector link: {ee_link}")
print(f"EE position: {ee_pos}")
print(f"EE quaternion: {ee_quat}")

# Camera offset
cam_offset_local = [0.09, 0.0, 0.0]
rot = R.from_quat(ee_quat)
cam_offset_world = rot.apply(cam_offset_local)
cam_pos = ee_pos + cam_offset_world

print(f"\nCamera position: {cam_pos}")
print(f"Camera offset (world): {cam_offset_world}")

# Camera parameters
W, H = 640, 480
fov = 60.0
near, far = 0.01, 10.0

# Compute view matrix
fwd = rot.apply([1, 0, 0])
up = rot.apply([0, 0, 1])
view_target = cam_pos + fwd * 2.0

print(f"Camera forward: {fwd}")
print(f"Camera up: {up}")
print(f"View target: {view_target}")

view_mat = p.computeViewMatrix(cam_pos.tolist(), view_target.tolist(), up.tolist())
proj_mat = p.computeProjectionMatrixFOV(fov, W/H, near, far)

# Capture image
w, h, rgba, depth, seg = p.getCameraImage(
    W, H, view_mat, proj_mat,
    renderer=p.ER_BULLET_HARDWARE_OPENGL,
    flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
)

rgba = np.array(rgba, dtype=np.uint8).reshape(h, w, 4)
depth = np.array(depth, dtype=np.float32).reshape(h, w)
seg = np.array(seg, dtype=np.int32).reshape(h, w)

print(f"\n{'='*70}")
print(f"IMAGE ANALYSIS")
print(f"{'='*70}")

# Check segmentation
mask = (seg & ((1 << 24) - 1)) == cube_id
print(f"Segmentation mask pixels: {np.sum(mask)}")

if np.sum(mask) > 0:
    ys, xs = np.where(mask)
    u_mean = np.mean(xs)
    v_mean = np.mean(ys)
    print(f"Mask centroid (u, v): ({u_mean:.1f}, {v_mean:.1f})")
    
    # Get depth at centroid
    u_int, v_int = int(u_mean), int(v_mean)
    d_buf = depth[v_int, u_int]
    print(f"Depth buffer value: {d_buf:.6f}")
    
    # Convert to linear depth
    z_linear = far * near / (far - (far - near) * d_buf)
    print(f"Linear depth (z_cam): {z_linear:.3f} m")
    
    # Back-project
    f = W / (2.0 * np.tan(np.deg2rad(fov) / 2.0))
    cx = W / 2.0
    cy = H / 2.0
    print(f"Focal length (pixels): {f:.1f}")
    print(f"Principal point: ({cx:.1f}, {cy:.1f})")
    
    x_img = (u_mean - cx) / f
    y_img = (v_mean - cy) / f
    print(f"Normalized image coords: x_img={x_img:.4f}, y_img={y_img:.4f}")
    
    # Try different back-projection methods
    print(f"\n{'='*70}")
    print(f"BACK-PROJECTION TESTS")
    print(f"{'='*70}")
    
    # Method 1: Current implementation
    point_cam_1 = np.array([z_linear, -x_img * z_linear, -y_img * z_linear])
    point_world_1 = cam_pos + rot.apply(point_cam_1)
    error_1 = np.linalg.norm(point_world_1 - cube_pos)
    print(f"Method 1 (current):")
    print(f"  Camera frame: {point_cam_1}")
    print(f"  World frame: {point_world_1}")
    print(f"  Error: {error_1:.3f} m")
    
    # Method 2: Alternative convention
    point_cam_2 = np.array([z_linear, x_img * z_linear, y_img * z_linear])
    point_world_2 = cam_pos + rot.apply(point_cam_2)
    error_2 = np.linalg.norm(point_world_2 - cube_pos)
    print(f"\nMethod 2 (no negation):")
    print(f"  Camera frame: {point_cam_2}")
    print(f"  World frame: {point_world_2}")
    print(f"  Error: {error_2:.3f} m")
    
    # Method 3: OpenCV convention (Z forward, X right, Y down)
    point_cam_3 = np.array([x_img * z_linear, y_img * z_linear, z_linear])
    point_world_3 = cam_pos + rot.apply(point_cam_3)
    error_3 = np.linalg.norm(point_world_3 - cube_pos)
    print(f"\nMethod 3 (OpenCV style):")
    print(f"  Camera frame: {point_cam_3}")
    print(f"  World frame: {point_world_3}")
    print(f"  Error: {error_3:.3f} m")
    
    # Method 4: PyBullet standard (from docs)
    point_cam_4 = np.array([-x_img * z_linear, y_img * z_linear, -z_linear])
    point_world_4 = cam_pos + rot.apply(point_cam_4)
    error_4 = np.linalg.norm(point_world_4 - cube_pos)
    print(f"\nMethod 4 (alternative signs):")
    print(f"  Camera frame: {point_cam_4}")
    print(f"  World frame: {point_world_4}")
    print(f"  Error: {error_4:.3f} m")
    
    print(f"\n{'='*70}")
    print(f"BEST METHOD: Method {np.argmin([error_1, error_2, error_3, error_4]) + 1}")
    print(f"Min error: {min(error_1, error_2, error_3, error_4):.3f} m")
    print(f"{'='*70}")
    
    # Visual comparison
    p.addUserDebugText("TRUE", cube_pos, [0, 1, 0], textSize=2)
    p.addUserDebugLine(cube_pos, cube_pos + [0, 0, 0.1], [0, 1, 0], 3)
    
    best_idx = np.argmin([error_1, error_2, error_3, error_4])
    best_point = [point_world_1, point_world_2, point_world_3, point_world_4][best_idx]
    p.addUserDebugText("EST", best_point, [1, 0, 0], textSize=2)
    p.addUserDebugLine(best_point, best_point + [0, 0, 0.1], [1, 0, 0], 3)
    p.addUserDebugLine(cube_pos, best_point, [1, 1, 0], 2)
    
    # Save debug image
    cv2.imwrite('/tmp/debug_rgb.png', cv2.cvtColor(rgba[:,:,:3], cv2.COLOR_RGB2BGR))
    cv2.imwrite('/tmp/debug_mask.png', (mask * 255).astype(np.uint8))
    print(f"\nDebug images saved to /tmp/debug_rgb.png and /tmp/debug_mask.png")

else:
    print("ERROR: No segmentation mask found for cube!")
    print("Possible issues:")
    print("  - Cube not in camera view")
    print("  - Segmentation not working")
    print("  - Camera pointing wrong direction")

print(f"\nKeeping simulation open for inspection...")
print("Close window to exit")

while True:
    p.stepSimulation()
    time.sleep(0.01)