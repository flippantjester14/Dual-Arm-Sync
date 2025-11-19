import time
import math
import numpy as np
import pybullet as p
import pybullet_data

# =========================================================
# CONFIG
# =========================================================
GUI = True
TIME_STEP = 1.0 / 240.0
SIM_DURATION = 90.0          # 1 min 30 s

PANDA_URDF = "franka_panda/panda.urdf"
EE_LINK = 11                 # Panda EE link index

TABLE_H = 0.4
TABLE_SIZE = [0.6, 0.6, 0.4]

ROBOT_A_BASE = [-0.6, 0.0, TABLE_H]
ROBOT_B_BASE = [ 0.6, 0.0, TABLE_H]

VERTICAL_PLANE_X = 0.0

# motion pattern
R_CIRCLE     = 0.20
R_LISSAJOUS  = 0.20
OMEGA        = 0.6
X_BASE       = VERTICAL_PLANE_X
X_AMPL       = 0.05
Z_CENTER     = 0.8
Y_CENTER     = 0.0
LISSAJOUS_T0 = 16.0

# phases
PHASE_PREGRASP_END = 2.0
PHASE_GRASP_END    = 4.0
PHASE_LIFT_END     = 6.0
PHASE_TO_PLANE_END = 10.0
PATTERN_START      = PHASE_TO_PLANE_END

# vision model
VISION_DELAY     = 0.10
VISION_NOISE_POS = 0.005
VISION_NOISE_ANG = 0.02

vision_buffer = []
last_meas_pos = None
last_meas_orn = None

# camera params
CAM_FOV_DEG = 60.0
CAM_WIDTH   = 160
CAM_HEIGHT  = 160
CAM_ASPECT  = 1.0

# camera behind Robot B, wall-mounted
CAM_POS    = np.array([ROBOT_B_BASE[0] + 0.6, ROBOT_B_BASE[1], TABLE_H + 0.7])
CAM_TARGET = np.array([VERTICAL_PLANE_X, 0.0, 0.8])
CAM_UP     = np.array([0.0, 0.0, 1.0])

# =========================================================
# UTILS
# =========================================================
def quat_from_rpy(roll, pitch, yaw):
    return p.getQuaternionFromEuler([roll, pitch, yaw])

def lerp(a, b, alpha):
    return [a[i] + alpha * (b[i] - a[i]) for i in range(len(a))]

def distance(a, b):
    return math.sqrt(sum((a[i] - b[i])**2 for i in range(len(a))))

def set_arm_to_pose(body_id, ee_link, pos, orn, max_force=200.0):
    q = p.calculateInverseKinematics(body_id, ee_link, pos, orn)
    n = p.getNumJoints(body_id)
    for j in range(n):
        if j < len(q):
            p.setJointMotorControl2(
                bodyIndex=body_id,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=q[j],
                force=max_force
            )

# =========================================================
# WORLD + ROBOTS + CAMERA
# =========================================================
if GUI:
    p.connect(p.GUI)
else:
    p.connect(p.DIRECT)

p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(TIME_STEP)

p.loadURDF("plane.urdf")

table_col = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[TABLE_SIZE[0]/2, TABLE_SIZE[1]/2, TABLE_SIZE[2]/2]
)
table_vis = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[TABLE_SIZE[0]/2, TABLE_SIZE[1]/2, TABLE_SIZE[2]/2],
    rgbaColor=[0.9, 0.9, 0.9, 1]
)

p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=table_col,
    baseVisualShapeIndex=table_vis,
    basePosition=[ROBOT_A_BASE[0], ROBOT_A_BASE[1], TABLE_H/2]
)
p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=table_col,
    baseVisualShapeIndex=table_vis,
    basePosition=[ROBOT_B_BASE[0], ROBOT_B_BASE[1], TABLE_H/2]
)

for z in np.linspace(0.1, 1.4, 12):
    p.addUserDebugLine(
        [VERTICAL_PLANE_X, -0.4, z],
        [VERTICAL_PLANE_X,  0.4, z],
        [0, 1, 0]
    )

robotA = p.loadURDF(
    PANDA_URDF,
    basePosition=ROBOT_A_BASE,
    baseOrientation=quat_from_rpy(0, 0, 0),
    useFixedBase=True
)
robotB = p.loadURDF(
    PANDA_URDF,
    basePosition=ROBOT_B_BASE,
    baseOrientation=quat_from_rpy(0, 0, math.pi),
    useFixedBase=True
)

home_panda = [0, -0.3, 0, -2.2, 0, 2.0, 0]
for j in range(p.getNumJoints(robotA)):
    q = home_panda[j] if j < len(home_panda) else 0.0
    p.resetJointState(robotA, j, q)
for j in range(p.getNumJoints(robotB)):
    q = home_panda[j] if j < len(home_panda) else 0.0
    p.resetJointState(robotB, j, q)

obj_start = [
    ROBOT_A_BASE[0] + 0.30,
    ROBOT_A_BASE[1],
    TABLE_H + 0.03
]
object_id = p.loadURDF("cube_small.urdf", obj_start, quat_from_rpy(0, 0, 0), globalScaling=1.2)
p.changeVisualShape(object_id, -1, rgbaColor=[1, 0, 0, 1])

view_matrix = p.computeViewMatrix(CAM_POS.tolist(), CAM_TARGET.tolist(), CAM_UP.tolist())
proj_matrix = p.computeProjectionMatrixFOV(
    fov=CAM_FOV_DEG, aspect=CAM_ASPECT, nearVal=0.1, farVal=5.0
)

cam_box_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
cam_box_vis = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.03, 0.03, 0.03],
    rgbaColor=[0.1, 0.1, 0.1, 1]
)
p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=cam_box_col,
    baseVisualShapeIndex=cam_box_vis,
    basePosition=CAM_POS.tolist(),
    baseOrientation=quat_from_rpy(0, 0, 0)
)
p.addUserDebugLine(CAM_POS.tolist(), CAM_TARGET.tolist(), [0, 0, 1])

p.resetDebugVisualizerCamera(
    cameraDistance=2.6,
    cameraYaw=90,
    cameraPitch=-30,
    cameraTargetPosition=[0.0, 0.0, 0.8]
)

# =========================================================
# CAMERA RAY GEOMETRY
# =========================================================
cam_forward = CAM_TARGET - CAM_POS
cam_forward = cam_forward / np.linalg.norm(cam_forward)
cam_up_vec = CAM_UP / np.linalg.norm(CAM_UP)
cam_right = np.cross(cam_forward, cam_up_vec)
cam_right = cam_right / np.linalg.norm(cam_right)
cam_up_corrected = np.cross(cam_right, cam_forward)
cam_up_corrected = cam_up_corrected / np.linalg.norm(cam_up_corrected)

fov_y = math.radians(CAM_FOV_DEG)
fov_x = math.radians(CAM_FOV_DEG)
tan_fov_x = math.tan(fov_x / 2.0)
tan_fov_y = math.tan(fov_y / 2.0)

def pixel_to_world_on_plane(px, py):
    u = (px / (CAM_WIDTH - 1)) * 2.0 - 1.0
    v = 1.0 - (py / (CAM_HEIGHT - 1)) * 2.0
    dir_cam = cam_forward + u * tan_fov_x * cam_right + v * tan_fov_y * cam_up_corrected
    dir_cam = dir_cam / np.linalg.norm(dir_cam)
    if abs(dir_cam[0]) < 1e-6:
        s = 1.0
    else:
        s = (VERTICAL_PLANE_X - CAM_POS[0]) / dir_cam[0]
    if s < 0:
        s = abs(s)
    return CAM_POS + s * dir_cam

# =========================================================
# PATTERN FOR ROBOT A
# =========================================================
def pattern_pose(t_pattern):
    x = X_BASE + X_AMPL * math.sin(OMEGA * t_pattern)
    if t_pattern < LISSAJOUS_T0:
        y = Y_CENTER + R_CIRCLE * math.cos(OMEGA * t_pattern)
        z = Z_CENTER + R_CIRCLE * math.sin(OMEGA * t_pattern)
    else:
        y = Y_CENTER + R_LISSAJOUS * math.sin(OMEGA * t_pattern)
        z = Z_CENTER + R_LISSAJOUS * math.sin(2 * OMEGA * t_pattern)
    return [x, y, z], quat_from_rpy(math.pi, 0, 0)

pregrasp_pos = [
    obj_start[0] - 0.06,
    obj_start[1],
    obj_start[2] + 0.15
]
grasp_pos = [
    obj_start[0],
    obj_start[1],
    obj_start[2] + 0.02
]
lift_pos = [
    obj_start[0],
    obj_start[1],
    obj_start[2] + 0.30
]
plane_center_pos, plane_center_orn = pattern_pose(0.0)
down_orient = quat_from_rpy(math.pi, 0, 0)
grasp_constraint = None

# =========================================================
# VISION MODEL
# =========================================================
def measure_from_camera():
    global last_meas_pos, last_meas_orn
    img = p.getCameraImage(
        width=CAM_WIDTH,
        height=CAM_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    rgb = img[2]
    rgb_arr = np.reshape(rgb, (CAM_HEIGHT, CAM_WIDTH, 4)).astype(np.uint8)
    r = rgb_arr[:, :, 0]
    g = rgb_arr[:, :, 1]
    b = rgb_arr[:, :, 2]
    mask = (r > 200) & (g < 80) & (b < 80)
    ys, xs = np.where(mask)
    if len(xs) == 0:
        if last_meas_pos is not None:
            return last_meas_pos, last_meas_orn
        pos = np.array([X_BASE, 0.0, Z_CENTER])
        orn = quat_from_rpy(math.pi, 0, 0)
        return pos, orn
    px = float(np.mean(xs))
    py = float(np.mean(ys))
    world_point = pixel_to_world_on_plane(px, py)
    pos = world_point
    orn = quat_from_rpy(math.pi, 0, 0)
    last_meas_pos = pos
    last_meas_orn = orn
    return pos, orn

def update_vision_buffer(t):
    pos, orn = measure_from_camera()
    vision_buffer.append((t, pos, orn))
    while vision_buffer and t - vision_buffer[0][0] > 3.0:
        vision_buffer.pop(0)

def sense_object_pose(t):
    if not vision_buffer:
        pos, orn = measure_from_camera()
    else:
        target_t = t - VISION_DELAY
        sample = vision_buffer[0]
        for s in vision_buffer:
            if s[0] <= target_t:
                sample = s
            else:
                break
        pos, orn = sample[1], sample[2]
    noisy_pos = np.array([
        pos[0] + np.random.normal(0, VISION_NOISE_POS),
        pos[1] + np.random.normal(0, VISION_NOISE_POS),
        pos[2] + np.random.normal(0, VISION_NOISE_POS),
    ])
    r, p_e, y = p.getEulerFromQuaternion(orn)
    r += np.random.normal(0, VISION_NOISE_ANG)
    p_e += np.random.normal(0, VISION_NOISE_ANG)
    y += np.random.normal(0, VISION_NOISE_ANG)
    noisy_orn = p.getQuaternionFromEuler([r, p_e, y])
    return noisy_pos.tolist(), noisy_orn

def desired_pose_for_B(obj_pos, obj_orn):
    offset = np.array([0.18, 0.0, 0.0])
    tgt = np.array(obj_pos) + offset
    return tgt.tolist(), obj_orn

# =========================================================
# MAIN LOOP (90 s)
# =========================================================
t0 = time.time()
tracking_log = []

while True:
    t = time.time() - t0
    if t >= SIM_DURATION:
        break

    update_vision_buffer(t)

    # Robot A
    if t < PHASE_PREGRASP_END:
        pos = pregrasp_pos
        orn = down_orient
    elif t < PHASE_GRASP_END:
        alpha = ((t - PHASE_PREGRASP_END) /
                 (PHASE_GRASP_END - PHASE_PREGRASP_END))
        pos = lerp(pregrasp_pos, grasp_pos, alpha)
        orn = down_orient
    elif t < PHASE_LIFT_END:
        alpha = ((t - PHASE_GRASP_END) /
                 (PHASE_LIFT_END - PHASE_GRASP_END))
        pos = lerp(grasp_pos, lift_pos, alpha)
        orn = down_orient
        if grasp_constraint is None:
            grasp_constraint = p.createConstraint(
                parentBodyUniqueId=robotA,
                parentLinkIndex=EE_LINK,
                childBodyUniqueId=object_id,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0]
            )
    elif t < PHASE_TO_PLANE_END:
        alpha = ((t - PHASE_LIFT_END) /
                 (PHASE_TO_PLANE_END - PHASE_LIFT_END))
        pos = lerp(lift_pos, plane_center_pos, alpha)
        orn = plane_center_orn
    else:
        t_pattern = t - PATTERN_START
        pos, orn = pattern_pose(t_pattern)
    set_arm_to_pose(robotA, EE_LINK, pos, orn)

    # Robot B (vision-only tracking)
    if t >= PATTERN_START:
        obj_pos_est, obj_orn_est = sense_object_pose(t)
        tgt_pos_B, tgt_orn_B = desired_pose_for_B(obj_pos_est, obj_orn_est)
        set_arm_to_pose(robotB, EE_LINK, tgt_pos_B, tgt_orn_B)
        ee_state_B = p.getLinkState(robotB, EE_LINK)
        ee_pos_B = ee_state_B[0]
        err = distance(ee_pos_B, tgt_pos_B)
        tracking_log.append((t, err))

    p.stepSimulation()
    if GUI:
        time.sleep(TIME_STEP)

p.disconnect()

with open("tracking_error.csv", "w") as f:
    f.write("time,error\n")
    for ti, ei in tracking_log:
        f.write(f"{ti:.4f},{ei:.6f}\n")
