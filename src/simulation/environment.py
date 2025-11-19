import os
import math
from typing import Tuple
import numpy as np
import pybullet as p
import pybullet_data as pd

ASSETS = os.path.join(os.path.dirname(__file__), "..", "..", "assets")

class SimulationEnvironment:
    def __init__(self, gui: bool = True, timestep: float = 1.0/240.0, client_id: int | None = None):
        self.client = client_id if client_id is not None else p.connect(p.GUI if gui else p.DIRECT)
        p.resetSimulation(physicsClientId=self.client)
        p.setTimeStep(timestep, physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setAdditionalSearchPath(pd.getDataPath())

        # Workspace: plane + two tables facing each other
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client)
        self.table_a_id = p.loadURDF("table/table.urdf",
                                     basePosition=[-0.8, 0.0, 0.0],
                                     baseOrientation=p.getQuaternionFromEuler([0,0,math.pi/2]),
                                     useFixedBase=True, physicsClientId=self.client)
        self.table_b_id = p.loadURDF("table/table.urdf",
                                     basePosition=[ 0.8, 0.0, 0.0],
                                     baseOrientation=p.getQuaternionFromEuler([0,0,-math.pi/2]),
                                     useFixedBase=True, physicsClientId=self.client)
        self.ids = {}

    def load_robot_a(self, kind: str, base_xyz, base_rpy):
        if kind.lower() == "ur5":
            urdf_path = os.path.join(ASSETS, "ur5", "ur5.urdf")
            if not os.path.isfile(urdf_path):
                print("[WARN] UR5 URDF not found. Falling back to xarm6.")
                urdf_path = "xarm/xarm6_robot.urdf"
        else:
            urdf_path = "xarm/xarm6_robot.urdf"

        rid = p.loadURDF(urdf_path,
                         basePosition=base_xyz,
                         baseOrientation=p.getQuaternionFromEuler(base_rpy),
                         useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION,
                         physicsClientId=self.client)
        self.ids["robot_a"] = rid
        return rid

    def load_robot_b_panda(self, base_xyz, base_rpy):
        rid = p.loadURDF("franka_panda/panda.urdf",
                         basePosition=base_xyz,
                         baseOrientation=p.getQuaternionFromEuler(base_rpy),
                         useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION,
                         physicsClientId=self.client)
        self.ids["robot_b"] = rid
        return rid

    def load_object_cube(self, xyz=( -0.6, 0.0, 0.76 ), rgba=(0.1,0.8,0.1,1.0), size=0.03):
        vs = p.createVisualShape(p.GEOM_BOX, halfExtents=[size]*3, rgbaColor=rgba, physicsClientId=self.client)
        cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size]*3, physicsClientId=self.client)
        oid = p.createMultiBody(baseMass=0.2,
                                baseCollisionShapeIndex=cs,
                                baseVisualShapeIndex=vs,
                                basePosition=xyz,
                                baseOrientation=p.getQuaternionFromEuler([0,0,0]),
                                physicsClientId=self.client)
        p.changeDynamics(oid, -1, lateralFriction=0.8, physicsClientId=self.client)
        self.ids["object"] = oid
        return oid

    def step(self, n=1):
        for _ in range(n):
            p.stepSimulation(physicsClientId=self.client)

    def get_link_pose(self, body_id: int, link_idx: int) -> Tuple[np.ndarray, np.ndarray]:
        if link_idx < 0:
            pos, orn = p.getBasePositionAndOrientation(body_id, physicsClientId=self.client)
        else:
            st = p.getLinkState(body_id, link_idx, computeForwardKinematics=True, physicsClientId=self.client)
            pos, orn = st[4], st[5]
        return np.array(pos), np.array(orn)
