from typing import Tuple, Optional, List
import numpy as np
import pybullet as p

def _movable_joints(body_id, cid=0):
    idx, names, ql, qu = [], [], [], []
    for j in range(p.getNumJoints(body_id, physicsClientId=cid)):
        ji = p.getJointInfo(body_id, j, physicsClientId=cid)
        if ji[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            idx.append(j); names.append(ji[1].decode())
            ql.append(ji[8]); qu.append(ji[9])
    return idx, names, np.array(ql), np.array(qu)

class RobotInterface:
    def __init__(self, body_id: int, cid: int, nullspace=None,
                 pos_kp: float = 0.7, pos_kd: float = 0.2, max_force: float = 120.0):
        self.body = body_id
        self.cid = cid
        self.joints, self.names, self.q_lower, self.q_upper = _movable_joints(self.body, cid)
        self.dof = len(self.joints)
        self.null = nullspace or {}
        self.kp = pos_kp
        self.kd = pos_kd
        self.max_force = max_force
        # Initialize to POSITION_CONTROL for stability
        p.setJointMotorControlArray(self.body, self.joints, p.POSITION_CONTROL,
                                    targetPositions=[0.0]*self.dof,
                                    positionGains=[self.kp]*self.dof,
                                    velocityGains=[self.kd]*self.dof,
                                    forces=[self.max_force]*self.dof,
                                    physicsClientId=self.cid)

    def joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        js = p.getJointStates(self.body, self.joints, physicsClientId=self.cid)
        q = np.array([s[0] for s in js]); dq = np.array([s[1] for s in js])
        return q, dq

    def link_index_by_name(self, name_substr: str) -> int:
        for j in range(p.getNumJoints(self.body, physicsClientId=self.cid)):
            ji = p.getJointInfo(self.body, j, physicsClientId=self.cid)
            if name_substr in ji[12].decode():
                return j
        return p.getNumJoints(self.body, physicsClientId=self.cid) - 1

    def ik_position(self, link_idx: int, target_pos, target_orn=None) -> np.ndarray:
        has_null = all(k in self.null for k in ["q_lower","q_upper","q_ref"])
        if target_orn is None:
            if has_null:
                q = p.calculateInverseKinematics(self.body, link_idx, target_pos,
                        lowerLimits=self.null["q_lower"], upperLimits=self.null["q_upper"],
                        jointRanges=(np.array(self.null["q_upper"])-np.array(self.null["q_lower"])).tolist(),
                        restPoses=self.null["q_ref"], physicsClientId=self.cid)
            else:
                q = p.calculateInverseKinematics(self.body, link_idx, target_pos, physicsClientId=self.cid)
        else:
            if has_null:
                q = p.calculateInverseKinematics(self.body, link_idx, target_pos, target_orn,
                        lowerLimits=self.null["q_lower"], upperLimits=self.null["q_upper"],
                        jointRanges=(np.array(self.null["q_upper"])-np.array(self.null["q_lower"])).tolist(),
                        restPoses=self.null["q_ref"], physicsClientId=self.cid)
            else:
                q = p.calculateInverseKinematics(self.body, link_idx, target_pos, target_orn, physicsClientId=self.cid)
        return np.array(q[:self.dof])

    def set_joint_positions(self, q_des: np.ndarray, rate_limit: float = 1.2):
        # Clamp to limits and rate-limit for smoothness
        q_cur, _ = self.joint_states()
        q_des = np.clip(q_des, self.q_lower, self.q_upper)
        dq = np.clip(q_des - q_cur, -rate_limit/240.0, rate_limit/240.0)
        q_cmd = q_cur + dq
        p.setJointMotorControlArray(self.body, self.joints, p.POSITION_CONTROL,
                                    targetPositions=q_cmd.tolist(),
                                    positionGains=[self.kp]*self.dof,
                                    velocityGains=[self.kd]*self.dof,
                                    forces=[self.max_force]*self.dof,
                                    physicsClientId=self.cid)
