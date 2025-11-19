import numpy as np
import pybullet as p

def look_at(from_p, to_p):
    # simple camera look vector
    z = (to_p - from_p); z = z/np.linalg.norm(z)
    up = np.array([0,0,1.0])
    x = np.cross(up, z); x = x/np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.c_[x,y,z]
    return R

def pose_increment(current, target, alpha=0.5):
    return current + alpha*(target - current)
