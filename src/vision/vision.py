import numpy as np
import pybullet as p

class Ema:
    def __init__(self, beta=0.7):
        self.beta = beta; self.val = None
    def __call__(self, x):
        if x is None: return self.val
        self.val = x if self.val is None else self.beta*self.val + (1-self.beta)*x
        return self.val

def get_camera_matrices(width, height, fov_deg, near, far):
    aspect = width/float(height)
    proj = p.computeProjectionMatrixFOV(fov_deg, aspect, near, far)
    fy = 0.5*height/np.tan(0.5*np.deg2rad(fov_deg)); fx = fy
    cx, cy = width/2.0, height/2.0
    return proj, fx, fy, cx, cy

def render_rgbd(cid, view, proj, width, height):
    img = p.getCameraImage(width, height, viewMatrix=view, projectionMatrix=proj,
                           physicsClientId=cid, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgb = np.reshape(np.array(img[2], dtype=np.uint8), (height, width, 4))[:,:,:3]
    depth = np.array(img[3]).reshape(height, width)
    return rgb, depth

def make_view_matrix(cam_pos, cam_target, cam_up=(0,0,1)):
    return p.computeViewMatrix(cam_pos, cam_target, cam_up)

def depth_to_linear(depth_buf, near, far):
    z = 2.0*depth_buf - 1.0
    return 2.0*near*far / (far + near - z*(far - near))

def simple_green_seg(rgb):
    g = rgb[:,:,1].astype(np.int16); r = rgb[:,:,0].astype(np.int16); b = rgb[:,:,2].astype(np.int16)
    mask = (g > 120) & (g > r+30) & (g > b+30)
    return mask.astype(np.uint8)

def centroid_from_mask(mask, min_pixels=120):
    ys, xs = np.nonzero(mask)
    if xs.size < min_pixels: return None
    return np.array([xs.mean(), ys.mean()])

def deproject(u, v, depth_lin, fx, fy, cx, cy):
    z = depth_lin
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z])
