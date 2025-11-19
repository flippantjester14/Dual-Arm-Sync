import numpy as np

def circle_lissajous_with_z(t, center, R=0.12, liss_ratio=(2,1), z_amp=0.04, z_freq=0.35):
    x0, y0, z0 = center
    # Alternate every 5 s
    phase_block = int(t // 5) % 2
    th = 0.8*t
    if phase_block == 0:
        y = y0 + R*np.cos(th)
        z = z0 + R*np.sin(th)
    else:
        a, b = liss_ratio
        y = y0 + R*np.sin(a*th)
        z = z0 + R*np.sin(b*th + np.pi/2)
    x = x0 + 0.04*np.sin(2*np.pi*z_freq*t)
    return np.array([x, y, z])
