import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as R

def compute_trajectory(df, deadband=10, sample_rate=100):
    """
    Computes world-frame trajectory, velocity, and acceleration magnitude 
    from raw IMU DataFrame.
    
    Parameters:
    - df: DataFrame with columns ['AccX','AccY','AccZ','AngX','AngY','AngZ']
    - deadband: threshold for |a| below which the device is considered at rest
    - sample_rate: sampling frequency in Hz
    
    Returns:
    - pos: (N,3) array of positions [m]
    - vel: (N,3) array of velocities [m/s]
    - a_mag: (N,) array of acceleration magnitudes [m/s²]
    """
    # Extract raw accel and angles
    acc = df[['AccX', 'AccY', 'AccZ']].values
    rpy = df[['AngX', 'AngY', 'AngZ']].values
    n = len(acc)
    

    # Build orientation objects
    rot = R.from_euler('xyz', rpy, degrees=True)    
    # Compute acceleration magnitude
    a_mag = np.linalg.norm(acc, axis=1)    

    # Per-sample gravity removal & world-frame accel
    g_world = np.array([0,0,10.081836018871865])#9.81
    acc_world = np.zeros((n, 3))
    for i in range(n):
        g_body_i     = rot[i].inv().apply(g_world)
        a_dyn_body   = acc[i] - g_body_i
        acc_world[i] = rot[i].apply(a_dyn_body)


    # 1) Take your first N samples as “static”
    N_cal = 10
    acc_static = acc[:N_cal]  # raw [N_cal×3]
    rpy_static = rpy[:N_cal]
    rot_static = R.from_euler('xyz', rpy_static, degrees=True)

    # 2) Estimate g_world magnitude from the static accel magnitudes
    g_mag = np.mean(np.linalg.norm(acc_static, axis=1))
    g_world = np.array([0, 0, g_mag])

    # 3) Compute the *body‐frame* mean accel
    #    mean_acc_body = g_body_static + bias_body
    mean_acc_body = np.mean(acc_static, axis=0)

    # 4) Compute the *body‐frame* gravity direction at sample 0
    g0_body = rot_static[0].inv().apply(g_world)

    # 5) Solve for constant bias in the body frame
    bias_body = mean_acc_body - g0_body

    for i in range(n):
        # instantaneous gravity in body frame
        g_body_i   = rot[i].inv().apply(g_world)
        # subtract both gravity and bias in *body* frame
        a_dyn_body = acc[i] - g_body_i - bias_body
        # rotate into world
        acc_world[i] = rot[i].apply(a_dyn_body)
    # Compute acceleration magnitude
    a_mag = np.linalg.norm(acc_world, axis=1)
    
    # Manual trapezoidal integration with deadband
    dt = 1.0 / sample_rate
    vel = np.zeros_like(acc_world)
    pos = np.zeros_like(acc_world)
    
    for i in range(1, n):
        # Integrate
        vel[i] = vel[i-1] + 0.5 * (acc_world[i] + acc_world[i-1]) * dt
        pos[i] = pos[i-1] + 0.5 * (vel[i] + vel[i-1]) * dt
        
        # Deadband rest clamp
        if a_mag[i] < deadband:
            vel[i] = 0
            pos[i] = pos[i-1]
    
    return pos, vel, a_mag

def fit_plane_pca(points):
    """
    Fit plane via PCA: returns centroid (plane point), normal, and in-plane axes u, v.
    """
    centroid = np.mean(points, axis=0)
    X = points - centroid
    _, _, Vt = np.linalg.svd(X, full_matrices=False)
    u = Vt[0] / np.linalg.norm(Vt[0])
    v = Vt[1] / np.linalg.norm(Vt[1])
    normal = Vt[2] / np.linalg.norm(Vt[2])
    return centroid, normal, u, v

def fit_circle_2d(xy):
    """
    Algebraic 2D circle fit: solves for (cx, cy, r)
    """
    x = xy[:,0]; y = xy[:,1]
    A = np.column_stack([2*x, 2*y, np.ones_like(x)])
    b = x**2 + y**2
    c, d, f = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c, d
    r = np.sqrt(f + cx**2 + cy**2)
    return np.array([cx, cy]), r

def calculate_plane_and_center_from_arc(pos):
    """
    Fit plane to trajectory, then fit a circle in that plane to find true center.
    Returns:
      - center_3d: 3D circle center
      - normal, u, v: plane axes
      - radius: circle radius
    """
    # 1) Fit plane
    centroid, normal, u, v = fit_plane_pca(pos)
    # 2) Project points to plane coords
    X = pos - centroid
    coords2d = np.vstack([X.dot(u), X.dot(v)]).T
    # 3) Fit circle in 2D
    center2d, radius = fit_circle_2d(coords2d)
    # 4) Back to 3D
    center_3d = centroid + center2d[0]*u + center2d[1]*v
    return center_3d, normal, u, v, radius

def plot_trajectory_with_fitted_plane_and_circle(pos, resolution=20, opacity=0.5, title="Trajectory + Fitted Plane"):
    """
    Plots trajectory, best-fit plane, and circle center/radius.
    """
    center, normal, u, v, radius = calculate_plane_and_center_from_arc(pos)
    # Create plane mesh
    r_lim = radius * 1.2
    us = np.linspace(-r_lim, r_lim, resolution)
    vs = np.linspace(-r_lim, r_lim, resolution)
    uu, vv = np.meshgrid(us, vs)
    Xp = center[0] + uu*u[0] + vv*v[0]
    Yp = center[1] + uu*u[1] + vv*v[1]
    Zp = center[2] + uu*u[2] + vv*v[2]

    fig = go.Figure()
    # plane
    fig.add_trace(go.Surface(x=Xp, y=Yp, z=Zp, opacity=opacity, showscale=False, name="Plane"))
    # trajectory
    fig.add_trace(go.Scatter3d(x=pos[:,0], y=pos[:,1], z=pos[:,2],
                                mode='lines+markers', line=dict(color='blue', width=3),
                                marker=dict(size=3), name='Trajectory'))
    # circle center
    fig.add_trace(go.Scatter3d(x=[center[0]], y=[center[1]], z=[center[2]],
                                mode='markers', marker=dict(color='red', size=6),
                                name=f'Circle Center (r={radius:.2f})'))
    fig.update_layout(scene=dict(xaxis_title='X [m]', yaxis_title='Y [m]', zaxis_title='Z [m]', aspectmode='data'),
                      title=title, margin=dict(l=0,r=0,b=0,t=30))
    fig.show()