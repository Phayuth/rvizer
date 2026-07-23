import numpy as np
import viser.transforms as tf
import scipy.spatial.transform as R
from u import write_taskspace_poses


def airbus_shopfloor_taskspace_points():
    # airbus_shopfloor
    # dont forget to offset z by 0.15 stool
    x = 0.6
    y = np.linspace(-0.35, 0.35, 15)
    z = np.linspace(0.2, 0.9, 20)
    position_array = np.meshgrid(x, y, z)
    position_array = np.stack(position_array, axis=-1).reshape(-1, 3)
    xyzw = np.array([0.5000, -0.5000, 0.5000, -0.5000])
    xyzw = xyzw / np.linalg.norm(xyzw)
    wxyz_array = np.tile(xyzw, (position_array.shape[0], 1))
    return position_array, wxyz_array


def cube_surface_grid(bounds, N):
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    x = np.linspace(xmin, xmax, N)
    y = np.linspace(ymin, ymax, N)
    z = np.linspace(zmin, zmax, N)
    pts = []

    # x = xmin, xmax
    Y, Z = np.meshgrid(y, z, indexing="ij")
    pts.append(np.c_[np.full(Y.size, xmin), Y.ravel(), Z.ravel()])
    pts.append(np.c_[np.full(Y.size, xmax), Y.ravel(), Z.ravel()])

    # y = ymin, ymax
    X, Z = np.meshgrid(x, z, indexing="ij")
    pts.append(np.c_[X.ravel(), np.full(X.size, ymin), Z.ravel()])
    pts.append(np.c_[X.ravel(), np.full(X.size, ymax), Z.ravel()])

    # z = zmin, zmax # top and bottom faces
    # X, Y = np.meshgrid(x, y, indexing="ij")
    # pts.append(np.c_[X.ravel(), Y.ravel(), np.full(X.size, zmin)])
    # pts.append(np.c_[X.ravel(), Y.ravel(), np.full(X.size, zmax)])

    pts = np.vstack(pts)
    pts = np.unique(pts, axis=0)  # remove duplicate edges/corners

    return pts


def pose_from_surface_point(p):
    x, y, z = p
    eps = 1e-8

    # Determine outward normal (z-axis of EE)
    if np.isclose(x, -0.6):
        z_axis = np.array([-1.0, 0.0, 0.0])
    elif np.isclose(x, 0.6):
        z_axis = np.array([1.0, 0.0, 0.0])
    elif np.isclose(y, -0.6):
        z_axis = np.array([0.0, -1.0, 0.0])
    elif np.isclose(y, 0.6):
        z_axis = np.array([0.0, 1.0, 0.0])
    elif np.isclose(z, 0.15):
        z_axis = np.array([0.0, 0.0, -1.0])
    elif np.isclose(z, 0.75):
        z_axis = np.array([0.0, 0.0, 1.0])
    else:
        raise ValueError("Point is not on cube surface.")

    # Preferred EE y-axis (global down)
    y_ref = np.array([0.0, 0.0, -1.0])

    # Handle singularity
    if abs(np.dot(z_axis, y_ref)) > 0.99:
        y_ref = np.array([1.0, 0.0, 0.0])

    # Right-handed frame
    x_axis = np.cross(y_ref, z_axis)
    x_axis /= np.linalg.norm(x_axis)

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    R = np.column_stack((x_axis, y_axis, z_axis))

    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = p
    return H


def single_stool_taskspace_points():
    # single_stool
    bounds = (-0.6, 0.6, -0.6, 0.6, 0.15, 0.75)  # x  # y  # z
    pts = cube_surface_grid(bounds, N=10)
    Hs = np.stack([pose_from_surface_point(p) for p in pts])
    position_array = []
    wxyz_array = []
    for h in Hs:
        Htf = tf.SE3.from_matrix(h)
        wxyz_xyz = Htf.wxyz_xyz
        position_array.append(wxyz_xyz[4:])
        wxyz_array.append(wxyz_xyz[:4])

    return np.array(position_array), np.array(wxyz_array)


def pick_task_poses():
    def _gen_linear_H(s, e, quat, num_tasks=10):
        t = np.linspace(s, e, num_tasks)
        Hlist = [np.eye(4) for _ in range(num_tasks)]
        for i in range(num_tasks):
            Hlist[i][:3, 3] = t[i]
            Hlist[i][:3, :3] = R.from_quat(quat).as_matrix()
        return Hlist

    def _Hrot_Z(a):
        H = np.eye(4)
        c, s = np.cos(a), np.sin(a)
        H[0:3, 0:3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        return H

    def _RotPI(H):
        Hdh_to_urdf = _Hrot_Z(np.pi)
        return np.linalg.inv(Hdh_to_urdf) @ H

    size = 4
    params = {
        0: ([-0.4, 0.6, 0.5], [0.4, 0.6, 0.5], [-0.707106, 0.0, 0.0, 0.707106]),
        1: ([-0.4, 0.6, 0.2], [0.4, 0.6, 0.2], [-0.707106, 0.0, 0.0, 0.707106]),
        2: ([-0.6, -0.4, 0.5], [-0.6, 0.4, 0.5], [-0.5, -0.5, 0.5, 0.5]),
        3: ([-0.6, -0.4, 0.2], [-0.6, 0.4, 0.2], [-0.5, -0.5, 0.5, 0.5]),
        4: ([0.4, -0.6, 0.5], [-0.4, -0.6, 0.5], [0.0, -0.707106, 0.707106, 0.0]),
        5: ([0.4, -0.6, 0.2], [-0.4, -0.6, 0.2], [0.0, -0.707106, 0.707106, 0.0]),
    }
    HH = []
    for k in params:
        s, e, quat = params[k]
        quat_noise = quat + np.random.normal(0, 0.05, size=4)
        HH += _gen_linear_H(s, e, quat_noise, num_tasks=size)
    Hlist = np.array(HH)
    Hlist = np.array([_RotPI(H) for H in Hlist])
    return Hlist


if __name__ == "__main__":
    position_array, wxyz_array = single_stool_taskspace_points()
    poses = np.hstack([position_array, wxyz_array])
    write_taskspace_poses(
        poses=poses,
        base_link="stool",
        name="single_stool_taskspace_poses",
        description="Taskspace poses for single stool surface",
        standard="xyz_qwqxqyqz",
    )

    position_array, wxyz_array = airbus_shopfloor_taskspace_points()
    poses = np.hstack([position_array, wxyz_array])
    write_taskspace_poses(
        poses=poses,
        base_link="stool",
        name="airbus_shopfloor_taskspace_poses",
        description="Taskspace poses for airbus shopfloor surface",
        standard="xyz_qwqxqyqz",
    )
