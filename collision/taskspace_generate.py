import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from u import write_taskspace_poses, Hlist_to_xyz_wxyz


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


def single_stool_taskspace_points():

    def _cube_surface_grid(bounds, N):
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

    def _pose_from_surface_point(p):
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

    bounds = (-0.6, 0.6, -0.6, 0.6, 0.15, 0.75)  # x  # y  # z
    pts = _cube_surface_grid(bounds, N=10)
    Hs = np.stack([_pose_from_surface_point(p) for p in pts])
    position_array, wxyz_array = Hlist_to_xyz_wxyz(Hs)
    return position_array, wxyz_array


def three_shelf_taskspace_points():
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
        quat_noise = quat + np.random.normal(0, 0.02, size=4)
        HH += _gen_linear_H(s, e, quat_noise, num_tasks=size)
    Hlist = np.array(HH)
    Hlist = np.array([_RotPI(H) for H in Hlist])
    position_array, wxyz_array = Hlist_to_xyz_wxyz(Hlist)
    return position_array, wxyz_array


def inspect_stool_random_taskspace_points():
    x, y, z, r = 0.5, 0.0, 0.2, 0.22  # center and radius of hemisphere
    n = 500
    center = np.array([x, y, z])

    Hlist = []
    for _ in range(n):
        # Sample point on upper hemisphere
        theta = np.random.uniform(0, 2 * np.pi)
        u = np.random.uniform(0, 1)
        s = np.sqrt(1 - u * u)

        p = np.array(
            [
                x + r * s * np.cos(theta),
                y + r * s * np.sin(theta),
                z + r * u,
            ]
        )

        # z-axis points toward center
        z_axis = center - p
        z_axis /= np.linalg.norm(z_axis)

        # Random x-axis orthogonal to z-axis
        while True:
            v = np.random.randn(3)
            x_axis = v - np.dot(v, z_axis) * z_axis
            norm = np.linalg.norm(x_axis)
            if norm > 1e-8:
                x_axis /= norm
                break

        # Right-handed frame
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)

        H = np.eye(4)
        H[:3, 0] = x_axis
        H[:3, 1] = y_axis
        H[:3, 2] = z_axis
        H[:3, 3] = p

        Hlist.append(H)
    position_array, wxyz_array = Hlist_to_xyz_wxyz(Hlist)
    return position_array, wxyz_array


def inspect_stool_taskspace_points():
    x, y, z, r = 0.5, 0.0, 0.2, 0.22  # center and radius of hemisphere
    center = np.array([x, y, z])

    def _hemisphere_surface_points(n_phi=8, n_theta_max=24):
        points = []
        phis = np.linspace(0, np.pi / 2, n_phi)
        for phi in phis:
            sin_phi = np.sin(phi)
            cos_phi = np.cos(phi)

            # circumference shrinks toward the pole
            n_theta = max(1, int(round(n_theta_max * sin_phi)))

            for theta in np.linspace(0, 2 * np.pi, n_theta, endpoint=False):
                points.append(
                    [
                        x + r * sin_phi * np.cos(theta),
                        y + r * sin_phi * np.sin(theta),
                        z + r * cos_phi,
                    ]
                )

        return np.asarray(points)

    points = _hemisphere_surface_points(n_phi=8, n_theta_max=24)
    n = points.shape[0]

    Hlist = []
    for i in range(n):
        p = points[i]
        # z-axis points toward center
        z_axis = center - p
        z_axis /= np.linalg.norm(z_axis)

        # Random x-axis orthogonal to z-axis
        while True:
            v = np.random.randn(3)
            x_axis = v - np.dot(v, z_axis) * z_axis
            norm = np.linalg.norm(x_axis)
            if norm > 1e-8:
                x_axis /= norm
                break

        # Right-handed frame
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)

        H = np.eye(4)
        H[:3, 0] = x_axis
        H[:3, 1] = y_axis
        H[:3, 2] = z_axis
        H[:3, 3] = p

        Hlist.append(H)
    position_array, wxyz_array = Hlist_to_xyz_wxyz(Hlist)
    return position_array, wxyz_array


def two_sided_taskspace_points():
    y = np.linspace(-0.5, 0.5, 5)
    z = np.linspace(-0.5, 0.5, 5)
    Y, Z = np.meshgrid(y, z)
    YZ = np.vstack([Y.ravel(), Z.ravel()]).T
    x = np.full(YZ.shape[0], 0.42)
    points1 = np.hstack([x[:, None], YZ])

    points2 = points1.copy()
    points2[:, 0] = -0.42

    points = np.vstack([points1, points2])
    n = points.shape[0]

    Hlist = np.empty((n, 4, 4))
    for i, p in enumerate(points):
        H = np.eye(4)
        H[0:3, 3] = p
        Hlist[i] = H
    position_array, wxyz_array = Hlist_to_xyz_wxyz(Hlist)
    return position_array, wxyz_array


def four_sided_noise_taskspace_points():
    # add a bit of noise on rotation now cluster is not perfect, but we can still find the cluster
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

    def _RotPI2(H):
        Hdh_to_urdf = _Hrot_Z(np.pi / 2)
        return np.linalg.inv(Hdh_to_urdf) @ H

    def _RotPI3(H):
        Hdh_to_urdf = _Hrot_Z(3 * np.pi / 2)
        return np.linalg.inv(Hdh_to_urdf) @ H

    size = 4
    params = {
        0: ([0.5, 0.5, 0.6], [0.5, -0.5, 0.6], [0.0, 0.707106, 0.0, 0.707106]),
        1: ([0.5, 0.5, 0.4], [0.5, -0.5, 0.4], [0.0, 0.707106, 0.0, 0.707106]),
        2: ([0.5, 0.5, 0.2], [0.5, -0.5, 0.2], [0.0, 0.707106, 0.0, 0.707106]),
        3: ([0.5, 0.5, 0.0], [0.5, -0.5, 0.0], [0.0, 0.707106, 0.0, 0.707106]),
    }
    HH = []
    for k in params:
        s, e, quat = params[k]
        quat_noise = quat + np.random.normal(0, 0.05, size=4)
        HH += _gen_linear_H(s, e, quat_noise, num_tasks=size)
    GG = []
    for h in HH:
        GG.append(_RotPI(h))
        GG.append(_RotPI2(h))
        GG.append(_RotPI3(h))
    Hlist = np.array(HH + GG)
    position_array, wxyz_array = Hlist_to_xyz_wxyz(Hlist)
    return position_array, wxyz_array


def epGH_taskspace_points():
    """Generates discrete set of poses to form the task space.
    Generates discrete set of poses, manually defined here as uniform grid facing into the world -z direction with 45 deg offsets.
    """

    def transform_lookat(at, eye, up):
        """Copied from OpenRAVE's transformLookat function in "geometry.h".

        Returns an end effector transform matrix that looks along a ray with a desired up vector (corresponding to y axis of the end effector).
        If up vector is parallel to ray, tries to use +y or +x direction instead.
        If ray length is zero, chooses ray to be +z direction by default.

        @param at the point space to look at, the camera will rotation and zoom around this point
        @param eye the position of the camera in space
        @param up desired end effector y axis direction
        @return end effector transform matrix
        """
        vdir = np.array(at) - eye
        if np.linalg.norm(vdir) > 1e-6:
            vdir *= 1 / np.linalg.norm(vdir)
        else:
            vdir = [0.0, 0.0, 1.0]

        vup = np.array(up) - vdir * np.dot(up, vdir)
        if np.linalg.norm(vup) < 1e-8:
            vup = [0.0, 1.0, 0.0]
            vup -= vdir * np.dot(vdir, vup)
            if np.linalg.norm(vup) < 1e-8:
                vup = [1.0, 0.0, 0.0]
                vup -= vdir * np.dot(vdir, vup)

        vup *= 1 / np.linalg.norm(vup)
        right = np.cross(vup, vdir)

        rot_mat = np.transpose([right, vup, vdir])
        H = [
            list(rot_mat[0]) + [eye[0]],
            list(rot_mat[1]) + [eye[1]],
            list(rot_mat[2]) + [eye[2]],
            [0, 0, 0, 1],
        ]
        return np.array(H)

    Hlist = []
    ats = [
        [0.0, 0.0, -1.0],
        [0.0, -1.0, -1.0],
        [-1.0, 0.0, -1.0],
        [0.0, 1.0, -1.0],
        [1.0, 0.0, -1.0],
    ]
    up_vector = [-1.0, 0.0, 0.0]

    step = 0.1
    pos_x_list = np.arange(0.25, 0.85 + step, step)
    pos_y_list = np.arange(-0.45, 0.45 + step, step)
    pos_z_list = np.arange(0.15, 0.45 + step, step)

    for pos_x in pos_x_list:
        for pos_y in pos_y_list:
            for pos_z in pos_z_list:
                for at_offset in ats:
                    eye = [pos_x, pos_y, pos_z]
                    # 0.001 is because IKFast solution is singular for poses pointing directly in z axis
                    at = [pos_x + 0.001, pos_y - 0.001, pos_z]
                    at = [x + y for x, y in zip(at, at_offset)]
                    H = transform_lookat(at, eye, up_vector)
                    Hlist.append(H)
    Hlist = np.array(Hlist)
    position_array, wxyz_array = Hlist_to_xyz_wxyz(Hlist)
    return position_array, wxyz_array


def stool_shelf_taskspace_points():
    # stool_shelf
    x = 0.6
    y = np.linspace(-0.40, 0.40, 7)
    z = np.linspace(0.15, 0.9, 3)
    position_array = np.meshgrid(x, y, z)
    position_array = np.stack(position_array, axis=-1).reshape(-1, 3)
    xyzw = np.array([0.5000, -0.5000, 0.5000, -0.5000])
    xyzw = xyzw / np.linalg.norm(xyzw)
    wxyz_array = np.tile(xyzw, (position_array.shape[0], 1))
    return position_array, wxyz_array


if __name__ == "__main__":
    dir_rsrc = os.environ["RSRC_DIR"]
    dir_rtsp = os.path.join(dir_rsrc, "rtsp_env")

    position_array, wxyz_array = single_stool_taskspace_points()
    poses = np.hstack([position_array, wxyz_array])
    write_taskspace_poses(
        poses=poses,
        base_link="stool",
        name="single_stool_taskspace_poses",
        description="Taskspace poses for single stool",
        standard="xyz_qwqxqyqz",
        path=dir_rtsp,
    )

    position_array, wxyz_array = airbus_shopfloor_taskspace_points()
    poses = np.hstack([position_array, wxyz_array])
    write_taskspace_poses(
        poses=poses,
        base_link="stool",
        name="airbus_shopfloor_taskspace_poses",
        description="Taskspace poses for airbus shopfloor",
        standard="xyz_qwqxqyqz",
        path=dir_rtsp,
    )

    position_array, wxyz_array = three_shelf_taskspace_points()
    poses = np.hstack([position_array, wxyz_array])
    write_taskspace_poses(
        poses=poses,
        base_link="world_link",
        name="three_shelf_taskspace_poses",
        description="Taskspace poses for three shelf surronding robot",
        standard="xyz_qwqxqyqz",
        path=dir_rtsp,
    )
