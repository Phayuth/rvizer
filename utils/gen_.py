import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from u import yaml_write, yaml_read


def write_taskspace_poses(
    poses,
    base_link,
    name=None,
    description=None,
    standard="xyz_qxqyqzqw",
    path=None,
):
    # format check
    poses = np.asarray(poses)
    if len(poses.shape) == 2:
        if poses.shape[1] != 7:
            raise ValueError(
                f"Expected poses to have shape (N, 7), got {poses.shape}"
            )
        X = poses
    elif len(poses.shape) == 3:
        if poses.shape[1:] != (4, 4):
            raise ValueError(
                f"Expected poses to have shape (N, 4, 4), got {poses.shape}"
            )
        X = Hlist_to_Xlist(poses)
    else:
        raise ValueError(
            f"Expected poses to have shape (N, 7) or (N, 4, 4), got {poses.shape}"
        )

    ts_dict = {}
    ts_dict["metadata"] = {}
    ts_dict["metadata"]["base_link"] = base_link
    ts_dict["metadata"]["name"] = name
    ts_dict["metadata"]["description"] = description
    ts_dict["metadata"]["standard"] = standard
    ts_dict["metadata"]["N"] = X.shape[0]
    ts_dict["points"] = X.tolist()

    if path is not None:
        fyaml = os.path.join(path, f"{ts_dict['metadata']['name']}.yaml")
    else:
        fyaml = f"{ts_dict['metadata']['name']}.yaml"
    yaml_write(ts_dict, fyaml)


def write_joint_trajectory(
    traj,
    time_from_start=None,
    joint_names=None,
    name=None,
    description=None,
    path=None,
):
    if time_from_start is None:
        time_from_start = (np.arange(traj.shape[0]) * 0.1).tolist()
    if joint_names is None:
        joint_names = [f"joint_{i}" for i in range(traj.shape[1])]

    traj = np.asarray(traj)
    traj_dict = {}
    traj_dict["metadata"] = {}
    traj_dict["metadata"]["name"] = name
    traj_dict["metadata"]["description"] = description
    traj_dict["metadata"]["joint_names"] = joint_names
    traj_dict["metadata"]["N"] = len(traj)
    traj_dict["points"] = traj.tolist()
    traj_dict["time_from_start"] = time_from_start.tolist()

    if path is not None:
        fyaml = os.path.join(path, f"{traj_dict['metadata']['name']}.yaml")
    else:
        fyaml = f"{traj_dict['metadata']['name']}.yaml"
    yaml_write(traj_dict, fyaml)


def read_taskspace_poses(path):
    ts_dict = yaml_read(path)
    base_link = ts_dict["metadata"]["base_link"]
    name = ts_dict["metadata"]["name"]
    description = ts_dict["metadata"]["description"]
    standard = ts_dict["metadata"]["standard"]
    N = ts_dict["metadata"]["N"]
    points = np.array(ts_dict["points"])
    return base_link, name, description, standard, N, points


def read_joint_trajectory(path):
    traj_dict = yaml_read(path)
    name = traj_dict["metadata"]["name"]
    description = traj_dict["metadata"]["description"]
    joint_names = traj_dict["metadata"]["joint_names"]
    N = traj_dict["metadata"]["N"]
    points = np.array(traj_dict["points"])
    time_from_start = np.array(traj_dict["time_from_start"])
    return name, description, joint_names, N, points, time_from_start


def Hlist_to_Xlist(Hlist):
    # Xlist = [(x,y,z, qx, qy, qz, qw), ...] # shape (N,7)
    # Hlist = [H1, H2, ...] # shape (N,4,4)
    Xlist = []
    for H in Hlist:
        t = H[:3, 3]
        R_mat = H[:3, :3]
        quat = R.from_matrix(R_mat).as_quat()
        X = np.hstack([t, quat])
        Xlist.append(X)
    return np.array(Xlist)


def Xlist_to_Hlist(Xlist):
    Hlist = []
    for X in Xlist:
        t = X[:3]
        quat = X[3:]
        R_mat = R.from_quat(quat).as_matrix()
        H = np.eye(4)
        H[:3, :3] = R_mat
        H[:3, 3] = t
        Hlist.append(H)
    return np.array(Hlist)
