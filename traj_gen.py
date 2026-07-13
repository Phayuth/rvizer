import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


def generate_joint_trajectory():
    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q1 = np.array([3.14, 3.14, 3.14, 3.14, 3.14, 3.14])
    traj = np.linspace(0, 1, 100)
    q_traj = np.outer(1 - traj, q0) + np.outer(traj, q1)
    print(f"==>> q_traj: \n{q_traj}")

    joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]

    traj_dict = {}
    traj_dict["joint_names"] = joint_names
    traj_dict["N"] = q_traj.shape[0]
    traj_dict["points"] = q_traj.tolist()
    traj_dict["time_from_start"] = (np.arange(q_traj.shape[0]) * 0.1).tolist()
    print(f"==>> traj_dict: \n{traj_dict}")

    yaml_file_path = "joint_trajectory.yaml"
    with open(yaml_file_path, "w") as yaml_file:
        yaml.safe_dump(
            traj_dict, yaml_file, default_flow_style=False, sort_keys=False
        )

    with open(yaml_file_path, "r") as yaml_file:
        traj_dict_rec = yaml.safe_load(yaml_file)
        print(f"==>> traj_dict_rec: \n{traj_dict_rec}")


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


def Hlist_to_Xlist(Hlist):
    Xlist = []
    for H in Hlist:
        t = H[:3, 3]
        R_mat = H[:3, :3]
        quat = R.from_matrix(R_mat).as_quat()
        X = np.hstack([t, quat])
        Xlist.append(X)
    return np.array(Xlist)


def generate_taskspace_poses():
    Hlist = pick_task_poses()
    Xlist = Hlist_to_Xlist(Hlist)

    ts_dict = {}
    ts_dict["standard"] = "xyz_qxqyqzqw"
    ts_dict["points"] = Xlist.tolist()
    ts_dict["N"] = Xlist.shape[0]

    yaml_file_path = "taskspace_poses.yaml"
    with open(yaml_file_path, "w") as yaml_file:
        yaml.safe_dump(
            ts_dict, yaml_file, default_flow_style=False, sort_keys=False
        )


def generate_taskspace_tour():
    Hlist = pick_task_poses()
    Xlist = Hlist_to_Xlist(Hlist)
    ts_dict = {}
    Xinit = np.array([0.0, 0.0, 1.0, -0.707106, 0.0, 0.0, 0.707106])

    Xlist = np.vstack([Xinit, Xlist])
    ts_dict["standard"] = "xyz_qxqyqzqw"
    ts_dict["is_points_ordered"] = False
    ts_dict["order"] = list(range(Xlist.shape[0]))
    ts_dict["points"] = Xlist.tolist()
    ts_dict["N"] = Xlist.shape[0]

    yaml_file_path = "taskspace_poses_tour.yaml"
    with open(yaml_file_path, "w") as yaml_file:
        yaml.safe_dump(
            ts_dict, yaml_file, default_flow_style=False, sort_keys=False
        )


if __name__ == "__main__":
    # generate_joint_trajectory()
    # generate_taskspace_poses()
    # generate_taskspace_tour()
    generate_taskspace_tour()