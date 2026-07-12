from http import client

import numpy as np
import viser
import viser.transforms as tf
import yourdfpy
import time
import threading
import yaml
from robot_descriptions.loaders.yourdfpy import load_robot_description
from bubblify.core import EnhancedViserUrdf
from rvizer.osutils import os_select_folder, os_list_directory, os_open_directory
from rvizer.guiutils import generate_color_interp
from pathlib import Path


class RvizerApp:

    def __init__(self, port=8080):
        # Gui State
        self.srv = viser.ViserServer(port=port)

        # Robots
        self.urdf_paths = {}
        self.urdf_instances = {}
        self.urdf_vizs = {}
        self.urdf_pose_origins = {}

        # Robot state
        self.traj = {}
        self._traj_play_stop = threading.Event()
        self._traj_play_thread = None
        self._traj_slider_programmatic_update = False

        # Environment Object
        self.eo_paths = {}
        self.eo_instances = {}
        self.eo_vizs = {}
        self.eo_pose_origins = {}

        # Task space state
        self.taskspace = None

        # Gui Elements
        self._setup_cwd()
        self._setup_rvizer_config()
        self._setup_refgrid()
        self._setup_robot()
        self._setup_env()
        self._setup_taskspace()
        self._setup_taskspace_graph()
        self._setup_utilities()

    def _setup_cwd(self):
        # gui handle
        with self.srv.gui.add_folder("Current Working Directory"):
            t_cwd = self.srv.gui.add_text("CWD", initial_value="/home/")
            btng_dir = self.srv.gui.add_button_group(
                label="Action", options=("Select", "Open", "View")
            )
            btn_dirs = self.srv.gui.add_button("Select Directory")
            btn_diro = self.srv.gui.add_button("Open Directory")

            with self.srv.gui.add_folder("Files in CWD"):
                _cwd_files_text = self.srv.gui.add_markdown("_empty_")

        # interaction handle
        @btn_dirs.on_click
        def _(event: viser.GuiEvent) -> None:
            f = os_select_folder(initial_dir="/home/", title="Select Directory")
            t_cwd.value = f
            _cwd_files_text.content = os_list_directory(Path(f))

        @btn_diro.on_click
        def _(event: viser.GuiEvent) -> None:
            os_open_directory(Path(t_cwd.value))

    def _setup_rvizer_config(self):
        with self.srv.gui.add_folder("RVizer Config", expand_by_default=False):
            self.srv.gui.add_button_group(
                label="Save/Load", options=("Load", "Save")
            )

    def _setup_refgrid(self):
        self.srv.scene.world_axes.visible = True

        self.srv.scene.add_grid(
            "/reference_grid",
            width=10,
            height=10,
            position=(0.0, 0.0, 0.0),
            cell_color=(200, 200, 200),
            cell_thickness=1.0,
        )

    def _setup_robot(self):
        fyaml = "scene.yaml"
        with open(fyaml, "r") as yaml_file:
            data = yaml.safe_load(yaml_file)

        r_names = [d["name"] for d in data.get("robots", [])]
        for i in range(len(r_names)):
            r_name = r_names[i]
            self.urdf_paths[r_name] = data["robots"][i]["urdf_path"]
            self.urdf_instances[r_name] = yourdfpy.URDF.load(
                str(self.urdf_paths[r_name]),
                load_meshes=True,
                build_scene_graph=True,
                load_collision_meshes=True,
                build_collision_scene_graph=True,
            )
            self.urdf_vizs[r_name] = EnhancedViserUrdf(
                self.srv,
                urdf_or_path=self.urdf_instances[r_name],
                load_meshes=True,
                load_collision_meshes=True,
                collision_mesh_color_override=(1.0, 0.0, 0.0, 0.4),
                root_node_name=f"/robot/{r_name}",
                root_position=np.array(data["robots"][i]["position"]),
                root_wxyz=np.array(data["robots"][i]["wxyz"]),
            )
            self.urdf_pose_origins[r_name] = (
                data["robots"][i]["position"],
                data["robots"][i]["wxyz"],
            )

        with self.srv.gui.add_folder("Robots"):
            # gui handle
            btn_urdf_load = self.srv.gui.add_button("Load a Robot Model")
            tab_group = self.srv.gui.add_tab_group()
            sldr_joints = {}
            btn_infos = {}
            btn_resets = {}
            btn_trajs = {}
            sldr_trajs = {}
            cb_loops = {}
            btng_players = {}

            # interaction handle
            init_configs = {}
            for r_name, urdf_viz in self.urdf_vizs.items():
                with tab_group.add_tab(f"{r_name}"):
                    sldr_joints[r_name] = []
                    init_configs[r_name] = []
                    r = self.urdf_vizs[r_name].get_actuated_joint_limits().items()
                    for j_name, (lower, upper) in r:
                        lower = lower if lower is not None else -np.pi
                        upper = upper if upper is not None else np.pi
                        initial_pos = (
                            0.0
                            if lower < -0.1 and upper > 0.1
                            else (lower + upper) / 2.0
                        )
                        slider = self.srv.gui.add_slider(
                            label=j_name,
                            min=lower,
                            max=upper,
                            step=1e-3,
                            initial_value=initial_pos,
                        )
                        sldr_joints[r_name].append(slider)
                        init_configs[r_name].append(initial_pos)
                    btn_infos[r_name] = self.srv.gui.add_button("Show Info")
                    btn_resets[r_name] = self.srv.gui.add_button("Reset Home")
                    btn_trajs[r_name] = self.srv.gui.add_button("Load Trajectory")
                    sldr_trajs[r_name] = self.srv.gui.add_slider(
                        "Progress", min=0.0, max=1.0, step=0.01, initial_value=0.0
                    )
                    cb_loops[r_name] = self.srv.gui.add_checkbox(
                        "Auto Loop", initial_value=False
                    )
                    btng_players[r_name] = self.srv.gui.add_button_group(
                        label="Player", options=("Play", "Pause", "Reset")
                    )
                    self.srv.gui.add_dropdown(
                        "Visibility", options=("Visual", "Collision", "Both")
                    )

            # Connect sliders to URDF update
            def update_robot_config(r_name):
                config = np.array([s.value for s in sldr_joints[r_name]])
                self.urdf_vizs[r_name].update_cfg(config)

            for r_name, _ in self.urdf_vizs.items():
                for slider in sldr_joints[r_name]:
                    slider.on_update(
                        lambda _, r_name=r_name: update_robot_config(r_name)
                    )

            # apply initial configuration
            for r_name in self.urdf_vizs:
                update_robot_config(r_name)

            # bind info buttons
            for r_name, btn in btn_infos.items():

                @btn.on_click
                def _(_, r_name=r_name):
                    with self.srv.gui.add_modal(f"{r_name} Info") as modal:
                        self.srv.gui.add_markdown(f"Robot Name: {r_name}")
                        btn_close = self.srv.gui.add_button(
                            "Close", icon=viser.Icon.MOUSE
                        )

                        @btn_close.on_click
                        def _(_) -> None:
                            modal.close()

            # bind reset buttons
            for r_name, btn in btn_resets.items():

                @btn.on_click
                def _(_, r_name=r_name):
                    for slider, init_val in zip(
                        sldr_joints[r_name], init_configs[r_name]
                    ):
                        slider.value = init_val

            # bind load trajectory buttons
            for r_name, btn in btn_trajs.items():

                @btn.on_click
                def _(event: viser.GuiEvent, r_name=r_name):
                    client = event.client
                    assert client is not None

                    yaml_file_path = "joint_trajectory.yaml"  # Example path
                    joint_names, points, time_from_start = self._load_trajectory(
                        yaml_file_path
                    )
                    self.traj = {
                        "joint_names": joint_names,
                        "N": points.shape[0],
                        "points": points,
                        "time_from_start": time_from_start,
                        "dof": points.shape[1],
                    }

                    client.add_notification(
                        title=f"Trajectory Loaded for {r_name}",
                        body=f"Trajectory has been successfully loaded from {yaml_file_path} ",
                        auto_close_seconds=5,
                        with_close_button=True,
                    )

                    sldr_trajs[r_name].min = 0.0
                    sldr_trajs[r_name].max = max(self.traj["N"] - 1, 0)
                    sldr_trajs[r_name].step = 1.0
                    self._traj_slider_programmatic_update = True
                    sldr_trajs[r_name].value = 0.0
                    self._traj_slider_programmatic_update = False

            # bind trajectory slider to update robot configuration
            def update_robot_config_traj(r_name):
                if self.traj is None or self._traj_slider_programmatic_update:
                    return

                idx = int(sldr_trajs[r_name].value)
                if 0 <= idx < self.traj["N"]:
                    config = np.asarray(self.traj["points"][idx])
                    self.urdf_vizs[r_name].update_cfg(config)

            for r_name, slider in sldr_trajs.items():
                slider.on_update(
                    lambda _, r_name=r_name: update_robot_config_traj(r_name)
                )

            # bind player buttons
            for r_name, slider in sldr_trajs.items():

                def handle_player_action(event: viser.GuiEvent, r_name=r_name):
                    action = event.target.value

                    if action == "Play":
                        self._start_trajectory_playback(
                            sldr_trajs[r_name],
                            cb_loops[r_name],
                            update_robot_config_traj,
                            r_name,
                        )
                    elif action == "Pause":
                        self._stop_trajectory_playback()
                    elif action == "Reset":
                        self._stop_trajectory_playback()
                        self._traj_slider_programmatic_update = True
                        sldr_trajs[r_name].value = 0.0
                        self._traj_slider_programmatic_update = False
                        update_robot_config_traj(r_name)

                btng_players[r_name].on_click(handle_player_action)

    def _load_trajectory(self, yaml_file_path):
        with open(yaml_file_path, "r") as yaml_file:
            traj_dict_rec = yaml.safe_load(yaml_file)
        joint_names = traj_dict_rec["joint_names"]
        points = np.array(traj_dict_rec["points"])
        time_from_start = np.array(traj_dict_rec["time_from_start"])
        return joint_names, points, time_from_start

    def _load_taskspace(self, yaml_file_path):
        with open(yaml_file_path, "r") as yaml_file:
            ts_dict_rec = yaml.safe_load(yaml_file)
        standard = ts_dict_rec["standard"]
        taskspace_poses = np.array(ts_dict_rec["points"])
        return standard, taskspace_poses

    def _stop_trajectory_playback(self):
        self._traj_play_stop.set()
        if (
            self._traj_play_thread is not None
            and self._traj_play_thread.is_alive()
        ):
            self._traj_play_thread.join(timeout=0.2)
        self._traj_play_thread = None

    def _start_trajectory_playback(
        self, trajslider, trajautoloop, update_robot_config, r_name
    ):
        if self.traj is None:
            return

        self._stop_trajectory_playback()
        self._traj_play_stop.clear()

        def _run():
            time_from_start = np.asarray(self.traj.get("time_from_start", []))
            start_idx = int(trajslider.value)
            start_idx = max(0, min(start_idx, self.traj["N"] - 1))

            while self.traj is not None and not self._traj_play_stop.is_set():
                prev_time = (
                    float(time_from_start[start_idx - 1])
                    if start_idx > 0 and start_idx - 1 < len(time_from_start)
                    else 0.0
                )

                for idx in range(start_idx, self.traj["N"]):
                    if self._traj_play_stop.is_set():
                        return

                    self._traj_slider_programmatic_update = True
                    trajslider.value = float(idx)
                    self._traj_slider_programmatic_update = False
                    update_robot_config(r_name)

                    if idx < len(time_from_start):
                        target_time = float(time_from_start[idx])
                        sleep_time = max(target_time - prev_time, 0.02)
                        prev_time = target_time
                    else:
                        sleep_time = 0.05

                    time.sleep(sleep_time)

                if not trajautoloop.value:
                    break

                start_idx = 0

        self._traj_play_thread = threading.Thread(target=_run, daemon=True)
        self._traj_play_thread.start()

    def _setup_env(self):
        fyaml = "scene.yaml"
        with open(fyaml, "r") as yaml_file:
            data = yaml.safe_load(yaml_file)

        eo_names = [d["name"] for d in data.get("env_objects", [])]
        for i in range(len(eo_names)):
            eo_name = eo_names[i]
            self.eo_paths[eo_name] = data["env_objects"][i]["urdf_path"]
            self.eo_instances[eo_name] = yourdfpy.URDF.load(
                str(self.eo_paths[eo_name]),
                load_meshes=True,
                build_scene_graph=True,
                load_collision_meshes=True,
                build_collision_scene_graph=True,
            )
            self.eo_vizs[eo_name] = EnhancedViserUrdf(
                self.srv,
                urdf_or_path=self.eo_instances[eo_name],
                load_meshes=True,
                load_collision_meshes=True,
                collision_mesh_color_override=(1.0, 0.0, 0.0, 0.4),
                root_node_name=f"/env_objects/{eo_name}",
                root_position=np.array(data["env_objects"][i]["position"]),
                root_wxyz=np.array(data["env_objects"][i]["wxyz"]),
            )
            self.eo_pose_origins[eo_name] = (
                data["env_objects"][i]["position"],
                data["env_objects"][i]["wxyz"],
            )

        # apply initial configuration
        config = np.array([])  # no actuated joints = 0
        for eo_name in self.eo_vizs:
            self.eo_vizs[eo_name].update_cfg(config)

        # gui handle
        with self.srv.gui.add_folder("Environment Objects"):
            dd_int = self.srv.gui.add_dropdown(
                label="Instances",
                options=eo_names,
            )
            dd_v = self.srv.gui.add_dropdown(
                "Visibility", options=("Visual", "Collision", "Both")
            )

    def _setup_taskspace(self):
        with self.srv.gui.add_folder("Task Space"):
            # gui handle
            btng_tslad = self.srv.gui.add_button_group(
                label="Action", options=("Load", "Add", "Delete")
            )
            btng_tstour = self.srv.gui.add_button_group(
                label="Tour", options=("Load", "View", "Hide")
            )
            sldr_tstour = self.srv.gui.add_slider(
                label="Tour Progress",
                min=0.0,
                max=1.0,
                step=0.01,
                initial_value=0.0,
            )

            # interaction handle
            ts_handles = []

            def _handle_btng_tslad(event: viser.GuiEvent) -> None:
                client = event.client
                action = event.target.value

                if action == "Load":
                    yaml_file_path = "taskspace_poses.yaml"  # Example path
                    standard, taskspace_poses = self._load_taskspace(
                        yaml_file_path
                    )
                    self.taskspace = {
                        "standard": standard,
                        "points": taskspace_poses,
                        "N": taskspace_poses.shape[0],
                        "dof": taskspace_poses.shape[1],
                    }

                    client.add_notification(
                        title="Task Space Loaded",
                        body="Task space poses have been successfully loaded from the YAML file.",
                        auto_close_seconds=5,
                        with_close_button=True,
                    )

                    for i in range(self.taskspace["N"]):
                        pose = self.taskspace["points"][i]
                        position = pose[:3]
                        if standard == "xyz_qxqyqzqw":
                            quat = np.array(
                                [
                                    pose[6],
                                    pose[3],
                                    pose[4],
                                    pose[5],
                                ]
                            )  # Convert to wxyz
                        else:
                            quat = np.array([pose[3], pose[4], pose[5], pose[6]])

                        ts_h_ = self.srv.scene.add_frame(
                            f"/task/tasks/frame_{i}",
                            position=position,
                            wxyz=quat,
                            axes_length=0.1,
                            axes_radius=0.005,
                        )
                        ts_handles.append(ts_h_)

            btng_tslad.on_click(_handle_btng_tslad)

            def _handle_btng_tstour(event: viser.GuiEvent) -> None:
                client = event.client
                action = event.target.value

                if action == "Load":
                    fyaml = "taskspace_poses_tour.yaml"
                    with open(fyaml, "r") as yaml_file:
                        data = yaml.safe_load(yaml_file)

                    ts_position = []
                    for i in range(len(ts_handles)):
                        tsh = ts_handles[i]
                        ts_position.append(tsh.position)

                    points = np.array(ts_position).reshape(-1, 3)  # shape (N, 3)
                    points = np.concatenate([points[:-1], points[1:]], axis=1)
                    points = points.reshape(-1, 2, 3)  # shape (N-1, 2, 3)

                    nn = points.shape[0] + 1
                    colors = generate_color_interp(n=nn, m="rgb01")
                    colors = np.array(colors)
                    colors = np.concatenate([colors[:-1], colors[1:]], axis=1)
                    colors = colors.reshape(-1, 2, 3)  # shape (N-1, 2, 3)

                    self.srv.scene.add_line_segments(
                        "/task/tour_order",
                        points=points,
                        colors=colors,
                        line_width=5,
                    )
                    self.srv.scene.add_label(
                        "/task/tour_start",
                        text="Start",
                        position=points[0, 0],
                    )
                    self.srv.scene.add_label(
                        "/task/tour_end",
                        text="End",
                        position=points[-1, 1],
                    )

                    client.add_notification(
                        title="Load Tour",
                        body="Load tour successfully.",
                        auto_close_seconds=5,
                        with_close_button=True,
                    )

                    tour_sphere = self.srv.scene.add_icosphere(
                        "/task/tour_sphere",
                        position=points[0, 0],
                        radius=0.03,
                        color=(0.0, 1.0, 0.0),
                    )

                    def update_tour_sphere(value):
                        idx = int(value)
                        if 0 <= idx < nn:
                            if idx < nn - 1:
                                tour_sphere.position = points[idx, 0]
                            else:
                                tour_sphere.position = points[-1, 1]

                    sldr_tstour.min = 0.0
                    sldr_tstour.max = max(nn - 1, 0)
                    sldr_tstour.step = 1.0
                    sldr_tstour.value = 0.0
                    sldr_tstour.on_update(
                        lambda event: update_tour_sphere(event.target.value)
                    )

            btng_tstour.on_click(_handle_btng_tstour)

    def _setup_taskspace_graph(self):
        pass

    def _setup_utilities(self):
        with self.srv.gui.add_folder("Utilities", expand_by_default=False):
            btn_tfv = self.srv.gui.add_button("TF Tree Viewer")
            btn_tfa = self.srv.gui.add_button("TF Add")
            btn_tfd = self.srv.gui.add_button("TF Delete")

    def run(self):
        """Run the application (blocking call)."""
        print("Starting Viser server...")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping Viser server...")
        finally:
            print("Server stopped.")
            # cleanup code if needed


if __name__ == "__main__":
    app = RvizerApp(port=8080)
    app.run()


# interesting code snippet for resetting joint sliders to initial values
# The loop was failing because the callback inside it was closing over the loop variable r_name.
# In Python, that variable is late-bound, so by the time the button is clicked, every handler can end up seeing the same final value.
# The explicit version worked because it hardcoded each robot name, so each button targeted the right slider list.

# br1 = btn_resets["ur5e"]
# br2 = btn_resets["ur5e_ghost"]

# @br1.on_click
# def _(event: viser.GuiEvent) -> None:
#     for slider, init_val in zip(
#         sliders_joint["ur5e"], init_configs["ur5e"]
#     ):
#         slider.value = init_val

# @br2.on_click
# def _(event: viser.GuiEvent) -> None:
#     for slider, init_val in zip(
#         sliders_joint["ur5e_ghost"], init_configs["ur5e_ghost"]
#     ):
#         slider.value = init_val
