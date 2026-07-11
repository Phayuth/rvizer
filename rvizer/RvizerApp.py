import numpy as np
import viser
import viser.transforms as tf
import yourdfpy
import time
import threading
import yaml
from robot_descriptions.loaders.yourdfpy import load_robot_description
from bubblify.core import EnhancedViserUrdf
from rvizer.osutils import os_select_folder, os_list_directory
from pathlib import Path


class RvizerApp:

    def __init__(self, port=8080):
        # Gui State
        self.srv = viser.ViserServer(port=port)

        # Load URDF
        self.urdf_paths = {}
        self.urdf_instances = {}
        self.urdf_vizs = {}

        # Robot state
        self.traj = {}
        self._traj_play_stop = threading.Event()
        self._traj_play_thread = None
        self._traj_slider_programmatic_update = False

        # Scene state
        self.scene_objects_path = {}
        self.scene_objects = {}
        self.scene_vizs = {}

        # Task space state
        self.taskspace = None

        # Gui Elements
        self._setup_cwd()
        self._setup_rvizer_config()
        self._setup_refgrid()
        self._setup_robot()
        self._setup_scene()
        self._setup_taskspace()
        self._setup_tf()

    def _setup_cwd(self):
        # gui handle
        with self.srv.gui.add_folder("Current Working Directory"):
            tin = self.srv.gui.add_text("CWD", initial_value="/home/")
            dirb = self.srv.gui.add_button("Open Directory")

            with self.srv.gui.add_folder("Files in CWD"):
                _cwd_files_text = self.srv.gui.add_markdown("_empty_")

        # interaction handle
        @dirb.on_click
        def _(event: viser.GuiEvent) -> None:
            f = os_select_folder(initial_dir="/home/", title="Select Directory")
            tin.value = f
            _cwd_files_text.content = os_list_directory(Path(f))

    def _setup_rvizer_config(self):
        with self.srv.gui.add_folder("RVizer Config"):
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
        urdf_dict = {
            "ur5e": "/home/yuth/Resources/ur5e/ur5e_extract_calibrated_spherized.urdf",
            "ur5e_ghost": "/home/yuth/Resources/ur5e/ur5e_extract_calibrated_spherized.urdf",
        }
        show_collision = True
        for r_name, urdf_path in urdf_dict.items():
            root_node_name = f"/robot/{r_name}"
            urdf = yourdfpy.URDF.load(
                str(urdf_path),  # urdf_path,
                load_meshes=True,
                build_scene_graph=True,
                load_collision_meshes=show_collision,
                build_collision_scene_graph=show_collision,
            )
            urdf_viz = EnhancedViserUrdf(
                self.srv,
                urdf_or_path=urdf,
                load_meshes=True,
                load_collision_meshes=show_collision,
                collision_mesh_color_override=(1.0, 0.0, 0.0, 0.4),
                root_node_name=root_node_name,
            )
            self.urdf_paths[r_name] = urdf_path
            self.urdf_instances[r_name] = urdf
            self.urdf_vizs[r_name] = urdf_viz

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

    def _setup_scene(self):
        """Set up the 3D scene with robot and other elements."""

        # xyz qxqyqzqw
        shelf_tf = {
            0: ((0, 0.75, 0), (0, 0, 1, 0), "id_0_"),
            1: ((0, -0.75, 0), (0, 0, 0, 1), "id_1_"),
            2: ((0.75, 0, 0), (0, 0, 0.5, 0.5), "id_2_"),
        }

        urdf_path = "/home/yuth/Resources/ur5e/shelf.urdf"
        show_collision = True
        urdf1 = yourdfpy.URDF.load(
            str(urdf_path),  # urdf_path,
            load_meshes=True,
            build_scene_graph=True,
            load_collision_meshes=show_collision,
            build_collision_scene_graph=show_collision,
        )
        shelf_position = (0.0, 0.75, 0.0)
        shelf_wxyz = (0.0, 0.0, 0.0, 1.0)  # qwqxqyqz
        shelf_urdf_viz1 = EnhancedViserUrdf(
            self.srv,
            urdf_or_path=urdf1,
            load_meshes=True,
            load_collision_meshes=show_collision,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.4),
            root_node_name="/scene/shelf",
            root_position=shelf_position,
            root_wxyz=shelf_wxyz,
        )

        config = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        shelf_urdf_viz1.update_cfg(config)

        urdf2 = yourdfpy.URDF.load(
            str(urdf_path),  # urdf_path,
            load_meshes=True,
            build_scene_graph=True,
            load_collision_meshes=show_collision,
            build_collision_scene_graph=show_collision,
        )
        shelf_position2 = (0.0, -0.75, 0.0)
        shelf_wxyz2 = (1.0, 0.0, 0.0, 0.0)
        shelf_urdf_viz2 = EnhancedViserUrdf(
            self.srv,
            urdf_or_path=urdf2,
            load_meshes=True,
            load_collision_meshes=show_collision,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.4),
            root_node_name="/scene/shelf2",
            root_position=shelf_position2,
            root_wxyz=shelf_wxyz2,
        )
        shelf_urdf_viz2.update_cfg(config)

        urdf3 = yourdfpy.URDF.load(
            str(urdf_path),  # urdf_path,
            load_meshes=True,
            build_scene_graph=True,
            load_collision_meshes=show_collision,
            build_collision_scene_graph=show_collision,
        )
        shelf_position3 = (0.75, 0.0, 0.0)
        shelf_wxyz3 = tf.SO3.exp(np.array([0.0, 0.0, np.pi / 2.0])).wxyz
        shelf_urdf_viz3 = EnhancedViserUrdf(
            self.srv,
            urdf_or_path=urdf3,
            load_meshes=True,
            load_collision_meshes=show_collision,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.4),
            root_node_name="/scene/shelf3",
            root_position=shelf_position3,
            root_wxyz=shelf_wxyz3,
        )
        shelf_urdf_viz3.update_cfg(config)

    def _setup_taskspace(self):
        with self.srv.gui.add_folder("Task Space"):

            # parent_frame = self.server.scene.add_frame(
            #     "/parent",
            #     position=(0.0, 0.0, 1.0),
            #     wxyz=tf.SO3.exp(np.array([0.0, 0.0, 0.0])).wxyz,
            # )
            # self.taskspace.append(parent_frame)

            loadtaskb = self.srv.gui.add_button("Load Task Space Frame")
            addtaskb = self.srv.gui.add_button("Add Task Space Frame")
            deltaskb = self.srv.gui.add_button("Delete Task Space Frame")

            @loadtaskb.on_click
            def _(event: viser.GuiEvent) -> None:
                client = event.client
                assert client is not None

                yaml_file_path = "taskspace_poses.yaml"  # Example path
                standard, taskspace_poses = self._load_taskspace(yaml_file_path)
                self.taskspace = {
                    "standard": standard,
                    "points": taskspace_poses,
                    "N": taskspace_poses.shape[0],
                    "dof": taskspace_poses.shape[1],
                }
                print(f"Loaded task space poses from {yaml_file_path}")

                client.add_notification(
                    title="Task Space Loaded",
                    body="Task space poses have been successfully loaded from the YAML file.",
                    auto_close_seconds=5,
                    with_close_button=True,
                )

                # self.server.scene.add_a

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
                    frame_name = f"/task/task_frame_{i}"
                    task_frame = self.srv.scene.add_frame(
                        frame_name,
                        position=position,
                        wxyz=quat,
                        axes_length=0.1,
                        axes_radius=0.005,
                    )
                    # self.taskspace[f"frame_{i}"] = task_frame

    def _setup_tf(self):
        with self.srv.gui.add_folder("TF Frames"):
            loadtfb = self.srv.gui.add_button("Load TF Frames")
            addtfb = self.srv.gui.add_button("Add TF Frame")
            deltfb = self.srv.gui.add_button("Delete TF Frame")

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
            pass


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
