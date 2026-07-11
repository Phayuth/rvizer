import numpy as np
import viser
import viser.transforms as tf
import yourdfpy
import time
import threading
from robot_descriptions.loaders.yourdfpy import load_robot_description
from bubblify.core import EnhancedViserUrdf

# from .advcore import pass


class AdvApp:

    def __init__(self, port=8080):
        # Gui State
        self.server = viser.ViserServer(port=port)
        self._traj_play_stop = threading.Event()
        self._traj_play_thread = None
        self._traj_slider_programmatic_update = False

        # Load URDF
        urdf_path = (
            "/home/yuth/Resources/ur5e/ur5e_extract_calibrated_spherized.urdf"
        )
        show_collision = True
        if urdf_path is not None:
            self.urdf = yourdfpy.URDF.load(
                str(urdf_path),  # urdf_path,
                load_meshes=True,
                build_scene_graph=True,
                load_collision_meshes=show_collision,
                build_collision_scene_graph=show_collision,
            )
            self.urdf_path = urdf_path
        else:
            robot_name = "panda"
            self.urdf = load_robot_description(
                robot_name + "_description",
                load_meshes=True,
                build_scene_graph=True,
                load_collision_meshes=show_collision,
                build_collision_scene_graph=show_collision,
            )
            self.urdf_path = None

        # Enhanced URDF visualizer with per-link control
        self.urdf_viz = EnhancedViserUrdf(
            self.server,
            urdf_or_path=self.urdf,
            load_meshes=True,
            load_collision_meshes=show_collision,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.4),
            root_node_name="/robot",
        )

        # robot state
        # self.joint_sliders = []
        # self.jointnames_fake = [
        #     "joint1",
        #     "joint2",
        #     "joint3",
        #     "joint4",
        #     "joint5",
        #     "joint6",
        # ]
        # self.joint_initial = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.joint_limits = [
        #     (-3.14, 3.14)
        # ] * 6  # Example joint limits for 6 joints
        self.joint_sliders = []
        self.traj = None

        # Scene state
        self.scene_objects_path = {}
        self.scene_objects = {}
        self.scene_vizs = {}

        # Task space state
        self.taskspace = None

        # Gui Elements
        self._setup_workdir()
        self._setup_refgrid()
        self._setup_import()
        self._setup_scene()
        self._setup_taskspace()

    def _setup_workdir(self):
        with self.server.gui.add_folder("Workdir"):
            self.server.gui.add_text("CWD", initial_value="/home/user/workdir")
            dirb = self.server.gui.add_button("Open Directory")

    def _load_trajectory(self, yaml_file_path):
        import yaml

        with open(yaml_file_path, "r") as yaml_file:
            traj_dict_rec = yaml.safe_load(yaml_file)
            print(f"==>> traj_dict_rec: \n{traj_dict_rec}")

        joint_names = traj_dict_rec["joint_names"]
        points = np.array(traj_dict_rec["points"])
        time_from_start = np.array(traj_dict_rec["time_from_start"])

        return joint_names, points, time_from_start

    def _load_taskspace(self, yaml_file_path):
        import yaml

        with open(yaml_file_path, "r") as yaml_file:
            ts_dict_rec = yaml.safe_load(yaml_file)
            print(f"==>> ts_dict_rec: \n{ts_dict_rec}")

        standard = ts_dict_rec["standard"]
        taskspace_poses = np.array(ts_dict_rec["points"])
        return standard, taskspace_poses

    def _config_from_traj_point(self, point):
        config = np.asarray(point)
        if len(config) < len(self.joint_sliders):
            config = np.hstack(
                [config, np.zeros(len(self.joint_sliders) - len(config))]
            )
        return config

    def _stop_trajectory_playback(self):
        self._traj_play_stop.set()
        if (
            self._traj_play_thread is not None
            and self._traj_play_thread.is_alive()
        ):
            self._traj_play_thread.join(timeout=0.2)
        self._traj_play_thread = None

    def _start_trajectory_playback(
        self, trajslider, trajautoloop, update_robot_config
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
                    update_robot_config()

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

    def _setup_import(self):
        with self.server.gui.add_folder("Import"):
            lb = self.server.gui.add_button("Load a Robot Model")
            infob = self.server.gui.add_button("Show Robot Info")

            # Joint sliders
            initial_config = []

            for joint_name, (
                lower,
                upper,
            ) in self.urdf_viz.get_actuated_joint_limits().items():
                lower = lower if lower is not None else -np.pi
                upper = upper if upper is not None else np.pi
                initial_pos = (
                    0.0 if lower < -0.1 and upper > 0.1 else (lower + upper) / 2.0
                )

                slider = self.server.gui.add_slider(
                    label=joint_name,
                    min=lower,
                    max=upper,
                    step=1e-3,
                    initial_value=initial_pos,
                )
                self.joint_sliders.append(slider)
                initial_config.append(initial_pos)

            # Connect sliders to URDF update
            def update_robot_config():
                config = np.array([s.value for s in self.joint_sliders])
                self.urdf_viz.update_cfg(config)

            for slider in self.joint_sliders:
                slider.on_update(lambda _: update_robot_config())

            # Apply initial configuration
            update_robot_config()

            # Reset button
            reset_joints_btn = self.server.gui.add_button("Reset to Home")

            @reset_joints_btn.on_click
            def _(_):
                for slider, init_val in zip(self.joint_sliders, initial_config):
                    slider.value = init_val

            trajb = self.server.gui.add_button("Load a Trajectory")
            trajslider = self.server.gui.add_slider(
                "Progress",
                min=0.0,
                max=1.0,
                step=0.01,
                initial_value=0.0,
            )
            trajautoloop = self.server.gui.add_checkbox(
                "Auto Loop", initial_value=False
            )

            button_group = self.server.gui.add_button_group(
                label="Player", options=("Play", "Pause", "Reset")
            )

            def update_robot_config_from_traj():
                if self.traj is None or self._traj_slider_programmatic_update:
                    return

                idx = int(trajslider.value)
                if 0 <= idx < self.traj["N"]:
                    config = self._config_from_traj_point(self.traj["points"][idx])
                    self.urdf_viz.update_cfg(config)

            trajslider.on_update(lambda _: update_robot_config_from_traj())

            def handle_player_action(event: viser.GuiEvent) -> None:
                action = event.target.value

                if action == "Play":
                    self._start_trajectory_playback(
                        trajslider, trajautoloop, update_robot_config_from_traj
                    )
                elif action == "Pause":
                    self._stop_trajectory_playback()
                elif action == "Reset":
                    self._stop_trajectory_playback()
                    self._traj_slider_programmatic_update = True
                    trajslider.value = 0.0
                    self._traj_slider_programmatic_update = False
                    update_robot_config_from_traj()

            button_group.on_click(handle_player_action)

            # interaction handle
            @trajb.on_click
            def _(event: viser.GuiEvent) -> None:
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
                print(f"Loaded trajectory from {yaml_file_path}")

                client.add_notification(
                    title="Trajectory Loaded",
                    body="Trajectory has been successfully loaded from the YAML file.",
                    auto_close_seconds=5,
                    with_close_button=True,
                )

                trajslider.min = 0.0
                trajslider.max = max(self.traj["N"] - 1, 0)
                trajslider.step = 1.0
                self._traj_slider_programmatic_update = True
                trajslider.value = 0.0
                self._traj_slider_programmatic_update = False

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
            self.server,
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
            self.server,
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
            self.server,
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
        with self.server.gui.add_folder("Task Space"):

            # parent_frame = self.server.scene.add_frame(
            #     "/parent",
            #     position=(0.0, 0.0, 1.0),
            #     wxyz=tf.SO3.exp(np.array([0.0, 0.0, 0.0])).wxyz,
            # )
            # self.taskspace.append(parent_frame)

            loadtaskb = self.server.gui.add_button("Load Task Space Frame")
            addtaskb = self.server.gui.add_button("Add Task Space Frame")
            deltaskb = self.server.gui.add_button("Delete Task Space Frame")

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
                    task_frame = self.server.scene.add_frame(
                        frame_name,
                        position=position,
                        wxyz=quat,
                        axes_length=0.1,
                        axes_radius=0.005,
                    )
                    # self.taskspace[f"frame_{i}"] = task_frame

    def _setup_refgrid(self):
        self.server.scene.world_axes.visible = True

        self.server.scene.add_grid(
            "/reference_grid",
            width=10,
            height=10,
            position=(0.0, 0.0, 0.0),
            cell_color=(200, 200, 200),
            cell_thickness=1.0,
        )

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
    app = AdvApp(port=8080)
    app.run()
