import os
import numpy as np
import viser
import viser.transforms as tf
import yourdfpy
import time
import threading
import yaml
from pathlib import Path
from bubblify.core import EnhancedViserUrdf
from rvizer.osutils import os_select_folder, os_list_directory, os_open_directory
from rvizer.guiutils import (
    gen_color_interp,
    load_trajectory,
    load_taskspace,
    load_taskspace_tour,
    load_robot_config,
)


class RvizerApp:

    def __init__(self, port=8080):
        # Gui State
        self.srv = viser.ViserServer(port=port)

        # cwd
        self.cwd = "/home/"

        # Scenesheet
        self.ss = None

        # Robots
        self.r_paths = {}
        self.r_instances = {}
        self.r_vizs = {}
        self.r_pose_origins = {}
        self.r_configs = {}

        # Robot state
        self.traj = {}
        self._traj_btn_play_stop = threading.Event()
        self._traj_btn_play_thread = None
        self._traj_sldr_prog_update = False

        # Environment Object
        self.eo_paths = {}
        self.eo_instances = {}
        self.eo_vizs = {}
        self.eo_pose_origins = {}

        # Task space state
        self.taskspace = None
        self.taskspace_tour = None

        # Gui Elements
        self._setup_cwd()
        self._setup_rvizer_config()
        # self._setup_robot()
        # self._setup_env()
        # self._setup_taskspace()
        # self._setup_rtsp()
        # self._setup_utilities()

    def _setup_cwd(self):
        with self.srv.gui.add_folder("Current Working Directory"):
            # gui handle
            t_cwd = self.srv.gui.add_text("CWD", initial_value=self.cwd)
            btng_dir = self.srv.gui.add_button_group(
                label="Action", options=("Select", "Open", "View", "Init")
            )
            cwd_files = ""

            def handle_btng_dir(event: viser.GuiEvent):
                nonlocal cwd_files
                action = event.target.value

                if action == "Select":
                    f = os_select_folder(
                        initial_dir=self.cwd, title="Select Directory"
                    )
                    t_cwd.value = f
                    self.cwd = f
                    cwd_files = os_list_directory(Path(f))

                elif action == "Open":
                    os_open_directory(Path(t_cwd.value))

                elif action == "View":
                    with self.srv.gui.add_modal("Directory Contents") as modal:
                        self.srv.gui.add_markdown(f"Directory: {t_cwd.value}")
                        self.srv.gui.add_markdown(f"Contents:\n{cwd_files}")
                        btn_close = self.srv.gui.add_button(
                            "Close", icon=viser.Icon.MOUSE
                        )

                        @btn_close.on_click
                        def _(_) -> None:
                            modal.close()

                elif action == "Init":
                    success = self._read_scene(self.cwd)
                    if not success:
                        return
                    self._setup_robot()
                    self._setup_env()
                    self._setup_taskspace()
                    self._setup_rtsp()
                    self._setup_utilities()

            btng_dir.on_click(handle_btng_dir)

    def _read_scene(self, fyaml):
        fyaml = os.path.join(fyaml, "scene.yaml")
        try:
            with open(fyaml, "r") as yaml_file:
                data = yaml.safe_load(yaml_file)
                self.ss = data
        except FileNotFoundError:
            print(f"Scene file not found: {fyaml}")
            self.ss = None
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file: {fyaml}\n{e}")
            self.ss = None
        else:
            print(f"Scene file loaded successfully: {fyaml}")
            return True
        return False

    def _setup_rvizer_config(self):
        self.srv.scene.world_axes.visible = True
        self.srv.scene.world_axes.axes_length = 5
        self.srv.scene.world_axes.axes_radius = 0.001
        self.srv.gui.configure_theme(
            control_width="large",
            control_layout="floating",
        )
        self.srv.scene.add_grid(
            "/reference_grid",
            width=10,
            height=10,
            position=(0.0, 0.0, 0.0),
            cell_color=(200, 200, 200),
            cell_thickness=1.0,
        )

        with self.srv.gui.add_folder("RVizer Config", expand_by_default=False):
            self.srv.gui.add_button_group(
                label="Save/Load", options=("Load", "Save")
            )

    def _setup_robot(self):
        r_names = [d["name"] for d in self.ss["robots"]]
        for i in range(len(r_names)):
            r_name = r_names[i]
            self.r_paths[r_name] = self.cwd + self.ss["robots"][i]["urdf_path"]
            self.r_instances[r_name] = yourdfpy.URDF.load(
                str(self.r_paths[r_name]),
                load_meshes=True,
                load_collision_meshes=True,
                build_scene_graph=True,
                build_collision_scene_graph=True,
            )
            self.r_vizs[r_name] = EnhancedViserUrdf(
                self.srv,
                urdf_or_path=self.r_instances[r_name],
                load_meshes=True,
                load_collision_meshes=True,
                mesh_color_override=self.ss["robots"][i].get(
                    "visual_color_override", None
                ),
                collision_mesh_color_override=self.ss["robots"][i].get(
                    "collision_color_override", (1.0, 0.0, 0.0, 0.4)
                ),
                root_node_name=f"/robot/{r_name}",
                root_position=np.array(self.ss["robots"][i]["position"]),
                root_wxyz=np.array(self.ss["robots"][i]["wxyz"]),
            )
            self.r_pose_origins[r_name] = (
                self.ss["robots"][i]["position"],
                self.ss["robots"][i]["wxyz"],
            )

        with self.srv.gui.add_folder("Robots"):
            # gui handle
            tab_group = self.srv.gui.add_tab_group()
            # ------------------
            dd_config_modes = {}
            btng_configs = {}

            # ------------------
            dd_trajs = {}
            sldr_joints = {}
            btn_resets = {}
            sldr_trajs = {}
            cb_loops = {}
            btng_players = {}
            cb_viss = {}

            def set_robot_visibility(r_name: str, mode: str) -> None:
                urdf_viz = self.r_vizs[r_name]
                if mode == "Visual":
                    urdf_viz.show_visual = True
                    urdf_viz.show_collision = False
                elif mode == "Collision":
                    urdf_viz.show_visual = False
                    urdf_viz.show_collision = True
                else:
                    urdf_viz.show_visual = True
                    urdf_viz.show_collision = True

            # interaction handle
            init_configs = {}
            for r_name, urdf_viz in self.r_vizs.items():
                with tab_group.add_tab(f"{r_name}"):
                    sldr_joints[r_name] = []
                    init_configs[r_name] = []
                    r = self.r_vizs[r_name].get_actuated_joint_limits().items()
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
                    btn_resets[r_name] = self.srv.gui.add_button("Reset Home")

                    # -----------------------------------
                    self.r_configs[r_name] = load_robot_config(
                        self.cwd + self.ss["robots_config"][0]["path"]
                    )
                    dd_config_modes[r_name] = self.srv.gui.add_dropdown(
                        label="Mode",
                        options=list(self.r_configs[r_name].keys()),
                        initial_value=list(self.r_configs[r_name].keys())[0],
                    )
                    btng_configs[r_name] = self.srv.gui.add_button_group(
                        label="Action", options=("Apply", "Save", "Remove")
                    )

                    self.srv.gui.add_divider()

                    # -----------------------------------
                    dd_trajs[r_name] = self.srv.gui.add_dropdown(
                        label="Trajectories",
                        options=[
                            d["name"] for d in self.ss["robots_trajectories"]
                        ],
                        initial_value=self.ss["robots_trajectories"][0]["name"],
                    )
                    btng_players[r_name] = self.srv.gui.add_button_group(
                        label="Player", options=("Load", "Play", "Pause", "Reset")
                    )
                    sldr_trajs[r_name] = self.srv.gui.add_slider(
                        "Progress", min=0.0, max=1.0, step=0.01, initial_value=0.0
                    )
                    cb_loops[r_name] = self.srv.gui.add_checkbox(
                        "AutoLoop", initial_value=False
                    )
                    cb_viss[r_name] = self.srv.gui.add_dropdown(
                        "Visibility",
                        options=("Visual", "Collision", "Both"),
                        initial_value="Visual",
                    )
                    set_robot_visibility(r_name, cb_viss[r_name].value)

                    @cb_viss[r_name].on_update
                    def _(_, r_name=r_name):
                        set_robot_visibility(r_name, cb_viss[r_name].value)

            # Connect sliders to URDF update
            def update_robot_config(r_name):
                config = np.array([s.value for s in sldr_joints[r_name]])
                self.r_vizs[r_name].update_cfg(config)

            for r_name, _ in self.r_vizs.items():
                for slider in sldr_joints[r_name]:
                    slider.on_update(
                        lambda _, r_name=r_name: update_robot_config(r_name)
                    )

            # apply initial configuration
            for r_name in self.r_vizs:
                update_robot_config(r_name)

            # bind reset buttons
            for r_name, btn in btn_resets.items():

                @btn.on_click
                def _(_, r_name=r_name):
                    for slider, init_val in zip(
                        sldr_joints[r_name], init_configs[r_name]
                    ):
                        slider.value = init_val

            # bind trajectory slider to update robot configuration
            def update_robot_config_traj(r_name):
                if self.traj is None or self._traj_sldr_prog_update:
                    return

                idx = int(sldr_trajs[r_name].value)
                if 0 <= idx < self.traj["N"]:
                    config = np.asarray(self.traj["points"][idx])
                    self.r_vizs[r_name].update_cfg(config)

            for r_name, slider in sldr_trajs.items():
                slider.on_update(
                    lambda _, r_name=r_name: update_robot_config_traj(r_name)
                )

            # bind config buttons
            for r_name, btng in btng_configs.items():

                def handle_config_action(event: viser.GuiEvent, r_name=r_name):
                    client = event.client
                    action = event.target.value
                    if action == "Apply":
                        q = self.r_configs[r_name][dd_config_modes[r_name].value]
                        self.r_vizs[r_name].update_cfg(q)
                    elif action == "Save":
                        pass
                    elif action == "Remove":
                        pass

                btng_configs[r_name].on_click(handle_config_action)

            # bind player buttons
            for r_name, slider in sldr_trajs.items():

                def handle_player_action(event: viser.GuiEvent, r_name=r_name):
                    client = event.client
                    action = event.target.value

                    if action == "Load":
                        fyaml = (
                            self.cwd + self.ss["robots_trajectories"][0]["path"]
                        )
                        self.traj = load_trajectory(fyaml)

                        client.add_notification(
                            title=f"Trajectory Loaded for {r_name}",
                            body=f"Trajectory has been successfully loaded",
                            auto_close_seconds=5,
                            with_close_button=True,
                        )

                        sldr_trajs[r_name].min = 0.0
                        sldr_trajs[r_name].max = max(self.traj["N"] - 1, 0)
                        sldr_trajs[r_name].step = 1.0
                        self._traj_sldr_prog_update = True
                        sldr_trajs[r_name].value = 0.0
                        self._traj_sldr_prog_update = False

                    elif action == "Play":
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
                        self._traj_sldr_prog_update = True
                        sldr_trajs[r_name].value = 0.0
                        self._traj_sldr_prog_update = False
                        update_robot_config_traj(r_name)

                btng_players[r_name].on_click(handle_player_action)

    def _stop_trajectory_playback(self):
        self._traj_btn_play_stop.set()
        if (
            self._traj_btn_play_thread is not None
            and self._traj_btn_play_thread.is_alive()
        ):
            self._traj_btn_play_thread.join(timeout=0.2)
        self._traj_btn_play_thread = None

    def _start_trajectory_playback(
        self,
        trajslider,
        trajautoloop,
        update_robot_config,
        r_name,
    ):
        if self.traj is None:
            return

        self._stop_trajectory_playback()
        self._traj_btn_play_stop.clear()

        def _run():
            time_from_start = np.asarray(self.traj.get("time_from_start", []))
            start_idx = int(trajslider.value)
            start_idx = max(0, min(start_idx, self.traj["N"] - 1))

            while self.traj is not None and not self._traj_btn_play_stop.is_set():
                prev_time = (
                    float(time_from_start[start_idx - 1])
                    if start_idx > 0 and start_idx - 1 < len(time_from_start)
                    else 0.0
                )

                for idx in range(start_idx, self.traj["N"]):
                    if self._traj_btn_play_stop.is_set():
                        return

                    self._traj_sldr_prog_update = True
                    trajslider.value = float(idx)
                    self._traj_sldr_prog_update = False
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

        self._traj_btn_play_thread = threading.Thread(target=_run, daemon=True)
        self._traj_btn_play_thread.start()

    def _setup_env(self):
        eo_names = [d["name"] for d in self.ss["env_objects"]]
        for i in range(len(eo_names)):
            eo_name = eo_names[i]
            self.eo_paths[eo_name] = (
                self.cwd + self.ss["env_objects"][i]["urdf_path"]
            )
            self.eo_instances[eo_name] = yourdfpy.URDF.load(
                str(self.eo_paths[eo_name]),
                load_meshes=True,
                load_collision_meshes=True,
                build_scene_graph=True,
                build_collision_scene_graph=True,
            )
            self.eo_vizs[eo_name] = EnhancedViserUrdf(
                self.srv,
                urdf_or_path=self.eo_instances[eo_name],
                load_meshes=True,
                load_collision_meshes=True,
                mesh_color_override=self.ss["env_objects"][i].get(
                    "visual_color_override", None
                ),
                collision_mesh_color_override=self.ss["env_objects"][i].get(
                    "collision_color_override", (1.0, 0.0, 0.0, 0.4)
                ),
                root_node_name=f"/env_objects/{eo_name}",
                root_position=np.array(self.ss["env_objects"][i]["position"]),
                root_wxyz=np.array(self.ss["env_objects"][i]["wxyz"]),
            )
            self.eo_pose_origins[eo_name] = (
                self.ss["env_objects"][i]["position"],
                self.ss["env_objects"][i]["wxyz"],
            )

        # apply initial configuration
        config = np.array([])  # no actuated joints = 0
        for eo_name in self.eo_vizs:
            self.eo_vizs[eo_name].update_cfg(config)

        # gui handle
        with self.srv.gui.add_folder("Environment Objects"):

            def set_env_visibility(mode: str):
                if dd_int.value == "All":
                    for eo_viz in self.eo_vizs.values():
                        if mode == "Visual":
                            eo_viz.show_visual = True
                            eo_viz.show_collision = False
                        elif mode == "Collision":
                            eo_viz.show_visual = False
                            eo_viz.show_collision = True
                        else:
                            eo_viz.show_visual = True
                            eo_viz.show_collision = True
                else:
                    eo_name = dd_int.value
                    eo_viz = self.eo_vizs[eo_name]
                    if mode == "Visual":
                        eo_viz.show_visual = True
                        eo_viz.show_collision = False
                    elif mode == "Collision":
                        eo_viz.show_visual = False
                        eo_viz.show_collision = True
                    else:
                        eo_viz.show_visual = True
                        eo_viz.show_collision = True

            eo_names_withall = ["All"] + eo_names
            dd_int = self.srv.gui.add_dropdown(
                label="Instances",
                options=eo_names_withall,
                initial_value="All",
            )
            dd_v = self.srv.gui.add_dropdown(
                "Visibility",
                options=("Visual", "Collision", "Both"),
                initial_value="Visual",
            )
            # apply initial visibility
            set_env_visibility(dd_v.value)

            @dd_v.on_update
            def _(_):
                set_env_visibility(dd_v.value)

    def _setup_taskspace(self):
        with self.srv.gui.add_folder("Task Space"):
            # gui handle
            ts_names = [d["name"] for d in self.ss["taskspaces"]]
            dd_ts = self.srv.gui.add_dropdown(
                label="Poses",
                options=ts_names,
                initial_value=ts_names[0],
            )
            btng_tslad = self.srv.gui.add_button_group(
                label="Action", options=("Load", "Add", "Delete")
            )

            # interaction handle
            ts_handles = []

            def _handle_btng_tslad(event: viser.GuiEvent) -> None:
                client = event.client
                action = event.target.value

                if action == "Load":
                    fyaml = (
                        self.cwd
                        + self.ss["taskspaces"][ts_names.index(dd_ts.value)][
                            "path"
                        ]
                    )
                    self.taskspace = load_taskspace(fyaml)

                    client.add_notification(
                        title="Task Space Loaded",
                        body="Task space poses have been successfully loaded.",
                        auto_close_seconds=5,
                        with_close_button=True,
                    )

                    for i in range(self.taskspace["N"]):
                        pose = self.taskspace["points"][i]
                        position = pose[:3]
                        if self.taskspace["standard"] == "xyz_qxqyqzqw":
                            # Convert to wxyz
                            quat = np.array([pose[6], pose[3], pose[4], pose[5]])
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

    def _setup_rtsp(self):
        with self.srv.gui.add_folder("RTSP"):
            # gui handle
            tst_names = [d["name"] for d in self.ss["taskspace_tours"]]
            dd_tst = self.srv.gui.add_dropdown(
                label="Tours",
                options=tst_names,
                initial_value=tst_names[0],
            )
            btng_tstour = self.srv.gui.add_button_group(
                label="Action", options=("Load", "ViewOrder")
            )
            sldr_tstour = self.srv.gui.add_slider(
                label="Progress",
                min=0.0,
                max=1.0,
                step=0.01,
                initial_value=0.0,
            )

            ts_handles = []

            def _handle_btng_tstour(event: viser.GuiEvent) -> None:
                client = event.client
                action = event.target.value

                if action == "Load":
                    fyaml = (
                        self.cwd
                        + self.ss["taskspace_tours"][
                            tst_names.index(dd_tst.value)
                        ]["path"]
                    )
                    self.taskspace_tour = load_taskspace_tour(fyaml)

                    for i in range(self.taskspace_tour["N"]):
                        pose = self.taskspace_tour["points"][i]
                        position = pose[:3]
                        if self.taskspace_tour["standard"] == "xyz_qxqyqzqw":
                            # Convert to wxyz
                            quat = np.array([pose[6], pose[3], pose[4], pose[5]])
                        else:
                            quat = np.array([pose[3], pose[4], pose[5], pose[6]])

                        ts_h_ = self.srv.scene.add_frame(
                            f"/task/tasks_tour/frame_{i}",
                            position=position,
                            wxyz=quat,
                            axes_length=0.1,
                            axes_radius=0.005,
                        )
                        ts_handles.append(ts_h_)

                    pose = self.taskspace_tour["points"]
                    ts_position = pose[:, :3]  # shape (N, 3)
                    ts_tour = self.taskspace_tour["order"]  # shape (N,)
                    ts_position_in_tour = ts_position[ts_tour]  # shape (N, 3)

                    # line segments
                    points = ts_position_in_tour.copy()
                    points = np.concatenate([points[:-1], points[1:]], axis=1)
                    points = points.reshape(-1, 2, 3)  # shape (N-1, 2, 3)

                    nn = points.shape[0] + 1
                    colors = gen_color_interp(n=nn, m="rgb01")
                    colors = np.array(colors)
                    colors = np.concatenate([colors[:-1], colors[1:]], axis=1)
                    colors = colors.reshape(-1, 2, 3)  # shape (N-1, 2, 3)

                    self.srv.scene.add_line_segments(
                        "/task/tasks_tour/tour_order",
                        points=points,
                        colors=colors,
                        line_width=5,
                    )
                    self.srv.scene.add_label(
                        "/task/tasks_tour/tour_start",
                        text="Start",
                        position=points[0, 0],
                    )
                    self.srv.scene.add_label(
                        "/task/tasks_tour/tour_end",
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
                        "/task/tasks_tour/tour_sphere",
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

                elif action == "ViewOrder":
                    with self.srv.gui.add_modal("Tour Order") as modal:
                        if self.taskspace_tour is None:
                            self.srv.gui.add_markdown(
                                "No tour loaded. Please load a tour first."
                            )
                        else:
                            self.srv.gui.add_markdown(
                                f"Tour Order: {self.taskspace_tour['order']}"
                            )
                        btn_close = self.srv.gui.add_button(
                            "Close", icon=viser.Icon.MOUSE
                        )

                        @btn_close.on_click
                        def _(_) -> None:
                            modal.close()

            btng_tstour.on_click(_handle_btng_tstour)

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
