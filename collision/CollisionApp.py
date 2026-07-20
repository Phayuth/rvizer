import numpy as np
import viser
import viser.transforms as tf
import yourdfpy
import time
import yaml
from robot_descriptions.loaders.yourdfpy import load_robot_description
from bubblify.core import EnhancedViserUrdf
import pathlib
from aljnu_robot_descriptions import ALJNU_DESCRIPTIONS


class CollisionApp:

    def __init__(self, port=8080):
        self.srv = viser.ViserServer(port=port)

        show_collision = True
        # Dynamics URDF
        urdf_path = ALJNU_DESCRIPTIONS["ur5e_sph"]
        self.robot_name = pathlib.Path(urdf_path).stem
        self.urdf = yourdfpy.URDF.load(
            str(urdf_path),
            build_scene_graph=True,
            load_meshes=True,
            build_collision_scene_graph=show_collision,
            load_collision_meshes=show_collision,
        )
        # self.robot_name = "panda"
        # self.urdf = load_robot_description(
        #     self.robot_name + "_description",
        #     load_meshes=True,
        #     build_scene_graph=True,
        #     load_collision_meshes=show_collision,
        #     build_collision_scene_graph=show_collision,
        # )
        self.root_node_name = "/moving_base"
        self.urdf_viz = EnhancedViserUrdf(
            self.srv,
            urdf_or_path=self.urdf,
            load_meshes=True,
            load_collision_meshes=show_collision,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.4),
            root_node_name=self.root_node_name,
        )

        # static URDF
        env = ALJNU_DESCRIPTIONS["three_shelf"]
        self.env_name = pathlib.Path(env).stem
        self.urdf_env = yourdfpy.URDF.load(
            str(env),  # env,
            build_scene_graph=True,
            load_meshes=True,
            build_collision_scene_graph=show_collision,
            load_collision_meshes=show_collision,
        )
        self.urdf_env_viz = EnhancedViserUrdf(
            self.srv,
            urdf_or_path=self.urdf_env,
            load_meshes=True,
            load_collision_meshes=show_collision,
            collision_mesh_color_override=(0.0, 1.0, 0.0, 0.4),
            # root_node_name="/static_env",
        )

        self.joint_sliders = []
        self._setup_compose_config()
        self._setup_robot_controls()
        self._setup_env()
        self._setup_tf()
        self._setup_stack()

    def _setup_compose_config(self):
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

    def _setup_robot_controls(self):
        """Setup robot configuration and visibility controls."""
        with self.srv.gui.add_folder("🤖 Robot Controls"):

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

                slider = self.srv.gui.add_slider(
                    label=joint_name,
                    min=lower,
                    max=upper,
                    step=1e-3,
                    initial_value=initial_pos,
                )
                self.joint_sliders.append(slider)
                initial_config.append(initial_pos)

            # link
            link_names = [link.name for link in self.urdf.robot.links]
            link_handles = []
            for i in range(len(link_names)):
                l = self.srv.scene.add_frame(
                    f"/{self.robot_name}/link_{link_names[i]}_frame",
                    position=(0, 0, 0),
                    wxyz=(1, 0, 0, 0),
                    scale=0.05,
                )
                link_handles.append(l)

            # Connect sliders to URDF update worked
            # def update_robot_config():
            #     config = np.array([s.value for s in self.joint_sliders])
            #     self.urdf_viz.update_cfg(config)

            #     for i, link_name in enumerate(link_names):
            #         T_link_world = self.urdf.get_transform(
            #             link_name, self.urdf.base_link, collision_geometry=True
            #         )
            #         position = T_link_world[:3, 3]
            #         wxyz = tf.SO3.from_matrix(T_link_world[:3, :3]).wxyz
            #         link_handles[i].position = position
            #         link_handles[i].wxyz = wxyz

            # Reset button
            reset_joints_btn = self.srv.gui.add_button("🏠 Reset to Home")

            @reset_joints_btn.on_click
            def _(_):
                for slider, init_val in zip(self.joint_sliders, initial_config):
                    slider.value = init_val

            for h in link_handles:

                def handle_link_click(event: viser.GuiEvent, h=h):
                    event.client.add_notification(
                        f"Clicked on link: {h.name}",
                        body=f"Position: {h.position}, Orientation (wxyz): {h.wxyz}",
                        auto_close_seconds=3.0,
                    )

                h.on_click(handle_link_click)

            btn_reset_base = self.srv.gui.add_button("🏠 Reset Base Position")
            base = self.srv.scene.add_frame(
                self.root_node_name,
                position=(0, 0, 0),
                wxyz=(1, 0, 0, 0),
                scale=0.3,
            )
            gizmo = self.srv.scene.add_transform_controls(
                name="/gizmo",
                scale=0.3,
                active_axes=(True, True, True),
            )

            @gizmo.on_update
            def _(event):
                base.position = gizmo.position
                base.wxyz = gizmo.wxyz
                update_robot_config()

            def update_robot_config():
                config = np.array([s.value for s in self.joint_sliders])
                self.urdf_viz.update_cfg(config)

                for i, link_name in enumerate(link_names):
                    T_link_base = self.urdf.get_transform(
                        link_name, self.urdf.base_link, collision_geometry=True
                    )

                    base_to_world_position = base.position
                    base_to_world_wxyz = base.wxyz
                    base_to_world_xyzw = np.array(
                        (
                            base_to_world_wxyz[1],
                            base_to_world_wxyz[2],
                            base_to_world_wxyz[3],
                            base_to_world_wxyz[0],
                        )
                    )
                    base_to_world_matrix = tf.SE3.from_rotation_and_translation(
                        rotation=tf.SO3.from_quaternion_xyzw(base_to_world_xyzw),
                        translation=np.array(base_to_world_position),
                    )

                    T_link_world = base_to_world_matrix.as_matrix() @ T_link_base
                    position_world = T_link_world[:3, 3]
                    wxyz_world = tf.SO3.from_matrix(T_link_world[:3, :3]).wxyz

                    link_handles[i].position = position_world
                    link_handles[i].wxyz = wxyz_world

            for slider in self.joint_sliders:
                slider.on_update(lambda _: update_robot_config())

            # Apply initial configuration
            update_robot_config()

    def _setup_env(self):
        link_names = [link.name for link in self.urdf_env.robot.links]
        link_handles = []
        for i in range(len(link_names)):
            l = self.srv.scene.add_frame(
                f"/{self.env_name}/link_{link_names[i]}_frame",
                position=(0, 0, 0),
                wxyz=(1, 0, 0, 0),
                scale=0.05,
            )
            link_handles.append(l)

        for h in link_handles:

            def handle_link_click(event: viser.GuiEvent, h=h):
                event.client.add_notification(
                    f"Clicked on link: {h.name}",
                    body=f"Position: {h.position}, Orientation (wxyz): {h.wxyz}",
                    auto_close_seconds=3.0,
                )

            h.on_click(handle_link_click)

        def update_robot_config():
            config = np.array([])  # no actuated joints = 0
            self.urdf_env_viz.update_cfg(config)

            for i, link_name in enumerate(link_names):
                T_link_base = self.urdf_env.get_transform(
                    link_name, self.urdf_env.base_link, collision_geometry=True
                )

                link_handles[i].position = T_link_base[:3, 3]
                link_handles[i].wxyz = tf.SO3.from_matrix(T_link_base[:3, :3]).wxyz

        update_robot_config()

    def _setup_tf(self):
        print("Links:")
        link_names = [link.name for link in self.urdf.robot.links]
        joint_names = [joint.name for joint in self.urdf.robot.joints]
        joint_connections = [
            (joint.name, joint.parent, joint.child)
            for joint in self.urdf.robot.joints
        ]

        with self.srv.gui.add_folder("🧭 TF Viewer"):
            t_base = self.srv.gui.add_text(
                "Base Link", initial_value=self.urdf.base_link, disabled=True
            )
            self.srv.gui.add_divider()

            dd_links = self.srv.gui.add_dropdown(
                label="Links",
                options=link_names,
                initial_value=link_names[0],
            )
            self.srv.gui.add_divider()

            dd_joints = self.srv.gui.add_dropdown(
                label="Joints",
                options=joint_names,
                initial_value=joint_names[0],
            )
            t_pas = self.srv.gui.add_text(
                "Parent Link",
                initial_value=joint_connections[0][1],
                disabled=True,
            )
            t_chs = self.srv.gui.add_text(
                "Child Link",
                initial_value=joint_connections[0][2],
                disabled=True,
            )

            @dd_joints.on_update
            def _(event: viser.GuiEvent):
                t_pas.value = joint_connections[
                    joint_names.index(dd_joints.value)
                ][1]
                t_chs.value = joint_connections[
                    joint_names.index(dd_joints.value)
                ][2]

    def _setup_stack(self):
        """Setup the GUI stack for the application."""
        with self.srv.gui.add_folder("📦 Stack") as folder_stack:
            btn_load = self.srv.gui.add_button("Load Environment Object")

            @btn_load.on_click
            def _(event: viser.GuiEvent):
                with folder_stack:
                    bbbb = self.srv.gui.add_button("Remove Environment Object")

                    @bbbb.on_click
                    def _(event: viser.GuiEvent):
                        bbbb.remove()

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


def test_compose_app():
    app = CollisionApp(port=8080)
    app.run()


# def


if __name__ == "__main__":
    test_compose_app()
