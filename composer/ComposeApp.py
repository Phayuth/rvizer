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


class ComposeApp:

    def __init__(self, port=8080):
        self.server = viser.ViserServer(port=port)

        # urdf_path = Path("/rb3_730es_u/rb3_730es_u_armright.urdf")
        # urdf_path = Path("/humanoid_urdf/lift.urdf")
        # urdf_path = Path("/humanoid_urdf/humanoid_urdf.urdf")

        show_collision = True
        # Load URDF
        # self.urdf = yourdfpy.URDF.load(
        #     str(urdf_path),  # urdf_path,
        #     build_scene_graph=True,
        #     load_meshes=True,
        #     build_collision_scene_graph=show_collision,
        #     load_collision_meshes=show_collision,
        # )
        robot_name = "robotiq_2f85"
        self.urdf = load_robot_description(
            robot_name + "_description",
            load_meshes=True,
            build_scene_graph=True,
            load_collision_meshes=show_collision,
            build_collision_scene_graph=show_collision,
        )

        # Enhanced URDF visualizer with per-link control
        self.urdf_viz = EnhancedViserUrdf(
            self.server,
            urdf_or_path=self.urdf,
            load_meshes=True,
            load_collision_meshes=show_collision,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.4),
        )
        # self.urdf.base_link

        self.joint_sliders = []
        self._setup_robot_controls()
        self._setup_stack()

    def _setup_robot_controls(self):
        """Setup robot configuration and visibility controls."""
        with self.server.gui.add_folder("🤖 Robot Controls"):
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
            reset_joints_btn = self.server.gui.add_button("🏠 Reset to Home")

            @reset_joints_btn.on_click
            def _(_):
                for slider, init_val in zip(self.joint_sliders, initial_config):
                    slider.value = init_val

    def _setup_stack(self):
        """Setup the GUI stack for the application."""
        with self.server.gui.add_folder("📦 Stack") as folder_stack:
            btn_load = self.server.gui.add_button("Load Environment Object")

            @btn_load.on_click
            def _(event: viser.GuiEvent):
                with folder_stack:
                    bbbb = self.server.gui.add_button("Remove Environment Object")

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


if __name__ == "__main__":
    app = ComposeApp(port=8080)
    app.run()
