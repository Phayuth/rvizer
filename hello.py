import time

import viser


def main() -> None:
    server = viser.ViserServer()
    with server.gui.add_folder("Environment Objects") as folder_env:
        btn_load = server.gui.add_button("Load Environment Object")

        @btn_load.on_click
        def _(event: viser.GuiEvent):
            with folder_env:
                bbbb = server.gui.add_button("Remove Environment Object")

                @bbbb.on_click
                def _(event: viser.GuiEvent):
                    bbbb.remove()

    # Example 0: Open Folder
    with server.gui.add_folder("Files"):
        gui_button = server.gui.add_button("Open", icon=viser.Icon.MOUSE)

    @gui_button.on_click
    def _(_) -> None:
        print("Open button clicked!")
        with server.gui.add_modal("File Opened") as modal:
            server.gui.add_markdown(
                "You have successfully opened a file. This is a modal dialog. fdasfdsafdsfadsfdsafdsafdsfdsafsfds"
            )
            server.gui.add_html(
                '<iframe width="300" height="300" src="https://www.youtube.com/embed/mVg108SO14Q" title="Pink Floyd · Another Brick In The Wall (Parts I-II-III)" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>'
            )

            close_button = server.gui.add_button("Close", icon=viser.Icon.MOUSE)

            @close_button.on_click
            def close_modal(_) -> None:
                modal.close()

    persistent_notif_button = server.gui.add_button(
        "Show persistent notification (default)"
    )

    @persistent_notif_button.on_click
    def _(event: viser.GuiEvent) -> None:
        client = event.client
        assert client is not None

        client.add_notification(
            title="Persistent notification",
            body="This can be closed manually and does not disappear on its own! Click the X button to close it.",
            with_close_button=True,
        )

    # 1. Initialize the tab group
    tab_group = server.gui.add_tab_group()

    # 2. Add tabs using a context manager
    with tab_group.add_tab("Tab 1", icon=viser.Icon.SETTINGS):
        slider1 = server.gui.add_slider(
            label="Slider 1", min=0.0, max=10.0, step=0.1, initial_value=0.0
        )
        checkbox1 = server.gui.add_checkbox(label="Option 1", initial_value=True)

    with tab_group.add_tab("Tab 2", icon=viser.Icon.INFO_CIRCLE):
        text_input = server.gui.add_text(label="Name", initial_value="Viser User")
        button = server.gui.add_button(label="Submit")

    # Example 1: Organizing with folders
    with server.gui.add_folder("Camera Controls"):
        with server.gui.add_folder("Position"):
            server.gui.add_slider(
                "X", min=-5.0, max=5.0, step=0.1, initial_value=0.0
            )
            server.gui.add_slider(
                "Y", min=-5.0, max=5.0, step=0.1, initial_value=2.0
            )
            server.gui.add_slider(
                "Z", min=-5.0, max=5.0, step=0.1, initial_value=3.0
            )

        with server.gui.add_folder("Rotation"):
            server.gui.add_slider(
                "Pitch", min=-180, max=180, step=1, initial_value=0
            )
            server.gui.add_slider(
                "Yaw", min=-180, max=180, step=1, initial_value=0
            )
            server.gui.add_slider(
                "Roll", min=-180, max=180, step=1, initial_value=0
            )
    with server.gui.add_folder("Camera Settings"):
        server.gui.add_multi_slider(
            "Field of View", min=30, max=120, step=1, initial_value=(60, 90)
        )
        server.gui.add_progress_bar(1)

    # Example 2: Scene objects organization
    with server.gui.add_folder("Scene Objects"):
        with server.gui.add_folder("Lighting"):
            server.gui.add_checkbox("Enable Lighting", initial_value=True)
            server.gui.add_slider(
                "Intensity", min=0.0, max=2.0, step=0.1, initial_value=1.0
            )
            server.gui.add_rgb("Color", initial_value=(255, 255, 255))

        with server.gui.add_folder("Objects"):  # GUI objects folder
            show_axes = server.gui.add_checkbox(
                "Show Coordinate Axes", initial_value=True
            )
            server.gui.add_checkbox("Show Grid", initial_value=False)

            with server.gui.add_folder("Sphere"):
                sphere_radius = server.gui.add_slider(
                    "Radius", min=0.1, max=2.0, step=0.1, initial_value=0.5
                )
                sphere_color = server.gui.add_rgb(
                    "Color", initial_value=(255, 0, 0)
                )
                sphere_visible = server.gui.add_checkbox(
                    "Visible", initial_value=True
                )

    # Example 3: Settings and preferences
    with server.gui.add_folder("Settings"):
        with server.gui.add_folder("Display"):
            server.gui.add_rgb("Background", initial_value=(40, 40, 40))
            server.gui.add_checkbox("Wireframe Mode", initial_value=False)

        with server.gui.add_folder("Performance"):
            server.gui.add_slider(
                "FPS Limit", min=30, max=120, step=10, initial_value=60
            )
            server.gui.add_dropdown(
                "Quality",
                options=["Low", "Medium", "High"],
                initial_value="Medium",
            )

    # Add some visual objects to demonstrate the controls
    server.scene.add_icosphere(
        name="demo_sphere",
        radius=sphere_radius.value,
        color=(
            sphere_color.value[0] / 255.0,
            sphere_color.value[1] / 255.0,
            sphere_color.value[2] / 255.0,
        ),
        position=(0.0, 0.0, 0.0),
        visible=sphere_visible.value,
    )

    if show_axes.value:
        server.scene.add_frame("axes", axes_length=1.0, axes_radius=0.02)

    print("This example shows GUI organization with folders.")
    print("The sphere demonstrates some interactive controls.")

    print("Explore the organized GUI controls!")
    print("Notice how folders help group related functionality.")

    while True:
        time.sleep(0.1)


if __name__ == "__main__":
    main()
