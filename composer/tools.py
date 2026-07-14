import numpy as np
import yourdfpy
from pathlib import Path
import robot_descriptions
from robot_descriptions.loaders.yourdfpy import load_robot_description


def download_robot_description(robot_name="panda"):
    # print(robot_descriptions.DESCRIPTIONS) # list all available robot descriptions

    show_collision = True
    urdf = load_robot_description(
        robot_name + "_description",
        load_meshes=True,
        build_scene_graph=True,
        load_collision_meshes=show_collision,
        build_collision_scene_graph=show_collision,
    )

    urdf.write_xml_file(str(Path(f"{robot_name}.urdf")))


def append_prefix_to_urdf_links_joints(urdf_input, prefix, urdf_output):
    show_collision = True
    urdf = yourdfpy.URDF.load(
        str(urdf_input),  # urdf_input,
        build_scene_graph=True,
        load_meshes=True,
        build_collision_scene_graph=show_collision,
        load_collision_meshes=show_collision,
    )

    # rename links
    for link in urdf.robot.links:
        link.name = prefix + link.name

    # rename joints
    for joint in urdf.robot.joints:
        joint.name = prefix + joint.name
        joint.parent = prefix + joint.parent
        joint.child = prefix + joint.child

    # write the modified URDF to a new file
    urdf.write_xml_file(str(urdf_output))


def urdf_details(urdf_input):
    show_collision = True
    urdf = yourdfpy.URDF.load(
        str(urdf_input),  # urdf_input,
        build_scene_graph=True,
        load_meshes=True,
        build_collision_scene_graph=show_collision,
        load_collision_meshes=show_collision,
    )
    print(f"Base link: {urdf.base_link}")

    print("Links:")
    for link in urdf.robot.links:
        print(f"  {link.name}")

    print("\nJoints:")
    for joint in urdf.robot.joints:
        print(f"  {joint.name} (parent: {joint.parent}, child: {joint.child})")


if __name__ == "__main__":
    # urdf_input = Path("/rb3_730es_u/rb3_730es_u.urdf")
    # urdf_output = Path("/rb3_730es_u/rb3_730es_u_armright.urdf")
    # prefix = "armright_"
    # append_prefix_to_urdf_links_joints(urdf_input, prefix, urdf_output)

    show_collision = True
    robot_name = "panda"
    urdf = load_robot_description(
        robot_name + "_description",
        load_meshes=True,
        build_scene_graph=True,
        load_collision_meshes=show_collision,
        build_collision_scene_graph=show_collision,
    )
    print(urdf.base_link)
    print("Links:")
    for link in urdf.robot.links:
        print(f"  {link.name}")

    print("\nJoints:")
    for joint in urdf.robot.joints:
        print(f"  {joint.name} (parent: {joint.parent}, child: {joint.child})")

    TF = urdf.get_transform("panda_link0", "panda_hand_tcp")
    print(f"==>> TF: \n{TF}")
