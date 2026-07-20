import os
import copy
import numpy as np
from pathlib import Path
from lxml import etree as ET
import viser.transforms as tf
import yourdfpy
import robot_descriptions
from robot_descriptions.loaders.yourdfpy import load_robot_description
from aljnu_robot_descriptions import ALJNU_DESCRIPTIONS

rsrcrtsp = os.environ["RSRCRTSP_DIR"]


def see_available_robot_descriptions():
    print(robot_descriptions.DESCRIPTIONS)  # list all available robot descriptions


def download_robot_description(robot_name="panda"):
    collision = True
    urdf = load_robot_description(
        robot_name + "_description",
        load_meshes=True,
        build_scene_graph=True,
        load_collision_meshes=collision,
        build_collision_scene_graph=collision,
    )

    urdf.write_xml_file(str(Path(f"{robot_name}.urdf")))
    return urdf


def load_robot_description_from_file(urdf_path):
    collision = True
    urdf = yourdfpy.URDF.load(
        str(urdf_path),  # urdf_input,
        build_scene_graph=True,
        load_meshes=True,
        build_collision_scene_graph=collision,
        load_collision_meshes=collision,
    )
    return urdf


def append_prefix_to_urdf(prefix, urdf: yourdfpy.URDF):
    # rename robot name
    urdf.robot.name = prefix + urdf.robot.name

    # rename base link
    urdf._base_link = prefix + urdf.base_link

    # rename links
    for link in urdf.robot.links:
        link.name = prefix + link.name

    # rename joints
    for joint in urdf.robot.joints:
        joint.name = prefix + joint.name
        joint.parent = prefix + joint.parent
        joint.child = prefix + joint.child

    return urdf


def urdf_details(urdf: yourdfpy.URDF):
    print(f"Robot name: {urdf.robot.name}")
    print(f"Base link: {urdf.base_link}")
    print(f"Number of links: {len(urdf.robot.links)}")
    print(f"Number of joints: {len(urdf.robot.joints)}")
    print("Links:")
    for link in urdf.robot.links:
        print(f"  {link.name}")

    print("\nJoints:")
    for joint in urdf.robot.joints:
        print(f"  {joint.name} (parent: {joint.parent}, child: {joint.child})")

    return urdf


def get_transform(urdf: yourdfpy.URDF):
    # first arg is parent link
    # second arg is child link
    H = urdf.get_transform("panda_link0", "panda_hand_tcp")
    return H


def make_link(name):
    """
    <link name="name" />
    """
    link_template = f"""<link name="{name}" />"""
    return link_template


def make_fixed_joint(name, parent, child, xyz=[0, 0, 0], rpy=[0, 0, 0]):
    """
    <joint name="my_fixed_joint" type="fixed">
      <parent link="base_link"/>
      <child link="camera_link"/>
      <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
    </joint>
    """
    joint_template = f"""
    <joint name="{name}" type="fixed">
        <parent link="{parent}"/>
        <child link="{child}"/>
        <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>
    </joint>
    """
    return joint_template


def merge_urdfs(urdf_dict, outputname="merged", world_link="world_link"):
    urdfs = []
    for urdf_info in urdf_dict:
        urdf_path = urdf_info["urdf_path"]
        urdf = load_robot_description_from_file(urdf_path)
        urdf = append_prefix_to_urdf(urdf_info["prefix"], urdf)
        urdfs.append(urdf)
    xmls = [urdf.write_xml() for urdf in urdfs]
    roots = [xml.getroot() for xml in xmls]

    merged_root = ET.Element("robot", name=outputname)

    for root in roots:
        for child in root:
            merged_root.append(copy.deepcopy(child))

    merged_tree = ET.ElementTree(merged_root)

    # add world link as root to all
    l = make_link(world_link)
    merged_root.append(ET.fromstring(l))

    # add fixed joints to connect each robot to the world link
    for i, urdf in enumerate(urdfs):
        fj = make_fixed_joint(
            f"robot{i+1}_to_world",
            world_link,
            urdf.base_link,
            xyz=urdf_dict[i]["position"],
            rpy=_wxyz_to_rpy(urdf_dict[i]["wxyz"]),
        )
        merged_root.append(ET.fromstring(fj))

    # write merged URDF to file
    ET.indent(merged_tree, space="  ")
    merged_tree.write(
        outputname + ".urdf",
        pretty_print=True,
        xml_declaration=True,
        encoding="utf-8",
    )


def _wxyz_to_rpy(wxyz):
    """
    Convert quaternion (w, x, y, z) to roll, pitch, yaw (rpy)
    """
    w, x, y, z = wxyz
    r = tf.SO3.from_quaternion_xyzw(np.array([x, y, z, w]))
    rpy = r.as_rpy_radians()
    return rpy.roll.item(), rpy.pitch.item(), rpy.yaw.item()


if __name__ == "__main__":
    merge_dict = []
    urdf1 = {
        "prefix": "shelf_1_",
        "urdf_path": ALJNU_DESCRIPTIONS["shelf"],
        "position": [0.0, 0.75, 0.0],
        "wxyz": [0.0, 0.0, 0.0, 1.0],
    }
    merge_dict.append(urdf1)
    urdf2 = {
        "prefix": "shelf_2_",
        "urdf_path": ALJNU_DESCRIPTIONS["shelf"],
        "position": [0.0, -0.75, 0.0],
        "wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    merge_dict.append(urdf2)
    urdf3 = {
        "prefix": "shelf_3_",
        "urdf_path": ALJNU_DESCRIPTIONS["shelf"],
        "position": [0.75, 0.0, 0.0],
        "wxyz": [0.70710678, 0.0, 0.0, 0.70710678],
    }
    merge_dict.append(urdf3)
    urdf4 = {
        "prefix": "plane_",
        "urdf_path": ALJNU_DESCRIPTIONS["plane"],
        "position": [0.0, 0.0, 0.0],
        "wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    merge_dict.append(urdf4)

    merge_urdfs(merge_dict)
