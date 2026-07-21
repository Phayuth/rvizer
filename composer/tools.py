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


def see_available_robot_descriptions():
    print(robot_descriptions.DESCRIPTIONS)  # list all available robot descriptions


def download_robot_description(robot_name="panda", save=False):
    collision = True
    urdf = load_robot_description(
        robot_name + "_description",
        load_meshes=True,
        build_scene_graph=True,
        load_collision_meshes=collision,
        build_collision_scene_graph=collision,
    )

    if save:
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


def urdf_details(urdf: yourdfpy.URDF):
    print(f"Robot name: {urdf.robot.name}")
    print(f"Base link: {urdf.base_link}")
    print(f"Number of links: {len(urdf.robot.links)}")
    print(f"Number of joints: {len(urdf.robot.joints)}")
    print("Links:")
    for link in urdf.robot.links:
        print(f"  {link.name}")
        print(
            f"  └-Visuals: {len(link.visuals)} | Collisions: {len(link.collisions)}"
        )
    print("\nJoints:")
    for joint in urdf.robot.joints:
        print(f"  {joint.name} (parent: {joint.parent}, child: {joint.child})")


def get_transform(urdf: yourdfpy.URDF):
    # first arg is parent link
    # second arg is child link
    H = urdf.get_transform("panda_link0", "panda_hand_tcp")
    return H


if __name__ == "__main__":
    urdf = download_robot_description("panda")
    urdf_details(urdf)
