import numpy as np
import yourdfpy
import time
import threading
import yaml
from pathlib import Path
import pprint
import robot_descriptions

# print(robot_descriptions.DESCRIPTIONS)
urdf_path = Path("/rb3_730es_u/rb3_730es_u.urdf")
# urdf_path = Path("/humanoid_urdf/lift.urdf")
show_collision = True

# Load URDF
if urdf_path is not None:
    urdf = yourdfpy.URDF.load(
        str(urdf_path),  # urdf_path,
        build_scene_graph=True,
        load_meshes=True,
        build_collision_scene_graph=show_collision,
        load_collision_meshes=show_collision,
    )
    urdf_path = urdf_path
urdf.show()

print("URDF loaded successfully.")
print(urdf.base_link)
print(urdf.get_transform(urdf.base_link, urdf.base_link))
print(urdf.robot.links)
for link in urdf.robot.links:
    pprint.pprint(
        f"Link: {link.name}, Visuals: {len(link.visuals)}, Collisions: {len(link.collisions)}"
    )


# 2. Define the prefix you want to add
prefix = "armright_"

# Rename links
for link in urdf.robot.links:
    link.name = prefix + link.name

# Rename joints and their references
for joint in urdf.robot.joints:
    joint.name = prefix + joint.name
    joint.parent = prefix + joint.parent
    joint.child = prefix + joint.child

urdf.write_xml_file("/rb3_730es_u/rb3_730es_u_armright.urdf")
