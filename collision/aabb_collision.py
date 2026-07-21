import os
import yourdfpy
import numpy as np
import yaml
import pathlib
import viser.transforms as tf
from aljnu_robot_descriptions import ALJNU_DESCRIPTIONS, dir_urdfs
import argparse

np.set_printoptions(precision=4, suppress=True, linewidth=200)


def is_axis_aligned_rotation(R, atol=1e-6):
    """
    Returns True if R is a rotation consisting only of 90° increments
    (i.e., a signed permutation matrix).
    """
    R = np.asarray(R)
    if R.shape != (3, 3):
        return False
    if not np.isclose(np.linalg.det(R), 1.0, atol=atol):
        return False
    R_round = np.round(R)
    if not np.allclose(R, R_round, atol=atol):
        return False
    if not np.all(np.isin(R_round, [-1, 0, 1])):
        return False
    if not np.all(np.sum(np.abs(R_round), axis=0) == 1):
        return False
    if not np.all(np.sum(np.abs(R_round), axis=1) == 1):
        return False
    return True


def parse_boxes_from_urdf(urdf: yourdfpy.URDF):
    boxes_s = []  # size of boxes in local link frame
    boxes_o = []  # origin of boxes in local link frame
    boxes_l = []  # link name of boxes in local link frame

    for link in urdf.robot.links:
        for v in link.visuals:
            o = v.origin
            g = v.geometry
            if g.box is not None:
                boxes_s.append(g.box.size)
                boxes_o.append(o)
                boxes_l.append(link.name)
            else:
                raise ValueError(f"Geometry is not a box: {g}")

    return boxes_s, boxes_o, boxes_l


def box_to_world_aabb(H_WL, H_LB, local_size):
    """
    Determine the box center and size in world frame.
    Assumes all rotations are multiples of 90°.
    """
    H_WB = H_WL @ H_LB

    center = H_WB[:3, 3]
    size = np.abs(H_WB[:3, :3]) @ np.asarray(local_size)

    return {
        "center": center,
        "size": size,
    }


def transform_box_to_target_link_aabb(
    boxes_s, boxes_o, boxes_l, urdf: yourdfpy.URDF, target_link
):
    # Target Link should be robot base link as we check collision from robot base link to target link

    aabbs = {}
    for i in range(len(boxes_s)):
        is_aligned = is_axis_aligned_rotation(boxes_o[i][:3, :3])
        if not is_aligned:
            raise ValueError(f"Box {i} in link {boxes_l[i]} is non-axis-aligned.")
        H_WL = urdf.get_transform(boxes_l[i], target_link)
        H_LB = boxes_o[i]
        aabb = box_to_world_aabb(H_WL, H_LB, boxes_s[i])
        aabbs["box_" + str(i)] = {
            "link": boxes_l[i],
            "center": aabb["center"].tolist(),
            "size": aabb["size"].tolist(),
        }

    return aabbs


def parse_links_and_transforms_to_target(urdf: yourdfpy.URDF, target_link: str):
    # Target Link should be robot base link as we check collision from robot base link to target link
    link_names = [link.name for link in urdf.robot.links]
    links_in_target_link = {}
    for link_name in link_names:
        H_WL = urdf.get_transform(link_name, target_link)
        H = tf.SE3.from_matrix(H_WL)
        qt = H.wxyz_xyz.tolist()
        xyz_wxyz = [qt[4], qt[5], qt[6], qt[0], qt[1], qt[2], qt[3]]
        links_in_target_link[link_name] = {}
        links_in_target_link[link_name]["xyz"] = xyz_wxyz[0:3]
        links_in_target_link[link_name]["wxyz"] = xyz_wxyz[3:7]

    return link_names, links_in_target_link


def generate_static_collision(urdf: yourdfpy.URDF, target):
    # Parse boxes from URDF
    boxes_s, boxes_o, boxes_l = parse_boxes_from_urdf(urdf)

    # Compute AABBs in World Link Frame
    aabbs = transform_box_to_target_link_aabb(
        boxes_s, boxes_o, boxes_l, urdf, target
    )

    # print the detials of each box
    for i, (box_name, box_data) in enumerate(aabbs.items()):
        print(
            f"Box {i}: size={boxes_s[i]}, origin={boxes_o[i]}, link={boxes_l[i]}"
        )
        print(
            f"  World AABB: center={box_data['center']}, size={box_data['size']}"
        )
        print("".center(50, "-"))

    # generate metadata for the collision data
    link_names, links_in_target_link = parse_links_and_transforms_to_target(
        urdf, target
    )
    data = {"metadata": {}}
    data["metadata"]["name"] = name
    data["metadata"]["base_link"] = urdf.base_link
    data["metadata"]["target_link"] = target
    data["metadata"]["total_boxes"] = len(boxes_s)
    data["metadata"]["total_links"] = len(link_names)
    data["metadata"]["links_boxes_count"] = {link: 0 for link in link_names}
    for box in aabbs.values():
        link = box["link"]
        data["metadata"]["links_boxes_count"][link] += 1
    data["metadata"]["links_in_target_link"] = links_in_target_link

    data["collision_in_target_link"] = aabbs

    return data


def write_collision_data_to_yaml(data):
    name = data["metadata"]["name"]
    ff = os.path.join(dir_urdfs, f"{name}_collision.yaml")
    with open(ff, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def verify_load_collision_data_from_yaml(name):
    ff = os.path.join(dir_urdfs, f"{name}_collision.yaml")
    with open(ff, "r") as f:
        data = yaml.safe_load(f)

    print(f"Loaded collision data for robot: {data['metadata']['name']}")
    print(f"Total boxes: {data['metadata']['total_boxes']}")
    print(f"Total links: {data['metadata']['total_links']}")
    print("Links and their box counts:")
    for link, count in data["metadata"]["links_boxes_count"].items():
        print(f"  {link}: {count} boxes")
    print(f"Target link: {data['metadata']['target_link']}")

    n = len(data["collision_in_target_link"])
    box_in_base = np.zeros((n, 4, 4))
    boxsz_in_base = np.zeros((n, 3))

    collision_in_target_link = data["collision_in_target_link"]
    for i, (box_name, box_data) in enumerate(collision_in_target_link.items()):
        link_name = box_data["link"]
        center = np.array(box_data["center"])
        size = np.array(box_data["size"])
        print(f"Box {box_name}: link={link_name}, center={center}, size={size}")
        H = np.eye(4)
        H[:3, 3] = center
        box_in_base[i] = H
        boxsz_in_base[i] = size

    print("Box transforms in base link frame:")
    print(box_in_base)
    print("Box sizes in base link frame:")
    print(boxsz_in_base)


if __name__ == "__main__":
    # parser = argparse.ArgumentParser(
    #     description="Generate static collision data for a URDF robot."
    # )
    # parser.add_argument("--urdf", help="Path to the URDF file")
    # parser.add_argument("--robot-name", help="Name of the robot")
    # args = parser.parse_args()

    urdf_path = ALJNU_DESCRIPTIONS["three_shelf"]
    name = pathlib.Path(urdf_path).stem  # get the filename without extension
    show_collision = True
    urdf = yourdfpy.URDF.load(
        str(urdf_path),  # urdf_path,
        build_scene_graph=True,
        load_meshes=True,
        build_collision_scene_graph=show_collision,
        load_collision_meshes=show_collision,
    )

    data = generate_static_collision(urdf, "world_link")
    write_collision_data_to_yaml(data)
    verify_load_collision_data_from_yaml(name)
