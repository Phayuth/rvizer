import yourdfpy
import numpy as np
import yaml
import pathlib
import viser.transforms as tf

urdf_path = "/airbus_shopfloor.urdf"
robot_name = pathlib.Path(urdf_path).stem  # get the filename without extension
show_collision = True
urdf = yourdfpy.URDF.load(
    str(urdf_path),  # urdf_path,
    build_scene_graph=True,
    load_meshes=True,
    build_collision_scene_graph=show_collision,
    load_collision_meshes=show_collision,
)

link_names = [link.name for link in urdf.robot.links]
selected_as_world_link = link_names[0]  # select the first link as the world link

# AABB collision only available for these types of pair
# AABB(box) to AABB(box)
# AABB(box) to sphere
# sphere to sphere


def is_axis_aligned_rotation(R, atol=1e-6):
    """
    Returns True if R is a rotation consisting only of 90° increments
    (i.e., a signed permutation matrix).
    """
    R = np.asarray(R)

    if R.shape != (3, 3):
        return False

    # Proper rotation
    if not np.isclose(np.linalg.det(R), 1.0, atol=atol):
        return False

    # Entries should be close to {-1,0,1}
    R_round = np.round(R)
    if not np.allclose(R, R_round, atol=atol):
        return False

    if not np.all(np.isin(R_round, [-1, 0, 1])):
        return False

    # Exactly one ±1 per row/column
    if not np.all(np.sum(np.abs(R_round), axis=0) == 1):
        return False
    if not np.all(np.sum(np.abs(R_round), axis=1) == 1):
        return False

    return True


boxes = []
boxe_o = []
boxe_l = []

# check how many geometry objects are in the URDF
for link in urdf.robot.links:
    print(f"Link: {link.name}")
    print(f"  Visuals: {len(link.visuals)}")
    print(f"  Collisions: {len(link.collisions)}")
    for v in link.visuals:
        o = v.origin
        g = v.geometry
        if g.box is not None:
            boxes.append(g.box.size)
            boxe_o.append(o)
            boxe_l.append(link.name)
        else:
            print(f"  Visual geometry is not a box: {g}")


def box_to_world_aabb(H_WL, H_LB, local_size):
    """
    Convert a box defined in its own frame to a world-space AABB.

    Assumes all rotations are multiples of 90°.
    """
    H_WB = H_WL @ H_LB

    center = H_WB[:3, 3]
    size = np.abs(H_WB[:3, :3]) @ np.asarray(local_size)

    return {
        "center": center,
        "size": size,
    }


link_tf_in_world_link_xyz_wxyz = {}
for link_name in link_names:
    H_WL = urdf.get_transform(link_name, selected_as_world_link)
    H = tf.SE3.from_matrix(H_WL)
    qt = H.wxyz_xyz.tolist()
    xyz_wxyz = [qt[4], qt[5], qt[6], qt[0], qt[1], qt[2], qt[3]]
    link_tf_in_world_link_xyz_wxyz[link_name] = xyz_wxyz


aabbs = {}

for i in range(len(boxes)):
    print(f"Box {i}: size={boxes[i]}, origin={boxe_o[i]}, link={boxe_l[i]}")
    is_aligned = is_axis_aligned_rotation(boxe_o[i][:3, :3])
    print(f"  Is axis-aligned rotation: {is_aligned}")

    H_WL = urdf.get_transform(boxe_l[i], selected_as_world_link)
    H_LB = boxe_o[i]
    aabb = box_to_world_aabb(H_WL, H_LB, boxes[i])
    print(f"  World AABB: center={aabb['center']}, size={aabb['size']}")
    print("".center(50, "-"))

    aabbs["box_" + str(i)] = {
        "link": boxe_l[i],
        "center": aabb["center"].tolist(),
        "size": aabb["size"].tolist(),
    }


class ListFlowDumper(yaml.Dumper):
    def represent_sequence(self, tag, sequence, flow_style=None):
        # Force all sequences/lists into flow style []
        return super().represent_sequence(tag, sequence, flow_style=True)


def gen_static_collision():
    data = {}
    data["robot_name"] = robot_name
    data["link_names"] = link_names
    data["world_link"] = selected_as_world_link
    data["link_tf_in_world_link_xyz_wxyz"] = link_tf_in_world_link_xyz_wxyz
    data["collision_in_world_link"] = aabbs
    with open(f"{robot_name}_collision.yaml", "w") as f:
        yaml.dump(
            data,
            f,
            Dumper=ListFlowDumper,
            default_flow_style=False,
            sort_keys=False,
        )


gen_static_collision()
