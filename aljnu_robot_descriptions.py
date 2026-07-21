import os
from os.path import join

dir_rsrc = os.environ["RSRC_DIR"]
dir_urdfs = os.path.join(dir_rsrc, "urdfs")

# URDF paths
__du = dir_urdfs  # shorthand
ALJNU_DESCRIPTIONS = {
    "ur5e": join(__du, "ur5e", "ur5e_extract_calibrated.urdf"),
    "ur5e_sph": join(__du, "ur5e", "ur5e_extract_calibrated_spherized.urdf"),
    "three_shelf": join(__du, "three_shelf.urdf"),
    "single_stool": join(__du, "single_stool.urdf"),
    "single_bar_strict": join(__du, "single_bar_strict.urdf"),
    "shelf": join(__du, "shelf.urdf"),
    "airbus_shopfloor": join(__du, "airbus_shopfloor.urdf"),
    "simple_box": join(__du, "simple_box.urdf"),
    "plane": join(__du, "plane.urdf"),
    "three_planar_board": join(__du, "three_planar_board.urdf"),
    "stool_shelf": join(__du, "stool_shelf.urdf"),
}

# Collision YAML paths
# Simple StaticAABB collision defined in robot base link,
# Easy for vectorized collision checking
ALJNU_AABB_COLLISION_SHEET = {
    "airbus_shopfloor": join(__du, "airbus_shopfloor_collision.yaml"),
    "three_shelf": join(__du, "three_shelf_collision.yaml"),
    "single_stool": join(__du, "single_stool_collision.yaml"),
    "single_bar_strict": join(__du, "single_bar_strict_collision.yaml"),
    "three_planar_board": join(__du, "three_planar_board_collision.yaml"),
    "stool_shelf": join(__du, "stool_shelf_collision.yaml"),
}


def _check_existence(DICT):
    for key, value in DICT.items():
        if not os.path.exists(value):
            print(f"URDF file {key} does not exist at path: {value}")
        else:
            print(f"URDF file {key} exists at path: {value}")
    print(f"All files in exist.")


if __name__ == "__main__":
    _check_existence(ALJNU_DESCRIPTIONS)
    _check_existence(ALJNU_AABB_COLLISION_SHEET)

    # terminal interactive selection of URDF file
    from pick import pick

    title = "Choose a robot URDF to generate static collision data: "
    urdf_options = list(ALJNU_DESCRIPTIONS.keys())
    urdf_options.append("Exit")
    urdf_name, index = pick(urdf_options, title)
    if urdf_name == "Exit":
        print("Exiting...")
        exit(0)
    urdf_path = ALJNU_DESCRIPTIONS[urdf_name]
    print(f"==>> urdf_path: \n{urdf_path}")
