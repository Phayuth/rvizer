import os
from os.path import join

dir_rsrc = os.environ["RSRC_DIR"]
dir_urdfs = os.path.join(dir_rsrc, "urdfs")

# URDF paths
_du = dir_urdfs  # shorthand


class ALJNU_DESCRIPTIONS:

    ROBOTS = {
        "ur5e": join(_du, "ur5e", "ur5e_extract_calibrated.urdf"),
        "ur5e_sph": join(_du, "ur5e", "ur5e_extract_calibrated_spherized.urdf"),
        "rb3_730es_u": join(_du, "rb3_730es_u", "rb3_730es_u.urdf"),
        "aljnu_humanoid": join(_du, "aljnu_humanoid", "aljnu_humanoid.urdf"),
        "robotiq_85": join(_du, "robotiq2f", "robotiq_85.urdf"),
        "robotiq_hande": join(_du, "robotiq2f", "robotiq_hande.urdf"),
        "doosan_m0609_white": join(_du, "doosan_m0609", "m0609_white.urdf"),
        "doosan_m1509_white": join(_du, "doosan_m1509", "m1509_white.urdf"),
    }

    ENVS = {
        "airbus_shopfloor": join(_du, "airbus_shopfloor.urdf"),
        "three_shelf": join(_du, "three_shelf.urdf"),
        "single_stool": join(_du, "single_stool.urdf"),
        "single_bar_strict": join(_du, "single_bar_strict.urdf"),
        "three_planar_board": join(_du, "three_planar_board.urdf"),
        "stool_shelf": join(_du, "stool_shelf.urdf"),
        "inspect_stool": join(_du, "inspect_stool.urdf"),
        "shelf": join(_du, "shelf.urdf"),
        "simple_box": join(_du, "simple_box.urdf"),
        "plane": join(_du, "plane.urdf"),
    }

    """
    Collision YAML paths
    Simple Static AABB collision defined in robot base link,
    Easy for vectorized collision checking
    """
    ENVS_AABB_COL_SHEET = {
        "airbus_shopfloor": join(_du, "airbus_shopfloor_collision.yaml"),
        "three_shelf": join(_du, "three_shelf_collision.yaml"),
        "single_stool": join(_du, "single_stool_collision.yaml"),
        "single_bar_strict": join(_du, "single_bar_strict_collision.yaml"),
        "three_planar_board": join(_du, "three_planar_board_collision.yaml"),
        "stool_shelf": join(_du, "stool_shelf_collision.yaml"),
    }


def _check_existence(DICT):
    for key, value in DICT.items():
        if not os.path.exists(value):
            print(f"URDF file {key} does not exist at path: {value}")
        else:
            print(f"URDF file {key} exists at path: {value}")
    print(f"All files in exist.")


if __name__ == "__main__":
    _check_existence(ALJNU_DESCRIPTIONS.ROBOTS)
    _check_existence(ALJNU_DESCRIPTIONS.ENVS_AABB_COL_SHEET)

    # terminal interactive selection of URDF file
    from pick import pick

    title = "Choose a robot URDF to generate static collision data: "
    urdf_options = list(ALJNU_DESCRIPTIONS.ROBOTS.keys())
    urdf_options.append("Exit")
    urdf_name, index = pick(urdf_options, title)
    if urdf_name == "Exit":
        print("Exiting...")
        exit(0)
    urdf_path = ALJNU_DESCRIPTIONS.ROBOTS[urdf_name]
    print(f"==>> urdf_path: \n{urdf_path}")
