import os

dir_rsrc = os.environ["RSRC_DIR"]
dir_urdfs = os.path.join(dir_rsrc, "urdfs")

# URDF paths
ALJNU_DESCRIPTIONS = {
    "ur5e": os.path.join(dir_urdfs, "ur5e", "ur5e_extract_calibrated.urdf"),
    "ur5e_sph": os.path.join(
        dir_urdfs, "ur5e", "ur5e_extract_calibrated_spherized.urdf"
    ),
    "three_shelf": os.path.join(dir_urdfs, "three_shelf.urdf"),
    "single_stool": os.path.join(dir_urdfs, "single_stool.urdf"),
    "single_bar_strict": os.path.join(dir_urdfs, "single_bar_strict.urdf"),
    "shelf": os.path.join(dir_urdfs, "shelf.urdf"),
    "airbus_shopfloor": os.path.join(dir_urdfs, "airbus_shopfloor.urdf"),
    "simple_box": os.path.join(dir_urdfs, "simple_box.urdf"),
    "plane": os.path.join(dir_urdfs, "plane.urdf"),
    "three_planar_board": os.path.join(dir_urdfs, "three_planar_board.urdf"),
    "stool_shelf": os.path.join(dir_urdfs, "stool_shelf.urdf"),
}

# Collision YAML paths
# Simple StaticAABB collision defined in robot base link,
# Easy for vectorized collision checking
ALJNU_AABB_COLLISION_SHEET = {
    "airbus_shopfloor": os.path.join(dir_urdfs, "airbus_shopfloor_collision.yaml"),
}
