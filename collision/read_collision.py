import yaml
import numpy as np

with open("airbus_shopfloor_collision.yaml", "r") as f:
    data = yaml.safe_load(f)

n = len(data["collision_in_world_link"])
box_in_base = np.zeros((n, 4, 4))
boxsz_in_base = np.zeros((n, 3))

collision_in_world_link = data["collision_in_world_link"]
for i, (box_name, box_data) in enumerate(collision_in_world_link.items()):
    link_name = box_data["link"]
    center = np.array(box_data["center"])
    size = np.array(box_data["size"])
    print(f"Box {box_name}: link={link_name}, center={center}, size={size}")

    H = np.eye(4)
    H[:3, 3] = center
    box_in_base[i] = H
    boxsz_in_base[i] = size


print(box_in_base)
print(boxsz_in_base)
