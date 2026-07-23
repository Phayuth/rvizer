import numpy as np
import yaml


def yaml_write(data, filename):
    with open(filename, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def yaml_read(fyaml):
    with open(fyaml, "r") as f:
        data = yaml.safe_load(f)
    return data
