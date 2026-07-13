import numpy as np
import yaml


def gen_color_interp(n=12, s="#1e40af", e="#f97316", m="hex"):
    # LAB (CIE L*a*b*): Perceptually uniform

    def _hex_to_rgb01(hex_color):
        hex_color = hex_color.lstrip("#")
        return (
            np.array(
                [int(hex_color[i : i + 2], 16) for i in (0, 2, 4)], dtype=float
            )
            / 255.0
        )

    def _srgb_to_linear(rgb):
        return np.where(
            rgb <= 0.04045,
            rgb / 12.92,
            ((rgb + 0.055) / 1.055) ** 2.4,
        )

    def _linear_to_srgb(rgb):
        return np.where(
            rgb <= 0.0031308,
            12.92 * rgb,
            1.055 * np.power(np.clip(rgb, 0.0, None), 1 / 2.4) - 0.055,
        )

    def _rgb_to_xyz(rgb):
        rgb = _srgb_to_linear(rgb)
        return np.array(
            [
                0.4124564 * rgb[0] + 0.3575761 * rgb[1] + 0.1804375 * rgb[2],
                0.2126729 * rgb[0] + 0.7151522 * rgb[1] + 0.0721750 * rgb[2],
                0.0193339 * rgb[0] + 0.1191920 * rgb[1] + 0.9503041 * rgb[2],
            ]
        )

    def _xyz_to_rgb(xyz):
        rgb_linear = np.array(
            [
                3.2404542 * xyz[0] - 1.5371385 * xyz[1] - 0.4985314 * xyz[2],
                -0.9692660 * xyz[0] + 1.8760108 * xyz[1] + 0.0415560 * xyz[2],
                0.0556434 * xyz[0] - 0.2040259 * xyz[1] + 1.0572252 * xyz[2],
            ]
        )
        return np.clip(_linear_to_srgb(rgb_linear), 0.0, 1.0)

    def _xyz_to_lab(xyz):
        white = np.array([0.95047, 1.0, 1.08883])  # D65
        xyz = xyz / white

        def _f(t):
            delta = 6 / 29
            return np.where(t > delta**3, np.cbrt(t), t / (3 * delta**2) + 4 / 29)

        fx, fy, fz = _f(xyz)
        l = 116 * fy - 16
        a = 500 * (fx - fy)
        b = 200 * (fy - fz)
        return np.array([l, a, b])

    def _lab_to_xyz(lab):
        white = np.array([0.95047, 1.0, 1.08883])  # D65
        l, a, b = lab
        fy = (l + 16) / 116
        fx = fy + a / 500
        fz = fy - b / 200

        def _f_inv(t):
            delta = 6 / 29
            return np.where(t > delta, t**3, 3 * delta**2 * (t - 4 / 29))

        x = _f_inv(fx)
        y = _f_inv(fy)
        z = _f_inv(fz)
        return np.array([x, y, z]) * white

    def _rgb_to_lab(rgb):
        return _xyz_to_lab(_rgb_to_xyz(rgb))

    def _lab_to_hex(lab):
        rgb = _xyz_to_rgb(_lab_to_xyz(lab))
        rgb_255 = np.round(rgb * 255).astype(int)
        return "#{:02x}{:02x}{:02x}".format(*rgb_255)

    start_lab = _rgb_to_lab(_hex_to_rgb01(s))
    end_lab = _rgb_to_lab(_hex_to_rgb01(e))
    t = np.linspace(0.0, 1.0, n)
    lab_interp = np.outer(1 - t, start_lab) + np.outer(t, end_lab)
    hex_interp = [_lab_to_hex(lab) for lab in lab_interp]

    if m == "hex":
        return hex_interp
    elif m == "rgb01":
        return [_hex_to_rgb01(hex_color) for hex_color in hex_interp]
    elif m == "rgb255":
        return [
            tuple(int(c * 255) for c in _hex_to_rgb01(hex_color))
            for hex_color in hex_interp
        ]


def load_trajectory(path):
    with open(path, "r") as yaml_file:
        dict = yaml.safe_load(yaml_file)
        joint_names = dict["joint_names"]
        points = np.array(dict["points"])
        time_from_start = np.array(dict["time_from_start"])
        traj = {
            "joint_names": joint_names,
            "N": points.shape[0],
            "points": points,
            "time_from_start": time_from_start,
            "dof": points.shape[1],
        }
        return traj


def load_taskspace(path):
    with open(path, "r") as yaml_file:
        dict = yaml.safe_load(yaml_file)
        standard = dict["standard"]
        points = np.array(dict["points"])
        taskspace = {
            "standard": standard,
            "N": points.shape[0],
            "points": points,
        }
        return taskspace


def load_taskspace_tour(path):
    with open(path, "r") as yaml_file:
        dict = yaml.safe_load(yaml_file)
        standard = dict["standard"]
        order = dict["order"]
        is_points_ordered = dict["is_points_ordered"]
        points = np.array(dict["points"])
        N = points.shape[0]
        taskspace_tour = {
            "standard": standard,
            "N": N,
            "points": points,
            "order": order,
            "is_points_ordered": is_points_ordered,
        }
        return taskspace_tour


if __name__ == "__main__":
    colors = gen_color_interp(n=12)
    print(colors)

    rgbbb = gen_color_interp(n=12, m="rgb255")
    print(rgbbb)
