import numpy as np


def airbus_shopfloor_taskspace_points():
    # airbus_shopfloor
    # dont forget to offset z by 0.15 stool
    x = 0.6
    y = np.linspace(-0.35, 0.35, 15)
    z = np.linspace(0.2, 0.9, 20)
    position_array = np.meshgrid(x, y, z)
    position_array = np.stack(position_array, axis=-1).reshape(-1, 3)
    xyzw = np.array([0.5000, -0.5000, 0.5000, -0.5000])
    xyzw = xyzw / np.linalg.norm(xyzw)
    wxyz_array = np.tile(xyzw, (position_array.shape[0], 1))
    return position_array, wxyz_array


def cube_surface_grid(side_length, spacing):
    hx = side_length / 2
    hy = side_length / 2
    hz = side_length / 2
    xs = np.arange(-hx, hx + spacing * 0.5, spacing)
    ys = np.arange(-hy, hy + spacing * 0.5, spacing)
    zs = np.arange(-hz, hz + spacing * 0.5, spacing)

    pts = []

    # x = ±h
    Y, Z = np.meshgrid(ys, zs, indexing="ij")
    pts.append(np.column_stack([np.full(Y.size, -hx), Y.ravel(), Z.ravel()]))
    pts.append(np.column_stack([np.full(Y.size, +hx), Y.ravel(), Z.ravel()]))

    # y = ±h
    X, Z = np.meshgrid(xs, zs, indexing="ij")
    pts.append(np.column_stack([X.ravel(), np.full(X.size, -hy), Z.ravel()]))
    pts.append(np.column_stack([X.ravel(), np.full(X.size, +hy), Z.ravel()]))

    # z = ±h
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    pts.append(np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, -hz)]))
    pts.append(np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, +hz)]))

    pts = np.vstack(pts)

    # Remove duplicates on edges/corners
    pts = np.unique(np.round(pts, 12), axis=0)

    return pts


def single_stool_taskspace_points():
    # single_stool
    position_array = cube_surface_grid(side_length=1.0, spacing=0.25)
    wxyz_array = np.tile(np.array([1, 0, 0, 0]), (position_array.shape[0], 1))
    return position_array, wxyz_array


p, q = single_stool_taskspace_points()
