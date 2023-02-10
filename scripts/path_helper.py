from bezier_path import *


def calc_bezier_path_n_points(sx, sy, syaw, ex, ey, eyaw, n_points=10, offset=1):
    """
    Compute control points and path given start and end position.

    :param sx: (float) x-coordinate of the starting point
    :param sy: (float) y-coordinate of the starting point
    :param syaw: (float) yaw angle at start
    :param ex: (float) x-coordinate of the ending point
    :param ey: (float) y-coordinate of the ending point
    :param eyaw: (float) yaw angle at the end
    :param offset: (float)
    :return: (numpy array, numpy array)
    """
    dist = np.hypot(sx - ex, sy - ey) / offset
    control_points = np.array(
        [
            [sx, sy],
            [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
            [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
            [ex, ey],
        ]
    )

    return calc_bezier_path(control_points, n_points)


def calc_path_from_df_list(df_list):
    l = []
    for d in df_list:
        l.append(d.shape[0])

    length = min(l)

    path_x = pd.DataFrame(range(1, length + 1), columns=["turtle"])
    path_y = pd.DataFrame(range(1, length + 1), columns=["turtle"])

    for i in range(length):
        for j in range(len(df_list) - 1):
            current = df_list[j].iloc[i]
            target = df_list[j + 1].iloc[i]

            p2 = calc_bezier_path_n_points(
                current.x, current.y, current.d, target.x, target.y, target.d
            )

            for k in range(len(p2)):
                path_x.at[i, str(j) + "_" + str(k)] = p2[k][0]
                path_y.at[i, str(j) + "_" + str(k)] = p2[k][1]

    return path_x, path_y
