from numpy import exp, abs, angle
import pandas as pd

# p = "/root/catkin_ws/src/swarm/scripts/coordinates/"
# p = "~/ros/catkin_ws/src/swarm/scripts/coordinates/"
p = "~/catkin_ws/src/swarm/scripts/coordinates/"


def shape(s, target_space=3, target_length=5, path=p):
    # coordinate is 11x11, normalize to ensure, 3 spaces to the side and 5 at center

    df = pd.read_csv("{}{}.csv".format(p, s))

    df["x"] = df["x"] - df["x"].min()
    df["y"] = df["y"] - df["y"].min()

    l_x = abs(df["x"].max() - df["x"].min())
    l_y = abs(df["y"].max() - df["y"].min())

    length = max(l_x, l_y)

    arr = df.to_numpy()
    coord = (arr / length * target_length) + target_space
    return coord.round(decimals=4)


def polar2z(r, theta):
    return r * exp(1j * theta)


def z2polar(z):
    return (abs(z), angle(z))


def xy2polar(x, y):
    z = x + 1j * y
    return z2polar(z)


def sort_closest(df):
    # TODO, special sorting
    # find the furthest point

    # then find the next closest point

    return df


def coord_sorted(s, sort="none", path=p):
    sort = sort.lower()
    goal = shape(s)
    goal -= 5.5

    df = pd.DataFrame(goal, columns=["x", "y"])
    df["r"], df["d"] = xy2polar(df.x, df.y)

    if sort == "polar":
        df.sort_values(by="d", inplace=True)
    elif sort == "none" or sort == "no" or sort == "n":
        df = df  # don't change sorting
    # if sort == "closest":
    #     df = sort_closest(df)
    else:
        df.sort_values(by=sort, inplace=True)

    return df[["x", "y"]].to_numpy() + 5.5


def coord_shifted(coord, shift=1):
    df = pd.DataFrame(coord, columns=["x", "y"])
    l = df.index.to_list()
    idx = l[-shift:] + l[0:-shift]
    df = df.iloc[idx]
    return df[["x", "y"]].to_numpy()


def shape_df(s, target_space=3, target_length=5, path=p):
    # coordinate is 11x11, normalize to ensure, 3 spaces to the side and 5 at center

    df = pd.read_csv("{}{}.csv".format(p, s))

    df["x"] = df["x"] - df["x"].min()
    df["y"] = df["y"] - df["y"].min()

    l_x = abs(df["x"].max() - df["x"].min())
    l_y = abs(df["y"].max() - df["y"].min())

    length = max(l_x, l_y)

    df = (df / length * target_length) + target_space
    return df.round(decimals=4)


def coord_sorted_df(
    s, sort="none", offset=-5.5, target_space=3, target_length=5, path=p
):
    goal = shape_df(s, target_space, target_length)

    df = pd.DataFrame(goal, columns=["x", "y"])
    df["r"], df["d"] = xy2polar(df.x + offset, df.y + offset)

    if sort == "polar":
        df.sort_values(by="d", inplace=True)
    elif sort == "none" or sort == "no" or sort == "n":
        df = df  # don't change sorting
    # if sort == "closest":
    #     df = sort_closest(df)
    else:
        df.sort_values(by=sort, inplace=True)

    return df


def coord_shifted_df(df, shift=1):
    l = df.index.to_list()
    idx = l[-shift:] + l[0:-shift]
    df = df.iloc[idx]
    return df


def gen_df_list():
    df1 = coord_sorted_df("square", sort="polar")
    df2 = coord_sorted_df("circle", sort="polar")
    df3 = coord_sorted_df("star", sort="polar")
    df4 = coord_sorted_df("circle", sort="polar")
    df5 = coord_sorted_df("square", sort="polar")
    df6 = coord_sorted_df("diamond", sort="polar")
    df7 = coord_sorted_df("d5", sort="polar")
    return [df1, df2, df3, df4, df5, df6, df7]


def gen_df_breakdown():
    df1 = coord_sorted_df("square", sort="polar")
    df2 = coord_sorted_df("circle", sort="polar")
    df2 = coord_sorted_df("circle_32", sort="polar", target_length=9, target_space=1)
    return [df1, df2]
