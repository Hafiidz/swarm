from numpy import exp, abs, angle
import pandas as pd
import numpy as np

p = "/root/catkin_ws/src/swarm/scripts/"
# p = "~/catkin_ws/src/swarm/scripts/"


def shape(s):
    df = pd.read_csv("{}{}.csv".format(p, s))

    # coordinate is 11x11, normalize to ensure, 3 spaces to the side and 5 at center
    target_space = 3
    target_length = 5

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


def coord_sorted(s, sort="polar"):
    goal = shape(s)
    goal -= 5.5

    df = pd.DataFrame(goal, columns=["x", "y"])
    df["r"], df["d"] = xy2polar(df.x, df.y)

    if sort == "polar":
        df.sort_values(by="d", inplace=True)
    else:
        df.sort_values(by=sort, inplace=True)

    return df[["x", "y"]].to_numpy() + 5.5


def coord_shifted(coord, shift=1):
    df = pd.DataFrame(coord, columns=["x", "y"])
    l = df.index.to_list()
    idx = l[-shift:] + l[0:-shift]
    df = df.iloc[idx]
    return df[["x", "y"]].to_numpy()


def shape_df(s):
    df = pd.read_csv("{}{}.csv".format(p, s))

    # coordinate is 11x11, normalize to ensure, 3 spaces to the side and 5 at center
    target_space = 3
    target_length = 5

    df["x"] = df["x"] - df["x"].min()
    df["y"] = df["y"] - df["y"].min()

    l_x = abs(df["x"].max() - df["x"].min())
    l_y = abs(df["y"].max() - df["y"].min())

    length = max(l_x, l_y)

    df = (df / length * target_length) + target_space
    return df.round(decimals=4)


def coord_sorted_df(s, sort="polar", offset=-5.5):
    goal = shape_df(s)

    df = pd.DataFrame(goal, columns=["x", "y"])
    df["r"], df["d"] = xy2polar(df.x + offset, df.y + offset)

    if sort == "polar":
        df.sort_values(by="d", inplace=True)
    else:
        df.sort_values(by=sort, inplace=True)

    return df


def coord_shifted_df(df, shift=1):
    l = df.index.to_list()
    idx = l[-shift:] + l[0:-shift]
    df = df.iloc[idx]
    return df


def gen_df_list():
    df1 = coord_sorted_df("square")
    df2 = coord_sorted_df("circle")
    df3 = coord_shifted_df(df2)
    df4 = coord_sorted_df("square")
    df5 = coord_shifted_df(df4, 10)

    return [df1, df2, df3, df4, df5]
