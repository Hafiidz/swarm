#!/usr/bin/env python

import rospy
import time
import pandas as pd
from geometry_msgs.msg import Twist
from Turtle import *

import coord_helper as ch
import path_helper as ph
from math import pi


class Robot:
    def __init__(self):
        self.list = []

        # Initialize sketcher node
        rospy.init_node("sketcher", anonymous=False)

        # Publisher
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        # Subscriber
        # rospy.Subscriber('contours', segments, self.callback)

        # Rate to control frequency of operation of node
        self.rate = rospy.Rate(1)  # 10hz

        df = ch.coord_sorted_df("circle")
        self.contours = df[["x", "y", "d"]].to_numpy()
        self.numbers = df.shape[0]

        # self.numbers = 4
        # self.contours = [[1,1], [2,2], [3, 3], [4, 4]]

        # Reset the turtle-sim simulator
        reset_sim()

        time.sleep(2)  # Delay to make sure dynamic reconfigure is ready

    def spawn_source(self):
        """
        Spawn multiple turtles on the first point of each contour
        """
        rospy.loginfo("Spawning an army of turtles to perform drone show")
        for i in range(self.numbers):
            self.list.append(Turtle(i + 1))
            if i == 0:
                self.list[0].set_pen(0)
                self.list[0].teleport(
                    self.contours[i][0],
                    self.contours[i][1],
                    self.contours[i][2] + pi / 2,
                )
                self.list[0].set_pen(1)
            else:
                self.list[i].spawn(
                    self.contours[i][0],
                    self.contours[i][1],
                    self.contours[i][2] + pi / 2,
                )

    def log_pose(self):
        for i in range(self.numbers):
            rospy.loginfo(str(self.list[i].return_pose()))

    # TODO, still need work
    def move(self):
        # goal = ch.coord_sorted("square")
        goal = ch.coord_shifted(ch.coord_sorted("circle"), 2)
        n = min(goal.shape[0], self.numbers)
        for i in range(n):
            self.list[i].moveBezier(goal[i][0], goal[i][1])

    def move_parallel(self):
        # dfs = ch.gen_df_list()
        # path_plan_x, path_plan_y, path_plan_theta = ph.calc_path_from_df_list(dfs)

        path_plan_x = pd.read_csv(ch.p + "x.csv")
        path_plan_y = pd.read_csv(ch.p + "y.csv")
        path_plan_theta = pd.read_csv(ch.p + "t.csv")

        lj = path_plan_x.shape[1] - 1
        li = int(path_plan_x.shape[0] / 4)

        for j in range(lj):
            for i in range(li):
                self.list[i].teleport(
                    path_plan_x.iloc[i, j + 1],
                    path_plan_y.iloc[i, j + 1],
                    path_plan_theta.iloc[i, j + 1] + pi / 2,
                )
                self.list[i + li].teleport(
                    path_plan_x.iloc[i + li, j + 1],
                    path_plan_y.iloc[i + li, j + 1],
                    path_plan_theta.iloc[i + li, j + 1] + pi / 2,
                )
                self.list[i + 2 * li].teleport(
                    path_plan_x.iloc[i + 2 * li, j + 1],
                    path_plan_y.iloc[i + 2 * li, j + 1],
                    path_plan_theta.iloc[i + 2 * li, j + 1] + pi / 2,
                )
                self.list[i + 3 * li].teleport(
                    path_plan_x.iloc[i + 3 * li, j + 1],
                    path_plan_y.iloc[i + 3 * li, j + 1],
                    path_plan_theta.iloc[i + 3 * li, j + 1] + pi / 2,
                )


if __name__ == "__main__":
    try:
        r = Robot()
        r.spawn_source()
        # r.log_pose()
        r.move_parallel()

    except KeyboardInterrupt:
        exit()
