#!/usr/bin/env python

import rospy
import time
import pandas as pd
from geometry_msgs.msg import Twist
from Turtle import *

import coord_helper as ch
import path_helper as ph


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

        norm_arr = ch.coord_sorted("circle")
        self.contours = norm_arr
        self.numbers = norm_arr.shape[0]

        # self.numbers = 4
        # self.contours = [[1,1], [2,2], [3, 3], [4, 4]]

        # Reset the turtle-sim simulator
        reset_sim()

        time.sleep(2)  # Delay to make sure dynamic reconfigure is ready

    def spawn_source(self):
        """
        Spawn multiple turtles on the first point of each contour
        """
        rospy.loginfo("Spawning an army of turtles to sketch your image")
        for i in range(self.numbers):
            self.list.append(Turtle(i + 1))
            if i == 0:
                self.list[0].set_pen(0)
                self.list[0].teleport(self.contours[i][0], self.contours[i][1], 0.0)
                self.list[0].set_pen(1)
            else:
                self.list[i].spawn(self.contours[i][0], self.contours[i][1], 0.0)

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
        dfs = ch.gen_df_list()
        path_plan_x, path_plan_y = ph.calc_path_from_df_list(dfs)

        lj = path_plan_x.shape[1] - 1
        li = int(path_plan_x.shape[0] / 4)

        for j in range(lj):
            for i in range(li):
                self.list[i].teleport(
                    path_plan_x.iloc[i, j + 1], path_plan_y.iloc[i, j + 1], 0
                )
                self.list[i + li].teleport(
                    path_plan_x.iloc[i + li, j + 1], path_plan_y.iloc[i + li, j + 1], 0
                )
                self.list[i + 2 * li].teleport(
                    path_plan_x.iloc[i + 2 * li, j + 1],
                    path_plan_y.iloc[i + 2 * li, j + 1],
                    0,
                )
                self.list[i + 3 * li].teleport(
                    path_plan_x.iloc[i + 3 * li, j + 1],
                    path_plan_y.iloc[i + 3 * li, j + 1],
                    0,
                )


if __name__ == "__main__":
    try:
        r = Robot()
        r.spawn_source()
        r.log_pose()
        r.move_parallel()

    except KeyboardInterrupt:
        exit()
