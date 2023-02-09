#!/usr/bin/env python

import rospy
from turtlesim.srv import *
from std_srvs.srv import Empty

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

from nav_msgs.msg import Odometry
import path_helper as ph


def reset_sim():
    """
    Function to Reset the Simulator
    """
    try:
        reset_serv = rospy.ServiceProxy("/reset", Empty)
        reset_serv()
    except rospy.ServiceException as e:
        rospy.loginfo("Service execution failed: %s" + str(e))


def callback(msg):
    return msg


class Turtle:
    def __init__(self, i):
        self.name = "turtle" + str(i)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(
            "/{}/cmd_vel".format(self.name), Twist, queue_size=10
        )

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber(
            "/{}/pose".format(self.name), Pose, self.update_pose
        )

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def __str__(self):
        return "Turtle {}".format(self.name)

    def __repr__(self):
        return "Turtle {}: pose at {}".format(self.name, self.pose)

    def spawn(self, x, y, theta):
        """
        Function to spawn turtles in the Turtle-sim
        :param x: x-position with respect to origin at bottom-left
        :type x: float
        :param y: y-position with respect to origin at bottom-left
        :type y: float
        :param theta: orientation with respect to x-axis
        :type theta: float between [0 to 3] OR [0 to -3]
        """
        try:
            serv = rospy.ServiceProxy("/spawn", Spawn)
            serv(x, y, theta, self.name)
            self.pose.x = round(x, 4)
            self.pose.y = round(y, 4)
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))

    def set_pen(self, flag=True):
        """
        Function to sketch the turtle movements
        :param flag: To turn sketching pen - ON[True]/OFF[False]
        :type flag: bool
        """
        try:
            if not flag:
                set_serv = rospy.ServiceProxy("/" + self.name + "/set_pen", SetPen)
                set_serv(0, 0, 0, 0, 1)
            elif flag:
                set_serv = rospy.ServiceProxy("/" + self.name + "/set_pen", SetPen)
                set_serv(255, 255, 255, 2, 0)
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))

    def teleport(self, x, y, theta):
        """
        Function to teleport the turtle
        :param x: x-position with respect to origin at bottom-left
        :type x: float
        :param y: y-position with respect to origin at bottom-left
        :type y: float
        :param theta: orientation with respect to x-axis
        :type theta: float between [0 to 3] OR [0 to -3]
        """
        try:
            serv = rospy.ServiceProxy(
                "/" + self.name + "/teleport_absolute", TeleportAbsolute
            )
            serv(x, y, theta)
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))

    def kill_turtle(self):
        """
        Function to remove the turtle from Turtle-sim
        """
        try:
            serv = rospy.ServiceProxy("/kill", Kill)
            serv(self.name)
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))

    """"""
    """ Auxiliary Functions """
    """"""

    def return_pose(self):
        return (
            self.pose.x,
            self.pose.y,
            self.pose.theta,
            self.pose.linear_velocity,
            self.pose.angular_velocity,
        )

    """"""
    """ Movement Functions """
    """"""

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(
            pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)
        )

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def moveBezier(self, x, y):
        path_, control_point = ph.calc_4points_bezier_path(
            self.pose.x, self.pose.y, self.pose.theta, x, y, 0, 1
        )
        for i in range(path_.shape[0]):
            self.teleport(path_[i][0], path_[i][1], self.pose.theta)

    def move2goal(self, x, y):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = float(x)
        goal_pose.y = float(y)

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = float(0.01)

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


if __name__ == "__main__":
    try:
        # Create a turtle
        turtle2 = Turtle(2)

        # Spawn the turtle at 3, 3, 0
        turtle2.spawn(3, 3, 0)

        # Teleport the turtle to 3, 9, 0
        turtle2.teleport(3, 9, 0)

        # Stop sketching turtle movement
        turtle2.set_pen(False)

        # Teleport the turtle to 5, 9, 0
        turtle2.teleport(5, 9, 0)

        # Start sketching turtle movement
        turtle2.set_pen(True)

        # Teleport the turtle to 9, 9, 0
        turtle2.teleport(9, 9, 0)
    except KeyboardInterrupt:
        exit()
