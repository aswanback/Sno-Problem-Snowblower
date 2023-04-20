#!/usr/bin/env python3

import sys
import os
import numpy as np
import time

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from a_star import AStar, MapProcessor
from visualization_msgs.msg import MarkerArray, Marker


class Navigation:
    def __init__(self, node_name='Navigation'):
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=10)
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Create path planning variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
    

    def get_path_idx(self, path, vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  vehicle_pose          PoseStamped object containing the current vehicle position.
        @return idx                   Position int the path pointing to the next goal pose to follow.
        """
        idx = self.currIdx
        vp = vehicle_pose.pose.position
        min_dist = float('inf')
        # loop through each point in the path starting with the current
        for i, item in enumerate(path.poses):
            p = item.pose.position
            dist = ((p.x - vp.x) ** 2 + (p.y - vp.y) ** 2 + (p.z - vp.z) ** 2)
            # find the closest point index, return the following one
            if dist <= min_dist:
                min_dist = dist
                idx = i
        # make sure return is the goal pose index at max
        return min(idx+1, len(path.poses) - 1)

    def path_follower(self, vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return speed, heading
        """

        # set speed and absolute heading
        speed = 0.1
        vp = vehicle_pose.pose.position
        c = current_goal_pose.pose.position
        heading = np.arctan2(c.y - vp.y, c.x - vp.x)

        # Stop moving if at goal
        g = self.goal_pose.pose.position
        dist = np.sqrt((g.x - vp.x) ** 2 + (g.y - vp.y) ** 2)
        if dist < 0.075:
            rospy.loginfo('At goal, waiting')
            speed = 0
            # adjust to goal heading
            q = self.goal_pose.pose.orientation
            heading = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # rospy.loginfo(f's:{speed:.3f}\th:{heading:.3f}')
        return speed, heading

    def move_ttbot(self, speed, heading):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired yaw angle.
        @param  heading   Desired speed.
        @return none
        """
        # find current heading
        q = self.ttbot_pose.pose.orientation
        curr_angle = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # find angle difference between desired and current
        ang = heading - curr_angle
        ang = (ang + np.pi) % (2 * np.pi) - np.pi

        # create Twist message with linear speed only if within angle tolerance
        cmd_vel = Twist()
        if abs(ang) < 15/180*np.pi:
            cmd_vel.linear.x = speed
            cmd_vel.angular.z = ang * 3
        else:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = ang * 1.2

        # publish
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        """
        while not rospy.is_shutdown():
            # if the goal_pose has changed, replan A*
            if self.goal_pose != old_goal_pose:
                path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
            old_goal_pose = self.goal_pose
            # if no path is returned, stop moving
            if path is None:
                self.move_ttbot(0, 0)
                continue
            # find next point to go towards
            idx = self.get_path_idx(path, self.ttbot_pose)
            self.currIdx = idx
            current_goal = path.poses[idx]
            # decide speed and heading
            speed, heading = self.path_follower(self.ttbot_pose, current_goal)
            # move with speed and heading
            self.move_ttbot(speed, heading)