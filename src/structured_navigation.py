#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Jupiter Robot Technology Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Dante Huang

import rospy
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

original = 0
start = 0

class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server.")

        # A variable to hold the initial pose of the robot to be set by the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
           rospy.sleep(1)

        rospy.loginfo("Starting navigation node...")
        rospy.sleep(1)

        # List of locations
        self.locations = dict()
        self.locations['EmptySeat'] = [2.7257, 0.3012, 0.08340, 0.99652]
        self.locations['PinkBox'] = [3.5262, 2.0491, 0.5673, 0.8235]
        # Shelf coordimate needs to modify
        self.locations['Shelf'] = [3.5262, 2.0491, 0.5673, 0.8235]

        # Subscribe to get target location
        self.target_location = rospy.Subscriber("nav_cmd", String, self.nav_callback)
        self.nav_pub = rospy.Publisher("nav_feedback", String, queue_size=1)

        # --------------------------------------------------------------------------

        # Get initial location
        # quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        # self.origin = Pose(Point(0, 0, 0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        # --------------------------------------------------------------------------

    def nav_callback(self, target_location):
        self.goal = MoveBaseGoal()
        rospy.loginfo("Ready to go.")
        global start

        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # Robot will go to the target location
        if target_location in self.locations.keys():
            coordinate = self.locations[target_location]
            destination = Pose(Point(coordinate[0], coordinate[1], 0.000), Quaternion(0.0, 0.0, coordinate[2], coordinate[3]))
            rospy.loginfo("Going to point A")
            rospy.sleep(2)
            self.goal.target_pose.pose = destination
            self.move_base.send_goal(self.goal)
            waiting = self.move_base.wait_for_result(rospy.Duration(300))
            if waiting == 1:
                rospy.loginfo("Reached point A")
                self.nav_pub.publish("Done")
                rospy.sleep(2)
                start = 0
        else: 
            rospy.loginfo("Invalid Location")

        rospy.Rate(5).sleep()


    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
        global original
        if original == 0:
            self.origin = self.initial_pose.pose.pose
            original = 1

    def cleanup(self):
        rospy.loginfo("Shutting down navigation...")
        self.move_base.cancel_goal()

if __name__=="__main__":
    rospy.init_node('navi_point')
    try:
        NavToPoint()
        rospy.spin()
    except:
        pass
