#!/usr/bin/env python

'''
	This file is the whole patrol process navigation file
	------------------------------------------------------------------
	To run this patrol process file only, you need to run the lines below in command prompt:
	- roslaunch jupiterobot_bringup jupiterobot_bringup.launch
	- roslaunch jupiterobot_navigation rplidar_amcl_demo.launch map_file:=/home/mustar/catkin_ws/src/lab_demo/maps/lab_new_map.yaml
	- roslaunch turtlebot_rviz_launchers view_navigation.launch
	- rosrun lab_demo navigation.py
'''

import rospy
from gtts import gTTS
import os
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

from std_msgs.msg import String
from math import radians


original = 0

class officeTour:
	def __init__(self):
		rospy.on_shutdown(self.cleanup)
		
		self.task_exec = False
		self.human_detected = False
		self.description = ""
		# ================== INITIALIZATION ================== 
		# Subscribe to the move_base action server
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		rospy.loginfo("Waiting for move_base action server...")

		# Wait for the action server to become available
		self.move_base.wait_for_server(rospy.Duration(120))
		rospy.loginfo("Connected to move base server")

		# A variable to hold the initial pose of the robot to be set by the user in RViz
		initial_pose = PoseWithCovarianceStamped()
		rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
		# rospy.Subscriber('/move_base/result',MoveBaseActionResult,self.task_exec_callback)
		
		rospy.Subscriber('task_status', String, self.task_status_callback)

		self.sub = rospy.Subscriber('final_result', String, self.handle_payload)

		# Get the initial pose from the user
		rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
		rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

		# Make sure we have the initial pose
		while initial_pose.header.stamp == "":
			rospy.sleep(1)
		# =====================================================
			
		rospy.loginfo("Ready to go")
		rospy.sleep(1)

		# Coordinates
		# run amcl, rostopic echo /amcl_pose 
		locations = dict()
		
		coordinate1 = [2.725710883866503, 0.3011662559300836, 0.08340386282135936, 0.9965158281063456]

		# coordinate1 = [0.725710883866503, 0.3011662559300836, 0.08340386282135936, 0.9965158281063456]
		destination1 = Pose(Point(coordinate1[0], coordinate1[1], 0.000), Quaternion(0.0, 0.0, coordinate1[2], coordinate1[3]))
		
		coordinate2 = [3.5261334019702244, 2.049094052127344, 0.5673293615017563, 0.823490980872292]
		destination2 = Pose(Point(coordinate2[0], coordinate2[1], 0.000), Quaternion(0.0, 0.0, coordinate2[2], coordinate2[3]))
		
		# coordinate['whiteboard'] = [6.2163, -2.5276, 0.9871, 0.1598]

		# Start navigate to destination flow 
		self.goal = MoveBaseGoal()
		rospy.loginfo("Start Navigation")
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.header.stamp = rospy.Time.now()
		rospy.sleep(2)

		for i in range(2):
			# Pink Box
			self.goal.target_pose.pose = destination1
			self.move_base.send_goal(self.goal)
			waiting = self.move_base.wait_for_result(rospy.Duration(300))
			if waiting == 1:
				rospy.loginfo("Reached Pink Box")
				rospy.sleep(2)
			self.task_exec = True

			rospy.loginfo("Doing task ---- Waiting for people")

			while self.task_exec and not rospy.is_shutdown():
				rospy.loginfo("Waiting for task to complete...")
				rospy.sleep(10)		

			rospy.loginfo("Task completed")
			
			rospy.sleep(5)

			# Empty Seat
			self.goal.target_pose.pose = destination2
			self.move_base.send_goal(self.goal)
			waiting = self.move_base.wait_for_result(rospy.Duration(300))
			if waiting == 1:
				rospy.loginfo("Reached ")
				rospy.sleep(2)
			
			self.task_exec = True

			rospy.loginfo("Doing task ---- Talk to people")
			self.text2audio(self.description)

			# while self.task_exec and not rospy.is_shutdown():
			# 	rospy.loginfo("Waiting for task to complete...")
			# 	rospy.sleep(10)		

			rospy.loginfo("Task completed")
			
			rospy.sleep(5)

			# After visiting each corner, robot will go back to starting point
			rospy.loginfo("Going back initial point")
			rospy.sleep(2)
			self.goal.target_pose.pose = self.origin
			self.move_base.send_goal(self.goal)
			end_point = self.move_base.wait_for_result(rospy.Duration(300))
			if end_point == 1:
				rospy.loginfo("Reached initial point")
				rospy.sleep(2)
				
			rospy.Rate(5).sleep()

	def update_initial_pose(self, initial_pose):
		global original
		self.initial_pose = initial_pose
		if original == 0:
			self.origin = self.initial_pose.pose.pose
			original = 1

	def cleanup(self):
		rospy.loginfo("Shutting down navigation	....")
		self.move_base.cancel_goal()

	def task_exec_callback(self, task):
		if task.status.status == 3:  # 3 indicates the goal was reached
			rospy.loginfo("Goal reached: %s", task.status.text)
			self.task_exec = True
		else:
			rospy.loginfo("Goal not reached. Status: %d, Text: %s", task.status.status, task.status.text)
			self.task_exec = False

	def task_status_callback(self,data):
		print(data.data)
		final_data = data.data
		status = final_data.split('-')[0]
		word = final_data.split('-')[1]
		if status == "True":
			self.task_exec = False
			self.description = word
			rospy.loginfo('Done performing task')
	
	def handle_payload(self, received_payload):
		rospy.loginfo(received_payload.data)
		# self.text2audio(received_payload.data)
		self.task_exec = False
		rospy.loginfo('Done performing task')

	def text2audio(self, text):
		tts = gTTS(text)
		tts.save("main_audio.mp3")
		os.system("mpg321 main_audio.mp3")
		os.remove("main_audio.mp3")
        
if __name__=="__main__":
	rospy.init_node('navi_point')
	try:
		officeTour()
	except:
		pass