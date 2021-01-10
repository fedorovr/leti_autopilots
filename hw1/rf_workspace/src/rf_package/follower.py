#! /usr/bin/python

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from numpy import pi, hypot, arctan2
import os


class Main():
	def __init__(self):
		self.target_pose = Pose()
		self.follower_pose = Pose()
		self.target_sub = rospy.Subscriber('/michelangelo/pose', Pose, self.set_target)
		self.follower_sub = rospy.Subscriber('/raphael/pose', Pose, self.set_follower)
		self.follower_pub = rospy.Publisher('/raphael/cmd_vel', Twist, queue_size = 1)


	def set_target(self, pose):
		self.target_pose = pose


	def set_follower(self, pose):
		self.follower_pose = pose


	def run(self):
		while not rospy.is_shutdown():
			if self.distance() < 0.1 and self.distance() != 0:
				rospy.logerr('YOU ARE CAUGHT! YOU LOSE.')
				nodes = os.popen("rosnode list").readlines()
                                for i in range(len(nodes)):
                                    nodes[i] = nodes[i].replace("\n", "")
                                for node in nodes:
                                    os.system("rosnode kill " + node)
			else:
				angle = self.get_angle_target()
				msg = Twist()
				msg.angular.z = angle
				msg.linear.x = self.get_speed(angle)
				self.follower_pub.publish(msg)


	def get_speed(self, angle):
		return (pi - abs(angle)) / pi


	def distance(self):
		return hypot(self.target_pose.x - self.follower_pose.x, self.target_pose.y - self.follower_pose.y)


	def get_angle_target(self):
		location = self.follower_pose.theta
		follower_angle = arctan2(self.target_pose.y - self.follower_pose.y, self.target_pose.x - self.follower_pose.x)
		if location >= 0:
			if follower_angle >= location:
				return follower_angle - location
			else:
				if location - pi <= follower_angle:
					return follower_angle - location
				else:
					return follower_angle - location + 2 * pi
		else:
			if follower_angle <= location:
				return follower_angle - location
			else:
				if location + pi >= follower_angle:
					return follower_angle - location
				else:
					return follower_angle - location - 2 * pi
rospy.init_node('follower')
rospy.wait_for_service('/spawn')
spawn_func = rospy.ServiceProxy('/spawn', Spawn)
res = spawn_func(0.0, 5.0, 3.0, 'raphael')
m = Main()
m.run()
