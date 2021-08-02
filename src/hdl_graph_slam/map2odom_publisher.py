#!/usr/bin/python
# SPDX-License-Identifier: BSD-2-Clause
import tf
import rospy
from geometry_msgs.msg import *


class Map2OdomPublisher:
	def __init__(self):
		self.broadcaster = tf.TransformBroadcaster()
		self.subscriber = rospy.Subscriber('/hdl_graph_slam/odom2map', TransformStamped, self.callback)

		self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
		self.map_frame_id = rospy.get_param('~map_frame_id', 'map')

	def callback(self, odom_msg):
		self.odom_msg = odom_msg

	def spin(self):
		
		if not hasattr(self, 'odom_msg'):
			self.broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), self.odom_frame_id, self.map_frame_id)
			return

		pose = self.odom_msg.transform
		pos = (pose.translation.x, pose.translation.y, pose.translation.z)
		quat = (pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)
		
		self.broadcaster.sendTransform(pos, quat, rospy.Time.now(), self.odom_frame_id, self.map_frame_id)

if __name__ == '__main__':
	rospy.init_node('map2odom_publisher')
	node = Map2OdomPublisher()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		node.spin()
		rate.sleep()

