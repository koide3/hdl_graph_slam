#!/usr/bin/python
# SPDX-License-Identifier: BSD-2-Clause
import re
import os
import sys
import struct
import numpy
import scipy.io

import rospy
import rosbag
import progressbar
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
from geographic_msgs.msg import GeoPointStamped


def gps2navsat(filename, bag):
	with open(filename, 'r') as file:
		try:
			while True:
				data = struct.unpack('qddd', file.read(8*4))
				time = data[0]
				local = data[1:]
				lat_lon_el_theta = struct.unpack('dddd', file.read(8*4))
				gps_cov = struct.unpack('dddddddddddddddd', file.read(8*16))

				if abs(lat_lon_el_theta[0]) < 1e-1:
					continue

				navsat = NavSatFix()
				navsat.header.frame_id = 'gps'
				navsat.header.stamp = rospy.Time.from_sec(time * 1e-6)
				navsat.status.status = NavSatStatus.STATUS_FIX
				navsat.status.service = NavSatStatus.SERVICE_GPS

				navsat.latitude = lat_lon_el_theta[0]
				navsat.longitude = lat_lon_el_theta[1]
				navsat.altitude = lat_lon_el_theta[2]

				navsat.position_covariance = numpy.array(gps_cov).reshape(4, 4)[:3, :3].flatten().tolist()
				navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

				bag.write('/gps/fix', navsat, navsat.header.stamp)

				geopoint = GeoPointStamped()
				geopoint.header = navsat.header
				geopoint.position.latitude = lat_lon_el_theta[0]
				geopoint.position.longitude = lat_lon_el_theta[1]
				geopoint.position.altitude = lat_lon_el_theta[2]

				bag.write('/gps/geopoint', geopoint, geopoint.header.stamp)

		except:
			print 'done'


def mat2pointcloud(filename):
	m = scipy.io.loadmat(filename)
	scan = numpy.transpose(m['SCAN']['XYZ'][0][0]).astype(numpy.float32)
	stamp = m['SCAN']['timestamp_laser'][0][0][0][0] * 1e-6

	cloud = PointCloud2()
	cloud.header.stamp = rospy.Time.from_sec(stamp)
	cloud.header.frame_id = 'velodyne'
	cloud = pc2.create_cloud_xyz32(cloud.header, scan)
	return cloud


def main():
	if len(sys.argv) < 2:
		print 'usage: ford2bag.py src_dirname output_filename'

	output_filename = sys.argv[1]

	rospy.init_node('bag')
	filenames = sorted(['SCANS/' + x for x in os.listdir('SCANS') if re.match('Scan[0-9]*\.mat', x)])
	print filenames

	progress = progressbar.ProgressBar(max_value=len(filenames))
	pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=32)
	with rosbag.Bag(output_filename, 'w') as bag:
		gps2navsat('GPS.log', bag)
		for i, filename in enumerate(filenames):
			if rospy.is_shutdown():
				break
			progress.update(i)
			cloud = mat2pointcloud(filename)
			if pub.get_num_connections():
				pub.publish(cloud)
			bag.write('/velodyne_points', cloud, cloud.header.stamp)


if __name__ == '__main__':
	main()
