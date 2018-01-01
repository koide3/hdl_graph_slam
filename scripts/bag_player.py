#!/usr/bin/python
import sys
import yaml
import time
import curses
import StringIO

import rospy
import rosbag
import roslib
from std_msgs.msg import *
from sensor_msgs.msg import *
from rosgraph_msgs.msg import *
from progressbar import ProgressBar


class BagPlayer:
	def __init__(self, bagfile):
		print 'opening...',
		self.bag = rosbag.Bag(bagfile, 'r')
		print 'done'

		self.message_generator = self.bag.read_messages()

		info_dict = yaml.load(self.bag._get_yaml_info())
		self.duration = float(info_dict['duration'])
		self.endtime = float(info_dict['end'])

		self.progress = ProgressBar(0, self.duration, term_width=80, fd=StringIO.StringIO())

		self.publishers = {}
		for con in self.bag._get_connections():
			msg_class = roslib.message.get_message_class(con.datatype)
			self.publishers[con.topic] = rospy.Publisher(con.topic, msg_class, queue_size=256)
		self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=256)

		self.time_subs = {}
		self.target_times = {}
		self.latest_stamps = {}

		self.play()

	def update_time_subs(self):
		for topic_name, msg_type in rospy.get_published_topics():
			if 'read_until' in topic_name and 'std_msgs/Header' in msg_type:
				if topic_name not in self.time_subs:
					print 'ADD', topic_name
					self.time_subs[topic_name] = rospy.Subscriber(topic_name, Header, self.time_callback, topic_name)

	def time_callback(self, header_msg, topic_name):
		if header_msg.frame_id not in self.target_times:
			self.target_times[header_msg.frame_id] = {}
		self.target_times[header_msg.frame_id][topic_name] = header_msg.stamp

	def play_realtime(self, duration):
		topic, msg, stamp = self.message_generator.next()
		stamp_wall = time.time()

		start_stamp = stamp
		start_stamp_wall = stamp_wall

		self.start_stamp = start_stamp

		while not rospy.is_shutdown() and (stamp - start_stamp) < duration:
			stamp_wall = time.time()
			elapsed_stamp = stamp - start_stamp
			if (stamp_wall - start_stamp_wall) < (elapsed_stamp.secs + elapsed_stamp.nsecs * 1e-9):
				time.sleep(1e-6)
				self.update_time_subs()
				continue

			clock_msg = Clock()
			clock_msg.clock = stamp

			self.clock_pub.publish(clock_msg)
			self.publishers[topic].publish(msg)

			topic, msg, stamp = self.message_generator.next()

		return topic, msg, stamp

	def print_progress(self, stamp):
		self.stdscr.clear()
		self.stdscr.addstr(0, 0, 'topic')
		self.stdscr.addstr(0, 25, 'time')

		line = 1
		for target in self.target_times:
			if target not in self.publishers:
				continue

			for sub_name in self.target_times[target]:
				target_time = self.target_times[target][sub_name]
				self.stdscr.addstr(line, 0, sub_name[:-11])
				self.stdscr.addstr(line, 25, '%.6f' % (target_time.secs + target_time.nsecs * 1e-9))

				residual = target_time - self.latest_stamps[target].stamp

				color = 1 if residual.to_sec() > 0.0 else 2
                                self.stdscr.addstr(line, 50, '%.5f' % residual.to_sec(), curses.color_pair(color))
				line += 1

		if not hasattr(self, 'prev_stamp'):
			self.prev_stamp = stamp
			self.prev_stamp_wall = time.time()
			self.processing_speed = 1.0
		elif time.time() - self.prev_stamp_wall > 1.0:
			sim_duration = (stamp - self.prev_stamp).to_sec()
			wall_duration = time.time() - self.prev_stamp_wall
			self.processing_speed = sim_duration / wall_duration

		self.stdscr.addstr(line, 0, 'current_stamp')
                self.stdscr.addstr(line, 25, '%.6f' % stamp.to_sec())
                self.stdscr.addstr(line, 50, '(x%.2f)' % self.processing_speed)

		elapsed = (stamp - self.start_stamp).to_sec()
		self.progress.fd = StringIO.StringIO()
		try:
			self.progress.update(elapsed)
		except:
			pass
		self.stdscr.addstr(line + 1, 0, '----------')
		self.stdscr.addstr(line + 2, 0, self.progress.fd.getvalue())

		self.stdscr.refresh()

	def check_stamp(self, topic, msg):
		if topic not in self.target_times:
			return True

		target_time_map = self.target_times[topic]
		for sub_name in target_time_map:
			self.latest_stamps[topic] = msg.header
			if msg.header.stamp > target_time_map[sub_name]:
				return False

		return True

	def play(self):
		print 'play realtime for 3.0[sec]'
		topic, msg, stamp = self.play_realtime(rospy.Duration(15.0))
		self.update_time_subs()

		print 'play as fast as possible'
		self.stdscr = curses.initscr()
		curses.start_color()
		curses.noecho()

		curses.init_pair(1, curses.COLOR_BLUE, curses.COLOR_WHITE)
		curses.init_pair(2, curses.COLOR_RED, curses.COLOR_WHITE)

		try:
			while not rospy.is_shutdown():
				if not self.check_stamp(topic, msg):
					self.update_time_subs()
					self.print_progress(stamp)
					time.sleep(0.1)
					continue

				clock_msg = Clock()
				clock_msg.clock = stamp

				self.clock_pub.publish(clock_msg)
				self.publishers[topic].publish(msg)
				topic, msg, stamp = self.message_generator.next()
		except:
			clock_msg = Clock()
			clock_msg.clock = stamp + rospy.Duration(30.0)
			self.clock_pub.publish(clock_msg)
			time.sleep(0.5)

		curses.echo()
		curses.endwin()


def main():
	if len(sys.argv) < 2:
		print 'usage bag_player src_bagname'
		return

	rospy.init_node('bag_player')
	for argv in sys.argv[1:]:
		print argv
		bp = BagPlayer(argv)

if __name__ == '__main__':
	main()
