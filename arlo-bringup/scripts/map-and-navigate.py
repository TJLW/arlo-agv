#!/usr/bin/env python
import rospy
import math
import signal
import subprocess
import tf
import time

import geometry_msgs.msg

saved_map_filepath = '/home/tjlw/ROS/maps/latest_cartographer_map'

cartographer_configuration_directory = '/home/tjlw/ROS/catkin_ws/src/arlo/configuration_files/'

cartographer_configuration_basename = 'arlo_cartographer_params.lua'


if __name__ == '__main__':
	rospy.init_node('cartographer_tf_listener')

	listener = tf.TransformListener()

	trajectory_id = 0

	pose_x = 0.0
	pose_y = 0.0
	rot_z = 0.0

	trans = None
	rot = None

	odom_translation_relative_to_map = None
	odom_rotation_relative_to_map = None

	base_link_translation_relative_to_odom = None
	base_link_rotation_relative_to_odom = None

	base_link_translation_relative_to_map = None
	base_link_quaternion_relative_to_map = None


	base_link_pose_x = 0.0
	base_link_pose_y = 0.0
	base_link_pose_z = 0.0

	base_link_rot_x = 0.0
	base_link_rot_y = 0.0
	base_link_rot_z = 0.0

	base_link_quat_x = 0.0
	base_link_quat_y = 0.0
	base_link_quat_z = 0.0
	base_link_quat_w = 0.0

	while 1:


		# Assuming cartopgrapher is already running, we have started in map model
		# 	-Trajectory 0 is started on default
		print 'Starting cartographer map server...'
		map_process = subprocess.Popen(['rosrun', 'arlo', 'map'])

		# if trajectory_id > 0:
		# 	# Start new trajectory to continue cartographer SLAM
		# 	start_trajectory_call = subprocess.Popen(['rosrun', 'arlo',
		# 	'start_trajectory', cartographer_configuration_directory, cartographer_configuration_basename, '0', str(base_link_pose_x), str(base_link_pose_y), str(base_link_pose_z), str(base_link_rot_x), str(base_link_rot_y), str(base_link_rot_z)])


		input = raw_input("Press ENTER to change to navigation (Q/q to exit)...")

		if 'Q' in input or 'q' in input:
			print 'KILLING PROCESS.'
			kill = subprocess.Popen(['rosrun', 'arlo', 'kill-all-map-and-nav-processes'])
			exit()



		# print 'Ending mapping trajectory...'
		# end_trajectory_process = subprocess.Popen(['rosservice', 'call', '/finish_trajectory', str(trajectory_id)])
		# end_trajectory_process.wait()

		print 'Getting most recent map->odom transform...'
		try:
			# Get last known adjustment of map -> odom transform
			(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
			print trans
			print rot

		except:
			print 'TF lookup error...'

		# Run without arguments, no transform is applied to served map
		#map_updater = subprocess.Popen(['rosrun', 'arlo', 'save-and-serve-map', '&'])


		print 'Saving and serving a copy of latest map...'
		# Run with arguments, a transform is applied to the served map
		map_updater = subprocess.Popen(['rosrun', 'arlo', 'save-and-serve-map', str(trans[0]), str(trans[1]), str(trans[2]), str(rot[0]), str(rot[1]), str(rot[2]), str(rot[3]), '&'])

		print 'Starting navigation...'
		# Switch to navigation
		nav_process = subprocess.Popen(['rosrun', 'arlo', 'navigate'])


		input = raw_input("Press ENTER to change to mapping...")

		if 'Q' in input or 'q' in input:
			print 'KILLING PROCESS.'
			kill = subprocess.Popen(['rosrun', 'arlo', 'kill-all-map-and-nav-processes'])
			exit()


		# try:
		# 	# Get last known adjustment of map -> odom transform
		# 	(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
		# 	print trans
		# 	print rot
		#
		# except:
		# 	print 'TF lookup error...'
		# 	exit()

		print 'Getting most recent map->odom transform...'
		try:
			# Get last known adjustment of map -> odom transform
			(odom_translation_relative_to_map, odom_rotation_relative_to_map) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
			print odom_translation_relative_to_map
			print odom_rotation_relative_to_map
		except:
			print 'Map -> odom TF lookup error...'

		print 'Getting most recent odom->base_link transform...'
		try:
			# Get last known adjustment of map -> odom transform
			(base_link_translation_relative_to_odom, base_link_rotation_relative_to_odom) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
			print base_link_translation_relative_to_odom
			print base_link_rotation_relative_to_odom
		except:
			print 'Odom -> base_link TF lookup error...'

		print 'Getting most recent base_link->map transform...'
		try:
			# Get last known adjustment of map -> odom transform
			(base_link_translation_relative_to_map, base_link_quaternion_relative_to_map) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
			print base_link_translation_relative_to_map
			print base_link_quaternion_relative_to_map
		except (e):
			print 'Map -> base_link TF lookup error...'
			print e

		# Determine position for new trajectory
		base_link_pose_x = base_link_translation_relative_to_map[0]
		base_link_pose_y = base_link_translation_relative_to_map[1]
		base_link_pose_z = base_link_translation_relative_to_map[2]

		euler_orientation = tf.transformations.euler_from_quaternion(base_link_quaternion_relative_to_map)
		base_link_rot_x = euler_orientation[0]
		base_link_rot_y = euler_orientation[1]
		base_link_rot_z = euler_orientation[2]


		# base_link_quat_x = base_link_quaternion_relative_to_map[0]
		# base_link_quat_y = base_link_quaternion_relative_to_map[1]
		# base_link_quat_z = base_link_quaternion_relative_to_map[2]
		# base_link_quat_w = base_link_quaternion_relative_to_map[3]

		# # Determine position for new trajectory
		# base_link_pose_x = base_link_translation_relative_to_odom[0] + base_link_translation_relative_to_odom[0]
		# base_link_pose_y = base_link_translation_relative_to_odom[1] + base_link_translation_relative_to_odom[1]
		# base_link_pose_z = base_link_translation_relative_to_odom[2] + base_link_translation_relative_to_odom[2]
		#
		#
		# base_link_quat_x = base_link_rotation_relative_to_odom[0] + odom_rotation_relative_to_map[0]
		# base_link_quat_y = base_link_rotation_relative_to_odom[1] + odom_rotation_relative_to_map[1]
		# base_link_quat_z = base_link_rotation_relative_to_odom[2] + odom_rotation_relative_to_map[2]
		# base_link_quat_w = base_link_rotation_relative_to_odom[3] + odom_rotation_relative_to_map[3]

		# Incrementing to next trajectory
		trajectory_id += 1

