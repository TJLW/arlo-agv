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

	while 1:

		try:
			# Always start with cartographer map server to start with something
			cartographer_map_server = subprocess.Popen(['rosrun', 'cartographer_ros', 'cartographer_occupancy_grid_node', '-resolution', '0.05'])


			time.sleep(10)

			# Save a copy of cartographer's dope ass SLAM map
			map_saver = subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', saved_map_filepath])
			map_saver.wait()

			# End the current trajectory to stop cartographer
			end_trajectory_call = subprocess.Popen(['rosservice', 'call', '/finish_trajectory', str(trajectory_id)])
			end_trajectory_call.wait()

			# Get last known adjustment of map -> odom transform
			(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
			print trans
			print rot

			# Kill the cartographer map server
			cartographer_map_server.terminate()

			# Now we serve the map
			navigation_map_server = subprocess.Popen(['rosrun', 'map_server', 'map_server', saved_map_filepath + '.yaml'])

			# Also broadcast a transform for map -> odom
			odom_tf_broadcaster = subprocess.Popen(['rosrun', 'tf', 'static_transform_publisher','0','0','0','0','0','0', 'map', 'odom', '100'])

			time.sleep()

		except:
			print 'KILLING'
			cartographer_map_server.terminate()
			end_trajectory_call.terminate()
			map_saver.terminate()
			navigation_map_server.terminate()
			odom_tf_broadcaster.terminate()
			exit()



		# odom_tf_broadcaster.wait()
		# navigation_map_server.terminate()
		# odom_tf_broadcaster.terminate()
		#
		#
		#
		# # Start new trajectory to continue cartographer SLAM
		# start_trajectory_call = subprocess.Popen(['/home/tjlw/ROS/catkin_ws/src/arlo/bash/start_trajectory', cartographer_configuration_directory, cartographer_configuration_basename, str(trajectory_id), str(pose_x), str(pose_y), str(rot_z)])
		# start_trajectory_call.wait()
		#
		#
		# trajectory_id += 1
		#
		# print "Waiting..."
		# time.sleep(15)

