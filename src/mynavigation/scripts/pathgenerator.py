#!/usr/bin/env python2

import math

import rospy
import geometry_msgs.msg
import nav_msgs.srv
import nav_msgs.msg
from tf.transformations import quaternion_from_euler


staticmap = None
latest_path = None

pub = rospy.Publisher("pathgenerated", nav_msgs.msg.Path, queue_size=1)


def orientation_between(pos1, pos2):
	dx = pos2.x - pos1.x
	dy = pos2.y - pos1.y
	angle = math.atan2(dy, dx)

	ret_array = quaternion_from_euler(0, 0, angle)
	ret = geometry_msgs.msg.Quaternion()
	ret.x = ret_array[0]
	ret.y = ret_array[1]
	ret.z = ret_array[2]
	ret.w = ret_array[3]
	return ret

def calculate_path(themap, start, target):
	fakestart = geometry_msgs.msg.PoseStamped()
	fakestart.pose.position.x = 0
	fakestart.pose.position.y = 0
	fakestart.pose.position.z = 0
	start = fakestart # TODO
	
	# width = themap.map.info.width
	# height = themap.map.info.height
	# origin = themap.map.info.origin # type geometry_msgs/Pose
	rotation_to_target = orientation_between(start.pose.position, target.pose.position)

	# Calculate path: simple turn and face
	poses = []
	
	p_0 = geometry_msgs.msg.PoseStamped()
	p_0.pose.position = start.pose.position
	p_0.pose.orientation = rotation_to_target
	poses.append(p_0)

	p_half = geometry_msgs.msg.PoseStamped()
	p_half.pose.position.x = (target.pose.position.x - start.pose.position.x)/2
	p_half.pose.position.y = (target.pose.position.y - start.pose.position.y)/2
	p_half.pose.position.z = 0
	p_half.pose.orientation = rotation_to_target
	poses.append(p_half)

	p_f = geometry_msgs.msg.PoseStamped()
	p_f.pose.position = target.pose.position
	p_f.pose.orientation = target.pose.orientation
	poses.append(p_f)

	for pose in poses:
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "map"

	p = nav_msgs.msg.Path()
	p.header.stamp = rospy.Time.now()
	p.header.frame_id = "map"
	p.poses = poses # type geometry_msgs/PoseStamped[]
	return p

def callback_target(data):
	# rviz publishes goal on topic move_base_simple/goal, type geometry_msgs/PoseStamped (https://stackoverflow.com/questions/26719219)
	path = calculate_path(staticmap, None, data)
	# pub.publish(path)
	global latest_path
	latest_path = path

	rospy.loginfo("Got navigation target and updated the path")


def main():

	rospy.init_node("pathgenerator", anonymous=False)

	# Get the room map from the map_server node via the static_map service
	rospy.wait_for_service("static_map")
	get_staticmap = rospy.ServiceProxy("static_map", nav_msgs.srv.GetMap)
	try:
		global staticmap
		staticmap = get_staticmap()
	except rospy.ServiceException as e:
		print("map_server static_map service did not process request:", e)

	# subscribe to robot position...

	rospy.Subscriber("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, callback_target)

	rate = rospy.Rate(1) # in hz
	while not rospy.is_shutdown():
		if latest_path is not None:
			pub.publish(latest_path)
		rate.sleep()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	main()

