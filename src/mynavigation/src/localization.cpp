#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // used to convert tf2::Quaternion to geometry_msgs::Quaternion
#include <geometry_msgs/Twist.h>
#include <cmath>


geometry_msgs::Twist lastCommand; // units are m/s and rad/s

geometry_msgs::TransformStamped currentPosition;
float currentYaw = 0; // RPY needs to be separated because TransformStamped only stores quaternion rotation
nav_msgs::Odometry currentOdom;
ros::Time lastUpdateTime;

void updateCurrentPosition() {
  ros::Time currentTime = ros::Time::now();
  double dt = currentTime.toSec() - lastUpdateTime.toSec();

  currentPosition.header.stamp = currentTime;
  currentYaw += lastCommand.angular.z * dt;
  tf2::Quaternion q;
  q.setRPY(0, 0, currentYaw);
  currentPosition.transform.rotation.x = q.x();
  currentPosition.transform.rotation.y = q.y();
  currentPosition.transform.rotation.z = q.z();
  currentPosition.transform.rotation.w = q.w();
  currentPosition.transform.translation.x += cos(currentYaw) * lastCommand.linear.x * dt;
  currentPosition.transform.translation.y += sin(currentYaw) * lastCommand.linear.x * dt;

  currentOdom.header.stamp = currentTime;
  currentOdom.pose.pose.position.x = currentPosition.transform.translation.x;
  currentOdom.pose.pose.position.y = currentPosition.transform.translation.y;
  currentOdom.pose.pose.orientation = tf2::toMsg(q);
  currentOdom.twist.twist.linear.x = lastCommand.linear.x;
  currentOdom.twist.twist.angular.z = lastCommand.angular.z;

  lastUpdateTime = currentTime;
}

void cmdVelCallback(const geometry_msgs::Twist& msg){
  // ROS_INFO("Updating velocity: x=%f, y=%f, yaw=%f", msg.linear.x, msg.linear.y, msg.angular.z);
  updateCurrentPosition();
  lastCommand = msg;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "localizer_node");

  lastCommand.linear.x = 0;
  lastCommand.angular.z = 0;

  currentPosition.header.frame_id = "odom";
  currentPosition.child_frame_id = "base_link";
  currentPosition.transform.translation.x = 0.0;
  currentPosition.transform.translation.y = 0.0;
  currentPosition.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  currentPosition.transform.rotation.x = q.x();
  currentPosition.transform.rotation.y = q.y();
  currentPosition.transform.rotation.z = q.z();
  currentPosition.transform.rotation.w = q.w();


  currentOdom.header.frame_id = "odom";
  currentOdom.child_frame_id = "base_link";
  currentOdom.pose.pose.position.x = 0.0;
  currentOdom.pose.pose.position.y = 0.0;
  currentOdom.pose.pose.position.z = 0.0;
  currentOdom.pose.pose.orientation = tf2::toMsg(q);
  currentOdom.twist.twist.linear.x = 0.0;
  currentOdom.twist.twist.linear.y = 0.0;
  currentOdom.twist.twist.linear.z = 0.0;
  currentOdom.twist.twist.angular.z = 0.0;
  currentOdom.twist.twist.angular.y = 0.0;
  currentOdom.twist.twist.angular.z = 0.0;

  ros::NodeHandle nodeHandle;

  ros::Subscriber sub = nodeHandle.subscribe("/cmd_vel", 10, &cmdVelCallback); // queue size
  tf2_ros::TransformBroadcaster broadcaster;
  ros::Publisher publisher = nodeHandle.advertise<nav_msgs::Odometry>("odom", 10); // queue size

  ROS_INFO("Subscribed to /cmd_vel, starting to broadcast+publish in loop");

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    updateCurrentPosition();
    broadcaster.sendTransform(currentPosition);
    publisher.publish(currentOdom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
};