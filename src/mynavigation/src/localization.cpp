#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // used to convert tf2::Quaternion to geometry_msgs::Quaternion
#include <geometry_msgs/Twist.h>
#include <cmath>


geometry_msgs::Twist lastCommand; // units are m/s and rad/s

geometry_msgs::TransformStamped currentPosition;
float currentYaw = 0; // RPY needs to be separated because TransformStamped only stores quaternion rotation
nav_msgs::Odometry currentOdom;
ros::Time lastUpdateTime;

// tf2_ros::Buffer tfBuffer;
// tf2_ros::TransformListener tfListener(tfBuffer);

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

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray& msg) {
  float avgX = 0;
  float avgY = 0;

  std::string frame_id = msg.header.frame_id;
  int count = msg.detections.size();
  for(int i=0; i<count; i++) {
    // AprilTagDetectionArray -> AprilTagDetection -> PoseWithCovarianceStamped -> PoseWithCovariance -> Pose
    int id = msg.detections[i].id[0];
    geometry_msgs::Pose originalPose = msg.detections[i].pose.pose.pose;
    geometry_msgs::PointStamped originalPointStamped;
    originalPointStamped.header.frame_id = frame_id;
    originalPointStamped.point = originalPose.position;
    ROS_INFO("Got detections (#%d): f=%s, id=%d, p=(%f, %f, %f)", i, frame_id, id, originalPointStamped.point.x, originalPointStamped.point.y, originalPointStamped.point.z);

    // Convert detected tag position to odom frame
    geometry_msgs::PointStamped newPointStamped;
    // tfBuffer.transform(originalPointStamped, newPointStamped, "odom");
    // Other method of transforming: 
    // geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(frame_id, "odom", ros::Time::now());
    // tf2::doTransform();
    ROS_INFO("  transformed=(%f, %f, %f)", i, frame_id, id, newPointStamped.point.x, newPointStamped.point.y, newPointStamped.point.z);

    // Convert config tag position to odom frame

    // Perform vector arithmetic: robot = detected - actual


    avgX += newPointStamped.point.x;
    avgY += newPointStamped.point.y;
  }

  avgX /= count;
  avgY /= count;

  currentPosition.transform.translation.x = avgX;
  currentPosition.transform.translation.y = avgY;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "localization_node");

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

  ros::Subscriber cmdVelSub = nodeHandle.subscribe("/cmd_vel", 0, &cmdVelCallback); // queue size
  ros::Subscriber tagDetectionsSub = nodeHandle.subscribe("/tag_detections", 0, &tagDetectionsCallback);
  tf2_ros::TransformBroadcaster transformBroadcaster;
  ros::Publisher odomPublisher = nodeHandle.advertise<nav_msgs::Odometry>("odom", 10);

  ROS_INFO("Subscribed to /cmd_vel, starting to broadcast+publish in loop");

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    updateCurrentPosition();
    transformBroadcaster.sendTransform(currentPosition);
    odomPublisher.publish(currentOdom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
};