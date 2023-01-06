# ROS robot with AprilTag localization

WIP


## System Setup
- https://varhowto.com/install-ros-noetic-raspberry-pi-4 (works fine for my pi 3B+)
- ~~https://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi~~

### System dependencies
- Maybe system dependencies: opencv3+4?, java (maybe?), curl, libssl-dev, libcurl4-openssl-dev, libtinyxml-dev, liburdfdom-dev, liburdfdom-headers-dev, and more
  - For opencv, see https://howchoo.com/pi/install-opencv-on-raspberry-pi. I think it's necessary to include extra modules (for at least 4, maybe 3?)
    - If getting stuck, can increase swap size to 2048M and decrease GPU memory to 16M (in raspi-config)
    - It might be necessary to build both with -DWITH_GTK_2_X=ON in order to avoid rosrun errors?

### ROS packages
All performed in `~/catkin_ws/`
1. Generate a dependency list with [rosinstall_generator](https://wiki.ros.org/rosinstall_generator): `rosinstall_generator sensor_msgs ros_comm roslint navigation apriltag_ros image_common image_proc cv_camera --rosdistro noetic --deps --wet-only --tar > wet1.rosinstall`
2. Install ROS dependencies with [wstool](https://wiki.ros.org/wstool)
    - If it's the first time running wstool, use `wstool init -j4 -t src wet1.rosinstall`
    - If you're updating the list of packages, use `wstool merge -t src wetX.rosinstall` then `wstool update -j4 -t src`
3. Install system dependencies: `rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster`
    -*Does rosdep install big dependencies like opencv and java?*
4. Build packages: `sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3`
    - To build a specific package, add on `--pkg packagename`

### Camera
- Uses the "tag36h11" tags from https://github.com/AprilRobotics/apriltag-imgs
- Camera must be calibrated as described in http://wiki.ros.org/camera_calibration/. This converts from topic /camera/image_raw to /camera/image_rect that apriltag_ros needs. 


## ROS Nodes
- **mymotors/motor_controller**: subscribes to /cmd_vel that's generated from the ROS navigation stack and moves the robot accordingly
- **mynavigation/localizer**: subscribes to /tag_detections and /cmd_vel and combines them to calculate predicted robot position 
- **mynavigation/tag_positions**: publishes positions of the apriltags to be displayed in rviz


## Running
- `catkin_make_isolated` and `source setup.sh`
- `roslaunch mynavigation main.launch video:=true navigation:=true`
- Use rviz to set a navigation goal
