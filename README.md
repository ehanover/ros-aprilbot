# ROS robot with AprilTag localization

WIP

## Setup

https://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi

### Dependencies
- Maybe package dependencies: ros_comm ros_control std_msgs geometry_msgs sensor_msgs image_pipeline costmap_2d global_planner map_server move_base move_base_msgs nav_core navfn roslint
  - I also increased swap size to 1 gb while installing on pi
- Maybe system dependencies: opencv3+4, apriltag, java (maybe?), curl, libssl-dev, libcurl4-openssl-dev, libtinyxml-dev, liburdfdom-dev, liburdfdom-headers-dev, and probably more
  - I think using rosdep can show all system requirements?
  - For opencv, see https://howchoo.com/pi/install-opencv-on-raspberry-pi and/or https://linuxize.com/post/how-to-install-opencv-on-raspberry-pi/. I think it's necessary to include extra modules (for at least 4, maybe 3?)
    - It might be necessary to build both with -DWITH_GTK_2_X=ON in order to avoid rosrun errors?
    - Don't forget `sudo make install -j4` and `sudo ldconfig` after building

### Camera
- Uses the "tagStandard41h12" tags from https://github.com/AprilRobotics/apriltag-imgs
- Camera must be calibrated (http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)

