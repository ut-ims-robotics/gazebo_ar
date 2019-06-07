# ar_tf

#### This package is for translating AR tag marker positions to camera positions for Gazebo in order to use Gazebo as an AR headset visualisation interface.

The ROS distribution supported is Kinetic Kame and the Gazebo version supported is 10.0.0.

Packages necessary for the entire system: usb_cam ([http://wiki.ros.org/usb_cam]), ar_track_alvar ([http://wiki.ros.org/ar_track_alvar]) and gazebo_ros ([http://wiki.ros.org/gazebo_ros]).

### Installation of necessary packages: 

`sudo apt-get install ros-kinetic-ar-track-alvar`

`sudo apt-get install ros-kinetic-usb-cam`

Guide for Gazebo installation: [http://gazebosim.org/tutorials?tut=ros_installing#InstallGazebo]. 

### Necessary configuration:

From the launch files provided in the packages, usb_cam usb_cam-test.launch and for ar_track-alvar pr2_indiv.launch can be used as launch files.

For launching Gazebo, refer to [http://gazebosim.org/tutorials/?tut=ros_roslaunch].

Define marker locations and used marker names in tag_config.yaml.

Define transform between AR-headset camera and screen in gazebo_ar.launch.

Set ~camera_frame_id as "usb_cam" in usb_cam launch file.

If gzserver and gzclient are run on different devices, gzserver may be ran in headless mode.

A Gazebo world needs to be created in respect of marker locations defined in tag_config.yaml.
