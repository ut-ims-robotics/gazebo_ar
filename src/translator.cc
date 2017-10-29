#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <ignition/math.hh>

#include <iostream>
#include <string>

gazebo::msgs::Pose last_pose;

void convertToGzPose(const geometry_msgs::Pose& rosPosePtr) {
//MAYBE TODO: Stuff all this in a class so I don't need globals
   ignition::math::Pose3<double> intermed(
      rosPosePtr.position.x,
      rosPosePtr.position.y,
      rosPosePtr.position.z,
      rosPosePtr.orientation.x,
      rosPosePtr.orientation.y,
      rosPosePtr.orientation.z,
      rosPosePtr.orientation.w
      );
   last_pose = gazebo::msgs::Convert(intermed);
}

void artagReceived(const visualization_msgs::Marker::ConstPtr& msg) {
   ROS_INFO("CALLBACK RECEIVED");   
   convertToGzPose(msg->pose);
}

int main(int _argc, char **_argv)
{
	// gazebo startup
	gazebo::client::setup(_argc, _argv);
	gazebo::transport::NodePtr gzNode(new gazebo::transport::Node());
	gzNode->Init();
	gazebo::transport::PublisherPtr gzCamPub = gzNode->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");
   ROS_INFO("Gazebo setup done");

   //ros startup
   ros::init(_argc, _argv, "gazebo_ar");
   ros::NodeHandle rosNode;
   ros::Subscriber rosSub = rosNode.subscribe("visualization_marker", 100, artagReceived);
   ROS_INFO("ROS Setup done");
   ros::Rate r(100);

	while (ros::ok()) {
		gzCamPub->Publish(last_pose);
      ros::spinOnce();
      r.sleep();
	}

	// Make sure to shut everything down.
	gazebo::client::shutdown();
}

