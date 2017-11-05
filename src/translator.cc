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

void convertToGzPose(const geometry_msgs::Pose& marker_pose) {
//MAYBE TODO: Stuff all this in a class so I don't need globals
   ignition::math::Quaternion<double> cam_quaternion(/*1,0,0,0*/
      marker_pose.orientation.w,
      marker_pose.orientation.z,
      -marker_pose.orientation.x,
      marker_pose.orientation.y
   );
   ignition::math::Vector3<double> cam_position(
      marker_pose.position.z,
      -marker_pose.position.x,
      marker_pose.position.y + 0.5
   );
   /* ignition::math::Quaternion<double> z_quaternion(0, 0, 3.14159); */
   /* cam_quaternion *= z_quaternion; */
   ignition::math::Pose3<double> intermed(cam_position, cam_quaternion);

   last_pose = gazebo::msgs::Convert(intermed);
}

void artagReceived(const visualization_msgs::Marker::ConstPtr& msg) {
   ROS_INFO("CALLBACK RECEIVED");   
   convertToGzPose(msg->pose);
   /* last_pose = msg->pose; */
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
   ros::NodeHandle ros_node;
   ros::Subscriber ros_sub = ros_node.subscribe("visualization_marker", 100, artagReceived);
   /* ros::Publisher ros_pub = ros_node.advertise("gazebo/set_model_state", 100); */
   ROS_INFO("ROS Setup done");
   ros::Rate r(100);

	while (ros::ok()) {
		gzCamPub->Publish(last_pose);
      /* ros_pub.publish */

      ros::spinOnce();
      r.sleep();
	}

	// Make sure to shut everything down.
	gazebo::client::shutdown();
}

