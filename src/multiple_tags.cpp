#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <ignition/math.hh>

#include <iostream>
#include <string>

#define numberoftags 2

int main(int _argc, char** _argv){
  ros::init(_argc, _argv, "tag_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;
  
  //variables
  int count;
  std::string tagnames[numberoftags] = {"ar_marker_0", "ar_marker_1"};
  gazebo::msgs::Pose cam_pose;
  tf::Quaternion orientation;
  double alpha=0.2; //for calculating running exp average, higher alpha discounts older observations faster
  double runningroll;
  double runningpitch;
  double runningyaw;
  double runningx;
  double runningy;
  double runningz;
  double qx, qy, qz, qw;

  // gazebo startup
  gazebo::client::setup(_argc, _argv);
  gazebo::transport::NodePtr gzNode(new gazebo::transport::Node());
  gzNode->Init();
  gazebo::transport::PublisherPtr gzCamPub = gzNode->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");
  ROS_INFO("Gazebo setup done");

  //ros startup
  ros::init(_argc, _argv, "gazebo_ar");
  ros::NodeHandle ros_node;

  ROS_INFO("ROS Setup done");

  ros::Rate rate(10.0);

  while (node.ok())
  {
    for (int i=0; i<numberoftags; i++)
    {
      ros::Duration transformtime = ros::Duration(3); //timeout for transforms	
      tf::StampedTransform transform;           
      try{
        listener.lookupTransform(tagnames[i], "sim_cam", ros::Time(0), transform);
        if (ros::Time::now()-transform.stamp_<transformtime) //if transform is recent enough, revalue running values
        { 
          count++;	    
          runningx = (alpha * transform.getOrigin().x()) + (1.0 - alpha) * runningx;
          runningy = (alpha * transform.getOrigin().y()) + (1.0 - alpha) * runningy;
          runningz = (alpha * transform.getOrigin().z()) + (1.0 - alpha) * runningz;
        }
        orientation = (transform.getRotation());
        if (ros::Time::now()-transform.stamp_<transformtime)
        {
          qx = (alpha * orientation.x()) + (1.0 - alpha) * qx;
          qy = (alpha * orientation.y()) + (1.0 - alpha) * qy;
          qz = (alpha * orientation.z()) + (1.0 - alpha) * qz;
          qw = (alpha * orientation.w()) + (1.0 - alpha) * qw;
        }                
      }
      catch (tf::TransformException ex)
      {
          ROS_ERROR("%s",ex.what());
          //ros::Duration(1.0).sleep();
      }
    }
  ignition::math::Vector3<double> cam_position(runningx, runningy, runningz);
  ignition::math::Quaternion<double> runningorientation(qw, qx, qy, qz);
  ignition::math::Pose3<double> intermed(cam_position, runningorientation);
  ROS_INFO_STREAM("xyzw");
  ROS_INFO_STREAM(runningorientation);
  ROS_INFO_STREAM("position");
  ROS_INFO_STREAM(cam_position);	              
  rate.sleep();   
  gzCamPub->Publish(gazebo::msgs::Convert(intermed));
  }
  return 0;
};
