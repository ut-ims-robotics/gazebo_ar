#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <ignition/math.hh>
#include <math.h>

#include <iostream>
#include <string>
#include <boost/circular_buffer.hpp>

#include <tf/transform_broadcaster.h>

#define orientation_buffer_size 45

double running_x, running_y, running_z;

double current_x, current_y, current_z;

double current_roll_sin;
double current_pitch_sin;
double current_yaw_sin;
double current_roll_cos;
double current_pitch_cos;
double current_yaw_cos;

double running_roll_sin;
double running_pitch_sin;
double running_yaw_sin;

double running_roll_cos;
double running_pitch_cos;
double running_yaw_cos;

double current_roll;
double current_pitch;
double current_yaw;

//function for broadcasting a transform between desired tag and world
void broadcastTransform(tf::Vector3 pos, tf::Quaternion q, std::string tag_name)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(pos);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tag_name, "world"));
}

int main(int _argc, char **_argv)
{
  //ros startup
  ros::init(_argc, _argv, "tag_to_cam_pose");
  ros::NodeHandle ros_node;

  tf::TransformListener listener;

  // gazebo startup
  gazebo::client::setup(_argc, _argv);
  gazebo::transport::NodePtr gzNode(new gazebo::transport::Node());
  gzNode->Init();
  gazebo::transport::PublisherPtr gzCamPub = gzNode->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");
  ROS_INFO("Gazebo setup done");

  //std::map<std::string, int> tags_map;
  std::vector<std::string> markers_used;
  ros_node.getParam("/markers_used", markers_used); //predefined in yaml

  ROS_INFO("ROS Setup done");

  ros::Rate rate(10.0);

  //tags are read in from a config
  int tag_array_size = markers_used.size();

  tf::Vector3 tag_positions[tag_array_size];
  tf::Quaternion tag_orientations[tag_array_size];

  std::string temp_string;
  //read in tag positions and orientations from config file
  for (int i = 0; i < tag_array_size; i++)
  {
    double map_x;
    temp_string = markers_used[i] + "/x";
    ros_node.getParam(temp_string, map_x);
    double map_y;
    temp_string = markers_used[i] + "/y";
    ros_node.getParam(temp_string, map_y);
    double map_z;
    temp_string = markers_used[i] + "/z";
    ros_node.getParam(temp_string, map_z);
    tf::Vector3 tagpos(map_x, map_y, map_z);

    double map_roll;
    temp_string = markers_used[i] + "/roll";
    ros_node.getParam(temp_string, map_roll);
    double map_pitch;
    temp_string = markers_used[i] + "/pitch";
    ros_node.getParam(temp_string, map_pitch);
    double map_yaw;
    temp_string = markers_used[i] + "/yaw";
    ros_node.getParam(temp_string, map_yaw);
    tf::Quaternion tagorient;
    tagorient.setRPY(map_roll, map_pitch, map_yaw);

    tag_positions[i] = tagpos;
    tag_orientations[i] = tagorient;
    ROS_INFO_STREAM(tagpos);
    ROS_INFO_STREAM(tagorient);
  }

  //array for storing transforms between real life camera and gazebo world through every tag separately
  tf::StampedTransform cam_transforms[tag_array_size];
  //array for storing transforms between every tag and the camera separately
  tf::StampedTransform tag_transforms[tag_array_size];

  //circular buffer setup for calculating orientation through finding the summary vector of the data and then finding the angle of that vector
  //it is done to solve the problem behind averaging circular data
  typedef boost::circular_buffer<double> circular_buffer;
  circular_buffer roll_buffer_sin{orientation_buffer_size};
  circular_buffer pitch_buffer_sin{orientation_buffer_size};
  circular_buffer yaw_buffer_sin{orientation_buffer_size};

  circular_buffer roll_buffer_cos{orientation_buffer_size};
  circular_buffer pitch_buffer_cos{orientation_buffer_size};
  circular_buffer yaw_buffer_cos{orientation_buffer_size};

  while (ros_node.ok())

  {
    //reset all variables used in the averaging process
    current_x = 0, current_y = 0, current_z = 0;

    current_roll_sin = 0;
    current_pitch_sin = 0;
    current_yaw_sin = 0;

    current_roll_cos = 0;
    current_pitch_cos = 0;
    current_yaw_cos = 0;

    tf::StampedTransform tag_transform;
    int visible_tags = tag_array_size; //if tf is not recent enough, 1 will be subtracted

    //broadcast tag transforms to world one by one
    for (int i = 0; i < tag_array_size; i++)
    {
      try
      {
        //read latest position of each marker used in relation to camera
        listener.lookupTransform(markers_used[i], "usb_cam", ros::Time(0), tag_transforms[i]);
        //if broadcast is recent enough, use the marker to calculate cam pose
        if (ros::Time::now() - tag_transforms[i].stamp_ < ros::Duration(1))
        {
          broadcastTransform(tag_positions[i], tag_orientations[i], markers_used[i]);                                     //broadcast tf between world and specific tag
          listener.waitForTransform(markers_used[i], "world", ros::Time::now() - ros::Duration(0.1), ros::Duration(0.5)); //wait for the broadcast to kick in
          listener.lookupTransform("world", "sim_cam", ros::Time(0), cam_transforms[i]);                                  //the transform broadcasted previously now connects world to sim_cam
          current_x += cam_transforms[i].getOrigin().x();
          current_y += cam_transforms[i].getOrigin().y();
          current_z += cam_transforms[i].getOrigin().z();

          tf::Quaternion tf_orientation = (cam_transforms[i].getRotation());

          ignition::math::Quaternion<double> orientation(tf_orientation.w(), tf_orientation.x(), tf_orientation.y(), tf_orientation.z());

          roll_buffer_sin.push_back(sin(orientation.Roll()));
          pitch_buffer_sin.push_back(sin(orientation.Pitch()));
          yaw_buffer_sin.push_back(sin(orientation.Yaw()));
          roll_buffer_cos.push_back(cos(orientation.Roll()));
          pitch_buffer_cos.push_back(cos(orientation.Pitch()));
          yaw_buffer_cos.push_back(cos(orientation.Yaw()));
        }
        else
        {
          ROS_INFO_STREAM("Not detected: " + markers_used[i]);
          visible_tags--;
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }
    double alpha = 0.06; //for calculating running exp average, higher alpha discounts older observations faster
    //if there were tags, send data
    if (visible_tags > 0)
    {
      //avg all buffer values, recent value counts more
      for (int a = 0; a < roll_buffer_sin.size(); a++)
      {
        running_roll_sin = (alpha * roll_buffer_sin[a]) + (1.0 - alpha) * running_roll_sin;
        running_pitch_sin = (alpha * pitch_buffer_sin[a]) + (1.0 - alpha) * running_pitch_sin;
        running_yaw_sin = (alpha * yaw_buffer_sin[a]) + (1.0 - alpha) * running_yaw_sin;
        current_roll_sin += running_roll_sin;
        current_pitch_sin += running_pitch_sin;
        current_yaw_sin += running_yaw_sin;

        running_roll_cos = (alpha * roll_buffer_cos[a]) + (1.0 - alpha) * running_roll_cos;
        running_pitch_cos = (alpha * pitch_buffer_cos[a]) + (1.0 - alpha) * running_pitch_cos;
        running_yaw_cos = (alpha * yaw_buffer_cos[a]) + (1.0 - alpha) * running_yaw_cos;
        current_roll_cos += running_roll_cos;
        current_pitch_cos += running_pitch_cos;
        current_yaw_cos += running_yaw_cos;
      }

      //average position values of all tags and average with past data
      running_x = (alpha * current_x / visible_tags) + (1.0 - alpha) * running_x;
      running_y = (alpha * current_y / visible_tags) + (1.0 - alpha) * running_y;
      running_z = (alpha * current_z / visible_tags) + (1.0 - alpha) * running_z;

      //find camera orientation
      current_roll = atan2(current_roll_sin, current_roll_cos);
      current_pitch = atan2(current_pitch_sin, current_pitch_cos);
      current_yaw = atan2(current_yaw_sin, current_yaw_cos);

      //send data to gazebo
      ignition::math::Vector3<double> cam_position(running_x, running_y, running_z);
      ignition::math::Quaternion<double> cam_orientation(current_roll, current_pitch, current_yaw);
      ROS_INFO_STREAM("Number of visible tags " + visible_tags);
      cam_orientation.Normalize();
      ignition::math::Pose3<double> intermed(cam_position, cam_orientation);

      gazebo::msgs::Pose cam_pose;
      cam_pose = gazebo::msgs::Convert(intermed);
      gzCamPub->Publish(cam_pose);
    }
    rate.sleep();

    ros::spinOnce();
  }
  return 0;
};
