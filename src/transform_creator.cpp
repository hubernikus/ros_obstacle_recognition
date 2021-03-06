#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include <sstream>

#include <iostream>

const float filtCoeff = 0.999;
bool firstTFrecieved = false;
//tf::PoseStamped pose_filtered;
geometry_msgs::Pose pose_filtered;


//void (const std_msgs::String::ConstPtr& msg)
void obj1_callback(const geometry_msgs::PoseStamped& pose_robo1){
  // pose_filtered.position.x = pose_filtered.position.x*filtCoeff + (1.-filtCoeff)*transform.pose_robo1.position.x;
  //obs1_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link_obs1"));

  static tf::TransformBroadcaster mocap_br;
  tf::Quaternion quat;
  tf::Transform transform;

  float k;
  if(firstTFrecieved){
    k = filtCoeff;
  }
  else{
    k=0;
    firstTFrecieved = true;
  }
  
  // Filter position
  pose_filtered.position.x = pose_filtered.position.x*k + (1.-k)*pose_robo1.pose.position.x;
  pose_filtered.position.y = pose_filtered.position.y*k + (1.-k)*pose_robo1.pose.position.y;
  pose_filtered.position.z = pose_filtered.position.z*k + (1.-k)*pose_robo1.pose.position.z;

  transform.setOrigin(tf::Vector3(pose_filtered.position.x,
				  pose_filtered.position.y,
				  pose_filtered.position.z
				   ) );
  // Filter quaternion
  pose_filtered.orientation.x = pose_filtered.orientation.x*k + (1.-k)*pose_robo1.pose.orientation.x;
  pose_filtered.orientation.y = pose_filtered.orientation.y*k + (1.-k)*pose_robo1.pose.orientation.y;
  pose_filtered.orientation.z = pose_filtered.orientation.z*k + (1.-k)*pose_robo1.pose.orientation.z;
  pose_filtered.orientation.w = pose_filtered.orientation.w*k + (1.-k)*pose_robo1.pose.orientation.w;

  quat = tf::Quaternion(pose_filtered.orientation.x,
			pose_filtered.orientation.y,
			pose_filtered.orientation.z,
			pose_filtered.orientation.w);

  transform.setRotation(quat);
  
  // TODO change to real time stamp
  //mocap_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "mocap_world","world"));
  mocap_br.sendTransform(tf::StampedTransform(transform, pose_robo1.header.stamp, "mocap_world","world"));
  //mocap_br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "mocap_world","world")); // wrong is zero time

  ROS_INFO("MOCAP message recieved.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_creater");

  ros::NodeHandle n;

  //ros::Subscriber sub = n.subscribe("/object_1/pose", 10, &obj1_callback);
  ros::Subscriber sub = n.subscribe("/robot/pose", 10, &obj1_callback);
  static tf::TransformBroadcaster word_mocap_br;

  ros::Rate loop_rate(1);

  ros::spin();

  int count = 0;
  while (ros::ok())
  {
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "mocap_world"));
    ros::spinOnce();

    std::cout << "spin \n";

    loop_rate.sleep(); // constant speed

    ++count;  // increment coutner

  }

  return 0;
}
