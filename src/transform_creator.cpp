#include "ros/ros.h"

#include <tf/transform_broadcaster.h>

#include "geometry_msgs/PoseStamped.h"

#include "std_msgs/String.h"

#include <sstream>

//void (const std_msgs::String::ConstPtr& msg)
void obj1_callback(const geometry_msgs::PoseStamped& pose_robo1)
{
  static tf::TransformBroadcaster obs1_br;
  tf::Transform transform;
//  transform.position = pose_robo1.Pose.position;
 
  float aa;
  std::cout << "hello you" << aa << "me too \n";
//aa = pose_robo1->pose.position.x;
//transform.setOrigin(pose_robo1->pose.position);
//transform.orientation = pose_robo1.Pose.orientation;
//obs1_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link_obs1"));

static tf::TransformBroadcaster mocap_br;
transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
transform.setRotation(tf::Quaternion(1.0,0.0,0.0,0.0));
mocap_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "mocap_world"));

  ROS_INFO("I heard something");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_creater");

  ros::NodeHandle n;

  ros::Publisher tf_world_pub = n.advertise<std_msgs::String>("tf_world", 1000);

  ros::Subscriber sub = n.subscribe("/object_1/pose", 10, &obj1_callback);

  static tf::TransformBroadcaster obs2_br;

  static tf::TransformBroadcaster word_mocap_br;

  //ros::Publisher tf_obs1_pub = n.advertise<std_msgs::String>("tf", 1000);
  //os::Publisher tf_obs2_pub = n.advertise<std_msgs::String>("tf", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Message count - " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    tf_world_pub.publish(msg);

    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "mocap_world"));
    
    ros::spinOnce();
    loop_rate.sleep(); // constant speed
    ++count;  // increment coutner

  }


  return 0;
}