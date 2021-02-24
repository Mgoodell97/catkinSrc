#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>


nav_msgs::Odometry quadPose;
geometry_msgs::PoseStamped quadPoseWorld;
std::string UAV_Name;

bool quadPose_cb_flag = true;
float xOffset;
float yOffset;
float zOffset;



// callback function
void quadPose_cb(const nav_msgs::Odometry::ConstPtr& quadPoseValuesMsg){
  quadPose = *quadPoseValuesMsg;
  quadPose_cb_flag = false;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "worldUAVs";
  transformStamped.child_frame_id = UAV_Name;
  transformStamped.transform.translation.x = quadPose.pose.pose.position.x + xOffset;
  transformStamped.transform.translation.y = quadPose.pose.pose.position.y + yOffset;
  transformStamped.transform.translation.z = quadPose.pose.pose.position.z + zOffset;

  transformStamped.transform.rotation.x = quadPose.pose.pose.orientation.x;
  transformStamped.transform.rotation.y = quadPose.pose.pose.orientation.y;
  transformStamped.transform.rotation.z = quadPose.pose.pose.orientation.z;
  transformStamped.transform.rotation.w = quadPose.pose.pose.orientation.w;

  br.sendTransform(transformStamped);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "localToWorldTf");
  ros::NodeHandle nh;

  ros::Subscriber quadPose_sub = nh.subscribe<nav_msgs::Odometry>("mavros/global_position/local",1000, quadPose_cb);
  ros::Publisher pubPose = nh.advertise<geometry_msgs::PoseStamped>("true_position",1000);

  ros::Rate rate(50.0);

  // Get parameters
  ros::param::get("tfWorldUAV/x_offset", xOffset);
  ros::param::get("tfWorldUAV/y_offset", yOffset);
  ros::param::get("tfWorldUAV/z_offset", zOffset);
  ros::param::get("tfWorldUAV/UAV_Name", UAV_Name);

  while(ros::ok()){
    ros::spinOnce();
    while (quadPose_cb_flag) {
      ros::spinOnce();
      if (!quadPose_cb_flag){
        break;
      }
    }

    // Odometry and tf data
    quadPoseWorld.header.frame_id = "worldUAVs";
    quadPoseWorld.pose.position.x = quadPose.pose.pose.position.x + xOffset;
    quadPoseWorld.pose.position.y = quadPose.pose.pose.position.y + yOffset;
    quadPoseWorld.pose.position.z = quadPose.pose.pose.position.z + zOffset;

    quadPoseWorld.pose.orientation.x = quadPose.pose.pose.orientation.x;
    quadPoseWorld.pose.orientation.y = quadPose.pose.pose.orientation.y;
    quadPoseWorld.pose.orientation.z = quadPose.pose.pose.orientation.z;
    quadPoseWorld.pose.orientation.w = quadPose.pose.pose.orientation.w;

    pubPose.publish(quadPoseWorld);

    // ROS_INFO("-------------------------");
    // ROS_INFO(" ");
    // ROS_INFO("X: %f     Y: %f        Z: %f",xOffset,yOffset,zOffset);
    // ROS_INFO("X: %f     Y: %f        Z: %f",quadPoseWorldFrame.pose.pose.position.x,quadPoseWorldFrame.pose.pose.position.y,quadPoseWorldFrame.pose.pose.position.z);

    rate.sleep();
  }

  return 0;

}
