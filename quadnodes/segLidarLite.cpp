#include<ros/ros.h>
#include <ros/console.h>
#include<sensor_msgs/LaserScan.h>
#include<iomanip> // For std::setprecision and std::fixed


sensor_msgs::LaserScan lidarValues;
bool lidar_cb_flag = true;

// a callback function. Executed each time a new pose message arrives
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& lidarValuesMsg){
  //ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "distance: = " << msg.ranges[0]);
  lidarValues = *lidarValuesMsg;
  lidar_cb_flag = false;
}

int main (int argc, char **argv){

  // Initialize the ROS system and become a node.
  ros::init( argc, argv,"lidarTest");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Subscriber sub = nh.subscribe("test", 1000, &lidar_cb);

  ros::Rate rate(50.0);

  while(ros::ok()){
    ros::spinOnce();
    while (lidar_cb_flag) {
      ros::spinOnce();
      if (!lidar_cb_flag){
        break;
      }
    }

    ROS_INFO("Distance:", lidarValues.ranges[1]);
    //ROS_INFO("Throttle: %f     Roll: %f        Pitch: %f      Yaw: %f ",joyStickValues[0],-joyStickValues[1],joyStickValues[2],joyStickValues[3]);
    rate.sleep();
  }

  return 0;
}
