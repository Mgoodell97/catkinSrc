#include <ros/ros.h>

int main(int argc, char **argv){
  // initalize the ROS system
  ros::init(argc, argv, "hello_ros");

  // establish this program as a ROS Node
  ros::NodeHandle nh;

  // Send some output as a log message
  while (ros::ok ()){
  ROS_INFO_STREAM("Hello, ROS!\n");
}

}
