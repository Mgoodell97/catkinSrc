#include<ros/ros.h>
#include<turtlesim/Pose.h>
#include<iomanip> // For std::setprecision and std::fixed


// a callback function. Executed each time a new pose message arrives
void poseMessageRecieved(const turtlesim::Pose&msg){
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "Position: = ("<< msg.x << "," << msg.y <<")"
		<< "direction = " << msg.theta);
}


int main (int argc, char **argv){

  // Initialize the ROS system and become a node.
  ros::init( argc, argv,"sub_to_pose");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000,&poseMessageRecieved);

  // Let ROS take over
  ros::spin();
}
