#include<ros/ros.h>

// The srv class for the service
#include<turtlesim/Spawn.h>


int main (int argc, char **argv){

  // Initialize the ROS system and become a node.
  ros::init( argc, argv,"spawn_turtle");

  ros::NodeHandle nh;

  ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");

  turtlesim::Spawn::Request req;
  turtlesim::Spawn::Response resp;

  // Fill in the request data members.
  req.x = 2;
  req.y = 3;
  req.theta = M_PI/2;
  req.name = "Leo";

  // Actually call the service. This won't return until the service is complete
  bool success = spawnClient.call(req,resp);

  if(success){
    ROS_INFO_STREAM("Spawned a turtle named " << resp.name);
  }else{
    ROS_ERROR_STREAM("Failed to spawn");
  }
}
