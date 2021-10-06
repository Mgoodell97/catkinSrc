#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadnodes/gaussian.h>

#include <math.h>
#include <iostream>

// ##################
// # Functions
// ##################

float gaussFunc(float xFunc, float yFunc, float zFunc, float QFunc, float vFunc, float DyFunc, float DzFunc) {
	// returns concentration at x,y,z cordinates in plume frame
	return (QFunc/(4 * M_PI * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * (pow(yFunc,2)/DyFunc + pow(zFunc,2)/DzFunc))* 1000;
}

float getReading(float xRobotDef, float yRobotDef, float thetaFunc, float xPlumeFunc, float yPlumeFunc, float zFunc, float QFunc, float vFunc, float DyFunc, float DzFunc) {
	float Stheta = sin(thetaFunc);
	float Ctheta = cos(thetaFunc);

	float XplumeFrame = (Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc);
	float YplumeFrame = (-Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc);

	float reading;

	if (XplumeFrame <= 0){
		reading = 0;
	}
	else{
		reading = gaussFunc(XplumeFrame, YplumeFrame, zFunc, QFunc, vFunc, DyFunc, DzFunc);
	}

	return reading;
}

// ##################
// # Global variables
// ##################

geometry_msgs::PoseStamped robot_pose;
bool pose_cb_flag = true;

float QPlume;
float vPlume;
float DyPlume;
float DzPlume;

float xPlume;
float yPlume;
float zPlume;
float thetaPlume;

float sensorRate;

bool careAboutZ;

// ##################
// # Callbacks
// ##################

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& robot_pose_tmp){
  robot_pose = *robot_pose_tmp;
  pose_cb_flag = false;
}

// ##################
// # Main function
// ##################

int main(int argc, char **argv){
  ros::init(argc, argv, "gaussianSensor");
  ros::NodeHandle nh;

  ros::Subscriber controller_sub = nh.subscribe<geometry_msgs::PoseStamped>("mocap_node/Robot_1/pose",1, pose_cb);

	ros::Publisher pub = nh.advertise<quadnodes::gaussian>("gaussianReading",10);

  ros::param::get("/QPlume", QPlume);
  ros::param::get("/vPlume", vPlume);
  ros::param::get("/DyPlume", DyPlume);
  ros::param::get("/DzPlume", DzPlume);

  ros::param::get("/xPlume", xPlume);
  ros::param::get("/yPlume", yPlume);
	ros::param::get("/zPlume", zPlume);
  ros::param::get("/thetaPlume", thetaPlume);
	ros::param::get("/careAboutZ", careAboutZ);

	ros::param::get("/sensorRate", sensorRate);

  ros::Rate rate(sensorRate);

	quadnodes::gaussian gaussianReadingMsg;

  while(ros::ok()){
    ros::spinOnce();
    while (pose_cb_flag) {
      ros::spinOnce();
      if (!pose_cb_flag){
        break;
      }
    }

  // Get reading and publish
	if (!careAboutZ){  // Dont care about z height (2D)
  	// ROS_INFO("Robot 1 reading : %f \n", getReading(robot_pose.pose.position.x,robot_pose.pose.position.y, thetaPlume, xPlume, yPlume, 0.0, QPlume, vPlume, DyPlume, DzPlume));
		gaussianReadingMsg.ppm = getReading(robot_pose.pose.position.x,robot_pose.pose.position.y, thetaPlume, xPlume, yPlume, 0.0, QPlume, vPlume, DyPlume, DzPlume);
	}
	else{
		// ROS_INFO("Robot 1 reading : %f \n", getReading(robot_pose.pose.position.x,robot_pose.pose.position.y, thetaPlume, xPlume, yPlume, robot_pose.pose.position.z - zPlume, QPlume, vPlume, DyPlume, DzPlume));
		gaussianReadingMsg.ppm = getReading(robot_pose.pose.position.x,robot_pose.pose.position.y, thetaPlume, xPlume, yPlume, robot_pose.pose.position.z - zPlume, QPlume, vPlume, DyPlume, DzPlume);
	}

	pub.publish(gaussianReadingMsg);

  rate.sleep();
  }

  return 0;

}
