#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <math.h>       /* sqrt */
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std;

int yawAbsCount;
int lastFrame;

double desiredYaw[4] = { 0 , M_PI/2 , M_PI , 3*M_PI/2 };
// double desiredYaw[2] = { 0, 0 };
double absYaw;

bool state_cb_flag = false;
bool pose_cb_flag = false;
bool yaw_boot_up_frame_flag = false;

mavros_msgs::State current_state;

geometry_msgs::PoseStamped current_pose;

double absYawOrientation(double currentYaw){

  int frameChange;
  int yawFrame;
  int currentFrame;

  double trueYaw;

  if ((currentYaw < M_PI/2) && (currentYaw > 0)){
    yawFrame = 1; // yaw goes from -pi to pi
    trueYaw = currentYaw;
  } else if ((currentYaw < M_PI) && (currentYaw > M_PI/2)){ //frame 2 and 3 are needed so false detections arn't counted when moving from pi to -pi
  yawFrame = 2; // yaw goes from -pi to pi
  trueYaw = currentYaw;
} else if ((currentYaw > -M_PI) && (currentYaw < -M_PI/2)){
  yawFrame = 3; // yaw goes from 0 to 2*pi
  trueYaw = currentYaw + 2*M_PI;
} else{
  yawFrame = 4; // yaw goes from 0 to 2*pi
  trueYaw = currentYaw + 2*M_PI;
}

if(!yaw_boot_up_frame_flag){
  lastFrame = yawFrame;
  if ((yawFrame == 1) || (yawFrame == 2)){
    yawAbsCount = 0;
  }
  if ((yawFrame == 3 || yawFrame == 4)){
    yawAbsCount = -1;
    // ROS_INFO("%s", yaw_boot_up_frame_flag ? "true" : "false");
  }
  yaw_boot_up_frame_flag = true;
}

currentFrame = yawFrame;
frameChange = currentFrame - lastFrame;

if (frameChange == -3){
  yawAbsCount++;
}
if (frameChange == 3){
  yawAbsCount--;
}

lastFrame = currentFrame;

trueYaw = trueYaw + yawAbsCount*2*M_PI;

return trueYaw;
}

double capFunc(double currentValue, double newMin, double newMax){
  if (currentValue > newMax){
    currentValue = newMax;
  }
  if (currentValue < newMin){
    currentValue = newMin;
  }
  return currentValue;
}

std::vector<double> matrix2Vec(std::vector <std::vector<double>> matrix){

  int szRow = matrix.size();
  int szCol = matrix[0].size();
  int num = szRow*szCol;
  vector<double> vec(num, 1);

  int count = 0;
  for (int i = 0; i < szRow; i++) {
    for (int j = 0; j < szCol; j++){
      vec[count] = matrix[i][j];
      count++;
    }
  }

  return vec;
}

std::vector<double> linearSpacedArray(double a, double b, std::size_t N){
  double h = (b - a) / static_cast<double>(N-1);
  std::vector<double> xs(N);
  std::vector<double>::iterator x;
  double val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
    *x = val;
  }
  return xs;
}

std::vector <std::vector<double> > createWaypointMatrix(int xmin, int xmax, int Nx, int ymin, int ymax, int Ny,int zmin, int zmax, int Nz){
  std::vector<double> xgrid = linearSpacedArray(xmin, xmax, Nx);
  std::vector<double> ygrid = linearSpacedArray(ymin, ymax, Ny);
  std::vector<double> zgrid = linearSpacedArray(zmin, zmax, Nz);

  std::vector <std::vector<double> > xMatrixTemp(Ny, std::vector<double>(Nx, 0));
  std::vector <std::vector<double> > yMatrixTemp(Ny, std::vector<double>(Nx, 0));

  std::vector<double> xgridReverse;
  std::vector<double> ygridReverse;

  xgridReverse = xgrid;
  ygridReverse = ygrid;

  std::reverse(xgridReverse.begin(), xgridReverse.end());
  std::reverse(ygridReverse.begin(), ygridReverse.end());

  vector<double> yOnesvVec(xgrid.size(), 1);
  for(int i=0; i < ygrid.size(); i++){
    if (i%2 == 0){
      xMatrixTemp[i] = xgrid;
    }else { // when odd reverse direction
      xMatrixTemp[i] = xgridReverse;
    }
    for (int j=0; j < xgrid.size(); j++){
      yMatrixTemp[i][j] = ygrid[i];
    }
  }

  std::vector<double> xvecTemp = matrix2Vec(xMatrixTemp);
  std::vector<double> yvecTemp = matrix2Vec(yMatrixTemp);

  std::vector<double> xvecTempReverse;
  std::vector<double> yvecTempReverse;

  xvecTempReverse = xvecTemp;
  yvecTempReverse = yvecTemp;

  std::reverse(xvecTempReverse.begin(), xvecTempReverse.end());
  std::reverse(yvecTempReverse.begin(), yvecTempReverse.end());

  std::vector <std::vector<double> > xScanMatrix(Nz, std::vector<double>(xvecTemp.size(), 0));
  std::vector <std::vector<double> > yScanMatrix(Nz, std::vector<double>(xvecTemp.size(), 0));
  std::vector <std::vector<double> > zScanMatrix(Nz, std::vector<double>(xvecTemp.size(), 0));

  for(int i=0; i < zgrid.size(); i++){
    if (i%2 == 0){
      xScanMatrix[i] = xvecTemp;
      yScanMatrix[i] = yvecTemp;
    }else { // when odd reverse direction
      xScanMatrix[i] = xvecTempReverse;
      yScanMatrix[i] = yvecTempReverse;
    }
    for (int j=0; j < xvecTemp.size(); j++){
      zScanMatrix[i][j] = zgrid[i];
    }
  }


  std::vector<double> xvec = matrix2Vec(xScanMatrix);
  std::vector<double> yvec = matrix2Vec(yScanMatrix);
  std::vector<double> zvec = matrix2Vec(zScanMatrix);

  std::vector <std::vector<double> > waypointList(3, std::vector<double>(zvec.size(), 0));

  waypointList[0] = xvec;
  waypointList[1] = yvec;
  waypointList[2] = zvec;
  return waypointList;
}

// callback functions
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& poseMsg){
  current_pose = *poseMsg;

  tf::Quaternion current_quat(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
  tf::Matrix3x3 current_matrix(current_quat);

  double roll, pitch, yaw;
  current_matrix.getRPY(roll, pitch, yaw);
  absYaw = absYawOrientation(yaw);
  pose_cb_flag = true;
}

void state_cb(const mavros_msgs::State::ConstPtr& stateSubMsg){
  current_state = *stateSubMsg;
  state_cb_flag = true;
}

int main(int argc, char **argv){
  int xyzWaypointIndex = 0;
  int yawWaypointIndex = 0;

  double prop = 1;
  double xyzError[3];
  double yawError;
  double withinWaypoint;
  double withinWaypointYaw;
  double waypointRadius = 0.5;
  double stayTime = 5;
  double waypointRadiusYaw = 0.1;
  double stayTimeYaw = 1;

  bool justHitWaypoint = false;
  bool justHitWaypointYaw = false;
  bool yawWaypointLoopDone = false;

  ros::Time waypointStartTime;
  ros::Time waypointStartTimeYaw;

  ros::init(argc, argv, "zScan");
  ros::NodeHandle nh;

  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",100, &pose_cb);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",100, state_cb);

  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",100);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  //The px4 flight stack has a timeout of 500ms between two Offboard commands
  ros::Rate rate(50.0);

  // createWaypointMatrix(int xmin, int xmax, int Nx, int ymin, int ymax, int Ny,int zmin, int zmax, int Nz)
  std::vector <std::vector<double> > desiredPose = createWaypointMatrix(-2,2,3,-2,2,3,1,3,2);

  //Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot.
  while((ros::ok() && !current_state.connected) || !pose_cb_flag || !state_cb_flag){
    ros::spinOnce();
    rate.sleep();
    if (pose_cb_flag && state_cb_flag){
      break;
    }
  }

  //Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, MAVROS translates these coordinates to the standard ENU frame and vice-versa. This is why we set z to positive 2.
  // 1 = yaw left
  geometry_msgs::TwistStamped DesiredVel;
  DesiredVel.twist.linear.x = 0;
  DesiredVel.twist.linear.y = 0;
  DesiredVel.twist.linear.z = 0;
  DesiredVel.twist.angular.z = 0; // yaw

  //send a few setpoints before starting
  //Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected. Here, 100 was chosen as an arbitrary amount.
  for(int i = 50; ros::ok() && i > 0; --i){
    local_vel_pub.publish(DesiredVel);
    ros::spinOnce();
    rate.sleep();
  }

  //set the custom mode to OFFBOARD
  // more modes can be found here http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while(ros::ok()){

    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0))){
      if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))){
        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    if (current_state.armed){

      xyzError[0] = desiredPose[0][xyzWaypointIndex] - current_pose.pose.position.x;
      xyzError[1] = desiredPose[1][xyzWaypointIndex] - current_pose.pose.position.y;
      xyzError[2] = desiredPose[2][xyzWaypointIndex] - current_pose.pose.position.z;
      yawError = desiredYaw[yawWaypointIndex] - absYaw;

      withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2));
      withinWaypointYaw = abs(yawError);

      if(withinWaypoint <= waypointRadius){
        // ROS_INFO_THROTTLE(2,"Hit waypoint");
        if(!justHitWaypoint){
          // ROS_INFO_THROTTLE(2,"Hit waypoint");
          waypointStartTime = ros::Time::now();
          justHitWaypoint = true;
        }
        if(ros::Time::now() - waypointStartTime >= ros::Duration(stayTime) && yawWaypointLoopDone){
          // ROS_INFO("Moving to next waypoint");
          xyzWaypointIndex++;
          // ROS_INFO_THROTTLE(2,"Waypoint: %i\n",xyzWaypointIndex);
          if (xyzWaypointIndex == desiredPose[0].size()){
            xyzWaypointIndex = 0;
          }
          justHitWaypoint = false;
          yawWaypointLoopDone = false;
        }
      } else {
        // ROS_INFO_THROTTLE(1,"Moving to x: %f y: %f z: %f\n",desiredPose[0][xyzWaypointIndex],desiredPose[1][xyzWaypointIndex],desiredPose[2][xyzWaypointIndex]);
        justHitWaypoint = false;
        yawWaypointLoopDone = false;
      }

      if(withinWaypointYaw <= waypointRadiusYaw && justHitWaypoint){
        // ROS_INFO_THROTTLE(2,"Hit yaw waypoint");
        if(!justHitWaypointYaw){
          waypointStartTimeYaw = ros::Time::now();
          justHitWaypointYaw = true;
        }
        if(ros::Time::now() - waypointStartTimeYaw >= ros::Duration(stayTimeYaw)){
          // ROS_INFO("Moving to next waypoint");
          yawWaypointIndex++;
          if (yawWaypointIndex == 4){
            yawWaypointIndex = 0;
            yawAbsCount--;
            yawWaypointLoopDone = true;
          }
          justHitWaypointYaw = false;
        }
      } else {
        // ROS_INFO_THROTTLE(1,"Moving to yaw waypoint %f",desiredYaw[yawWaypointIndex]);
        justHitWaypointYaw = false;
      }

      //capFunc( currentValue,  newMin,  newMax)
      DesiredVel.twist.linear.x = capFunc( prop * xyzError[0],  -1,  1);
      DesiredVel.twist.linear.y = capFunc( prop * xyzError[1],  -1,  1);
      DesiredVel.twist.linear.z = capFunc( prop * xyzError[2],  -1,  1);
      DesiredVel.twist.angular.z = capFunc( prop * yawError,  -1,  1);

    }

    local_vel_pub.publish(DesiredVel);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;

}
