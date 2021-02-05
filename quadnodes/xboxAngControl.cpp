#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>

mavros_msgs::State current_state;
sensor_msgs::Joy joyStickValues;

bool controller_cb_flag = false;
bool state_cb_flag = false;

void state_cb(const mavros_msgs::State::ConstPtr& state_sub_msg){
  current_state = *state_sub_msg;
  state_cb_flag = true;
}

// callback function
void controller_cb(const sensor_msgs::Joy::ConstPtr& joyStickValuesMsg){
  joyStickValues = *joyStickValuesMsg;
  controller_cb_flag = true;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "xboxVelControl");
  ros::NodeHandle nh;

  //We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
  ros::Subscriber controller_sub = nh.subscribe<sensor_msgs::Joy>("joy",1000, &controller_cb);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",1000, state_cb);

  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",1000);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  //The px4 flight stack has a timeout of 500ms between two Offboard commands
  ros::Rate rate(30.0);

  //Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot.
  while((ros::ok() && !current_state.connected) || !controller_cb_flag || !state_cb_flag){
    ros::spinOnce();
    rate.sleep();
    if (controller_cb_flag && state_cb_flag){
      break;
    }
  }

  //ROS_INFO("Throttle: %f     Roll: %f        Pitch: %f      Yaw: %f ",joyStickValues.axes[1],-joyStickValues.axes[3],joyStickValues.axes[4],joyStickValues.axes[0]);
  //Desired xyz locations
  //Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, MAVROS translates these coordinates to the standard ENU frame and vice-versa. This is why we set z to positive 2.
  geometry_msgs::TwistStamped DesiredVel;
  DesiredVel.twist.linear.z = joyStickValues.axes[1];
  DesiredVel.twist.angular.x = joyStickValues.axes[4];
  DesiredVel.twist.angular.y = joyStickValues.axes[3];
  DesiredVel.twist.angular.z = joyStickValues.axes[0];

  //send a few setpoints before starting
  //Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected. Here, 100 was chosen as an arbitrary amount.
  for(int i = 100; ros::ok() && i > 0; --i){
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

  //The rest of the code is pretty self explanatory. We attempt to switch to Offboard mode,
  //after which we arm the quad to allow it to fly.
  //We space out the service calls by 5 seconds so to not flood the autopilot with the requests.
  //In the same loop, we continue sending the requested pose at the appropriate rate
  int scaleFactor = 1;

  while(ros::ok()){
    DesiredVel.twist.linear.z = scaleFactor*joyStickValues.axes[1];
    DesiredVel.twist.angular.x = scaleFactor*joyStickValues.axes[4];
    DesiredVel.twist.angular.y = scaleFactor*joyStickValues.axes[3];
    DesiredVel.twist.angular.z = scaleFactor*joyStickValues.axes[0];

    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
      if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    local_vel_pub.publish(DesiredVel);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;

}
