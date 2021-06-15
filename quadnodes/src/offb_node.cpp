#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

// create a simple callback which will save the current state of the autopilot. This will allow us to check connection, arming and Offboard flags
void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  //We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10, state_cb);

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  //The px4 flight stack has a timeout of 500ms between two Offboard commands
  ros::Rate rate(20.0);

  //Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot.
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  //Desired xyz locations
  //Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, MAVROS translates these coordinates to the standard ENU frame and vice-versa. This is why we set z to positive 2.
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  //send a few setpoints before starting
  //Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected. Here, 100 was chosen as an arbitrary amount.
  for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
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
  while(ros::ok()){
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

    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;

}
