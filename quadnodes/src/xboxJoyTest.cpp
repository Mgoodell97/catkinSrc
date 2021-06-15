#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

sensor_msgs::Joy joyStickValues;
bool controller_cb_flag = true;


// callback function
void controller_cb(const sensor_msgs::Joy::ConstPtr& joyStickValuesMsg){
  joyStickValues = *joyStickValuesMsg;
  controller_cb_flag = false;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "xboxJoyTest");
  ros::NodeHandle nh;

  ros::Subscriber controller_sub = nh.subscribe<sensor_msgs::Joy>("joy",1000, controller_cb);

  ros::Rate rate(20.0);

  while(ros::ok()){
    ros::spinOnce();
    while (controller_cb_flag) {
      ros::spinOnce();
      if (!controller_cb_flag){
        break;
      }
    }

    ROS_INFO("Throttle: %f     Roll: %f        Pitch: %f      Yaw: %f ",joyStickValues.axes[1],-joyStickValues.axes[3],joyStickValues.axes[4],joyStickValues.axes[0]);
    //ROS_INFO("Throttle: %f     Roll: %f        Pitch: %f      Yaw: %f ",joyStickValues[0],-joyStickValues[1],joyStickValues[2],joyStickValues[3]);
    rate.sleep();
  }

  return 0;

}
