#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <math.h>
#include <time.h>
#include <ctime>
#include <unistd.h>
#include <typeinfo>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "serial/serial.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <mavros_msgs/HomePosition.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8.h"
#include "enif_iuc/Waypoint.h"
#include "enif_iuc/WaypointTask.h"
#include "mps_driver/MPS.h"
#include <string>
#include <geographic_msgs/GeoPoint.h>
#include "enif_iuc/AgentHome.h"
#include "enif_iuc/AgentSource.h"

using namespace std;

#define COMMAND_TAKEOFF  1
#define COMMAND_WAYPOINT 2
#define COMMAND_GPS      3
#define COMMAND_MPS      4
#define COMMAND_STATE    5
#define COMMAND_BATTERY  6
#define COMMAND_BOX      7
#define COMMAND_HOME     8
#define COMMAND_LOCAL    9
#define COMMAND_AVEHOME  10
#define COMMAND_TARGETE    11
#define COMMAND_REALTARGET 12

#define GAS_NONE    0
#define GAS_PROPANE 1
#define GAS_METHANE 2

#define REMAP_A 0.5
#define REMAP_B -990.0

// transmit local and home info from quad
bool sendLocal = false;
bool sendBat   = false;
bool sendHome  = true;
bool sendRealTarget = true;
bool sendTargetE = true;

std::string USB;
int AGENT_NUMBER = 0;
int GS_ID = 0;
int GAS_ID = 0;

std_msgs::UInt8 state;
sensor_msgs::NavSatFix gps;
mps_driver::MPS mps, mps_other;
sensor_msgs::Range height,height_other;
sensor_msgs::BatteryState battery;

mavros_msgs::HomePosition home;
nav_msgs::Odometry local;

geographic_msgs::GeoPoint targetE;
enif_iuc::AgentSource realTarget;
enif_iuc::AgentHome agent_home;

//containers for enif_iuc_ground
std::vector<geographic_msgs::GeoPoint> agentHomes;
std::vector<int> agentID;
geographic_msgs::GeoPoint averageHome;

std_msgs::Bool takeoff_command;
std_msgs::Int8 alg;
enif_iuc::WaypointTask waypoint_list;
std_msgs::Float64MultiArray box;

bool check_double(double number)
{
  unsigned char *chptr;
  chptr = (unsigned char *) &number;
  for(int i = 0; i<sizeof(double); i++){
    if(*chptr == 0xd0 || *chptr == 0xda)
      return true;// return true if we hit the hole
    chptr++;
  }
  return false;
}

bool checkValue(double var, double min, double max)
{
  if(var>=min && var<=max)
    return true;
  return false;
}

int CharToInt(char a)
{
  return (int)(a - '0');
}

char IntToChar(int a)
{
  return (char)(a + '0');
}

void DoubleToChar(char* buf, double number)
{
  unsigned char *chptr;
  bool remap = check_double(number);
  double remap_number = number;
  if(remap == true)
    remap_number = number*REMAP_A+REMAP_B;
  chptr = (unsigned char *) &remap_number;
  for(int i = 0; i<sizeof(double); i++){
    buf[i] = *chptr + '0';
    chptr++;
  }
}

void FloatToChar(char* buf, float number)
{
  unsigned char *chptr;
  chptr = (unsigned char *) &number;
  for(int i = 0; i<sizeof(float); i++){
    buf[i] = *chptr + '0';
    chptr++;
  }
}

void CharToDouble(char* buf, double &number)
{
  unsigned char *chptr;
  chptr = (unsigned char *) &number;
  for(int i = 0; i<sizeof(double); i++){
    *chptr = buf[i] - '0';
    chptr++;
  }
  if(checkValue(number, -180.0*REMAP_A+REMAP_B, 180.0*REMAP_A+REMAP_B)){
    number = (number-REMAP_B)/REMAP_A;
  }
}

void CharToFloat(char* buf, float &number)
{
  unsigned char *chptr;
  chptr = (unsigned char *) &number;
  for(int i = 0; i<sizeof(float); i++){
    *chptr = buf[i] - '0';
    chptr++;
  }
}

int get_target_number(char* buf)
{
  int number = CharToInt(buf[1]);
  return number;
}

int get_command_type(char* buf)
{
  int number = CharToInt(buf[2]);
  return number;
}

bool checksum(char* buf)
{
  int length = strlen(buf+1);
  unsigned char sum = 0;
  for(int i = 1; i < length; i++){
    sum += buf[i];
  }
  if((unsigned char)buf[0] == sum)
    return true;
  else
    return false;
}

void form_checksum(char* buf)
{
  int length = strlen(buf+1);
  unsigned char sum = 0;
  for(int i = 1; i < length; i++){
    sum += buf[i];
  }
  buf[0] = sum;
}

std_msgs::Bool get_takeoff_command(char* buf, std_msgs::Int8 &newAlg)
{
  std_msgs::Bool result;
  int takeoff_newAlg = CharToInt(buf[3]);
  if(takeoff_newAlg >= 100)
    result.data = true;
  else
    result.data = false;
  newAlg.data = takeoff_newAlg - result.data*100;
      
  return result;
}

int get_waypoint_number(char* buf)
{
  return CharToInt(buf[3]);
}

void get_waypoint_info(char* buf, enif_iuc::WaypointTask &waypoint_list)
{
  double velocity, damping_distance;
  CharToDouble(buf+4, velocity);
  waypoint_list.velocity = velocity;
  CharToDouble(buf+12, damping_distance);
  waypoint_list.damping_distance = damping_distance;
}

void get_waypoints(int waypoint_number, char* buf, enif_iuc::WaypointTask &waypoint_list)
{
  int byte_number = 0;
  for(int i=0; i<waypoint_number; i++)
    {
      enif_iuc::Waypoint waypoint;
      double latitude, longitude, target_height;
      int staytime;
      // Get latitude
      CharToDouble(buf+20+byte_number, latitude);
      waypoint.latitude = latitude;
      byte_number += sizeof(double);
      // Get longitude
      CharToDouble(buf+20+byte_number, longitude);
      waypoint.longitude = longitude;
      byte_number += sizeof(double);
      // Get waypoint height
      CharToDouble(buf+20+byte_number, target_height);
      waypoint.target_height = target_height;
      byte_number += sizeof(double);
      // Get staytime
      waypoint.staytime = CharToInt(buf[20+byte_number]);
      byte_number++;
      waypoint_list.mission_waypoint.push_back(waypoint);
    }
}

void cut_buf(char* buf_0, char* buf_1, int size)
{
  //  char tbuf[size];
  //  buf_1 = tbuf;
  strncpy(buf_1, buf_0, size-1);
  buf_1[size-1] = 0x0A;
  //buf_0 += size;
}

int get_waypointlist_buf_size(int waypoint_number)
{
  int buf_size = 0;
  //20 bytes of wp info + 25 bytes per waypoint
  buf_size = 20+25*waypoint_number;
  return buf_size;
}

void get_box(char* buf, std_msgs::Float64MultiArray &box)
{
  double latitude, longitude, width, height, angle;
  double staytime, wp_height, velocity, wp_radius;
  double stepwidth, stepheight;
  CharToDouble(buf+3,  longitude);
  CharToDouble(buf+11, latitude);
  CharToDouble(buf+19, width);
  CharToDouble(buf+27, height);
  CharToDouble(buf+35, angle);
  staytime   = CharToInt(buf[43]);
  wp_height  = CharToInt(buf[44])/20.0;
  velocity   = CharToInt(buf[45]);
  wp_radius  = CharToInt(buf[46]);
  stepwidth  = CharToInt(buf[47]);
  stepheight = CharToInt(buf[48]);
  box.data.push_back(longitude);
  box.data.push_back(latitude);
  box.data.push_back(width);
  box.data.push_back(height);
  box.data.push_back(angle);
  box.data.push_back(staytime);
  box.data.push_back(wp_height);
  box.data.push_back(velocity);
  box.data.push_back(wp_radius);
  box.data.push_back(stepwidth);
  box.data.push_back(stepheight);
}

bool extract_GPS_from_MPS(mps_driver::MPS mps_read)
{
  if(checkValue(mps_read.GPS_latitude, -180, 180) &&
     checkValue(mps_read.GPS_longitude, -180, 180) &&
     checkValue(mps_read.GPS_altitude, 0, 2000)){
    gps.latitude = mps_read.GPS_latitude;
    gps.longitude = mps_read.GPS_longitude;
    gps.altitude = mps_read.GPS_altitude;
    
    height.range = mps_read.GPS_altitude;
    return true;
  }else{
    cout<<mps_read.GPS_latitude<<" "<<mps_read.GPS_longitude<<" "<<mps_read.GPS_altitude<<endl;
  }
  return false;
}

bool check_MPS(mps_driver::MPS mps_read)
{
  if(checkValue(mps_read.GPS_latitude, -180, 180) &&
     checkValue(mps_read.GPS_longitude, -180, 180) &&
     checkValue(mps_read.GPS_altitude, 0, 2000)){
    return true;
  }else{
    cout<<mps_read.GPS_latitude<<" "<<mps_read.GPS_longitude<<" "<<mps_read.GPS_altitude<<endl;
  }
  return false;
}


bool checkGeo(geographic_msgs::GeoPoint newData, geographic_msgs::GeoPoint storedData)
{
  // check if newData is new compared with storedData
  if(newData.latitude != storedData.latitude || newData.longitude != storedData.longitude)
    {
      return true;
    }
  return false;
}
  

bool checkHome(mavros_msgs::HomePosition myhome)
{
  /* bool checkValue(double var, double min, double max) */
  if (checkValue(myhome.geo.latitude, -180, 180)  &&
      checkValue(myhome.geo.longitude, -180, 180) &&
      checkValue(myhome.geo.altitude, 0, 2000) &&
      (fabs(myhome.geo.latitude) + fabs(myhome.geo.longitude) + fabs(myhome.geo.altitude)) > 1 &&
      (fabs(myhome.geo.latitude) + fabs(myhome.geo.longitude) + fabs(myhome.geo.altitude)) < 1000000)
    {
      return true;
    }
  return false;
}

bool checkLocal(nav_msgs::Odometry mylocal)
{
  /* bool checkValue(double var, double min, double max) */
  if (checkValue(mylocal.pose.pose.position.x, -100, 100)  &&
      checkValue(mylocal.pose.pose.position.y, -100, 100) &&
      checkValue(mylocal.pose.pose.position.z, -100, 100))
    {
      return true;
    }
  return false;
}

void get_mps(char* buf)
{
  GAS_ID = CharToInt(buf[3]);
  if(GAS_ID == GAS_PROPANE){
    string str = "Propane";
    mps.gasID = str;
  }else if(GAS_ID == GAS_METHANE){
    string str = "Methane";
    mps.gasID = str;
  }else{
    string str = "None";
    mps.gasID = str;
  }
  float percentLEL, temperature, local_height, humidity;
  double GPS_latitude, GPS_longitude, GPS_altitude;
  CharToFloat(buf+4, percentLEL);
  mps.percentLEL = percentLEL;
  CharToFloat(buf+4+4, temperature);
  mps.temperature = temperature;
  CharToFloat(buf+4+8, local_height);
  mps.local_z = local_height;
  CharToFloat(buf+4+12, humidity);
  mps.humidity = humidity;
  CharToDouble(buf+4+16, GPS_latitude);
  mps.GPS_latitude = GPS_latitude;
  CharToDouble(buf+4+24, GPS_longitude);
  mps.GPS_longitude = GPS_longitude;
  CharToDouble(buf+4+32, GPS_altitude);
  mps.GPS_altitude = GPS_altitude;
  buf = buf + 44;
  
}

void get_other_mps(char* buf)
{
  GAS_ID = CharToInt(buf[3]);
  if(GAS_ID == GAS_PROPANE){
    string str = "Propane";
    mps_other.gasID = str;
  }else if(GAS_ID == GAS_METHANE){
    string str = "Methane";
    mps_other.gasID = str;
  }else{
    string str = "None";
    mps_other.gasID = str;
  }
  float percentLEL, temperature, local_height, humidity;
  double GPS_latitude, GPS_longitude, GPS_altitude;
  CharToFloat(buf+4, percentLEL);
  mps_other.percentLEL = percentLEL;
  CharToFloat(buf+4+4, temperature);
  mps_other.temperature = temperature;
  CharToFloat(buf+4+8, local_height);
  mps_other.local_z = local_height;
  CharToFloat(buf+4+12, humidity);
  mps_other.humidity = humidity;
  CharToDouble(buf+4+16, GPS_latitude);
  mps_other.GPS_latitude = GPS_latitude;
  CharToDouble(buf+4+24, GPS_longitude);
  mps_other.GPS_longitude = GPS_longitude;
  CharToDouble(buf+4+32, GPS_altitude);
  mps_other.GPS_altitude = GPS_altitude;
  buf = buf + 44;
  
}

void get_targetE(char* buf)
{
  double latitude, longitude, altitude;
  
  CharToDouble(buf+3, latitude);
  CharToDouble(buf+11, longitude);
  CharToDouble(buf+19, altitude);
  
  targetE.latitude = latitude;
  targetE.longitude = longitude;  
  targetE.altitude = altitude;
  buf = buf + 28;
}

void get_realTarget(char* buf)
{
  double latitude, longitude, altitude, wind_speed;
  float angle, diff_y, diff_z, release_rate;
  CharToDouble(buf+3, latitude);
  CharToDouble(buf+11, longitude);
  CharToDouble(buf+19, altitude);
  CharToFloat(buf+27, angle);

  wind_speed = CharToInt(buf[31]);
  CharToFloat(buf+32, diff_y);
  CharToFloat(buf+36, diff_z);
  CharToFloat(buf+40, release_rate);
  
  realTarget.source.latitude = latitude;
  realTarget.source.longitude = longitude;  
  realTarget.source.altitude = altitude;
  realTarget.angle = angle;
  realTarget.wind_speed = wind_speed;
  realTarget.diff_y = diff_y;
  realTarget.diff_z = diff_z;
  realTarget.release_rate = release_rate;
  
  buf = buf + 44;
}


void getAvehome(void){
  // std::cout<<"get ave home"<<std::endl;
  // get average home location

  double aveLong=0.0;
  double aveLat=0.0;
  double aveAlt=0.0;
  
  for (int i=0; i<agentHomes.size(); i++){    
    // std::cout<<"agentHome: "<<agentHomes[i]<<std::endl;
    aveLong = aveLong + agentHomes[i].longitude;
    aveLat = aveLat + agentHomes[i].latitude;
    aveAlt = aveAlt + agentHomes[i].altitude;
    //ROS_INFO("i: %d, lat: %f, long: %f, alt: %f", agentHomes[i].latitude, agentHomes[i].longitude, agentHomes[i].altitude);
    
  }
  
  averageHome.longitude = aveLong/agentHomes.size();
  averageHome.latitude = aveLat/agentHomes.size();
  averageHome.altitude = aveAlt/agentHomes.size();

}

void clearmps()
{
  mps.percentLEL = 0;
  mps.temperature = 0;
  mps.pressure = 0;
  mps.humidity = 0;
}
