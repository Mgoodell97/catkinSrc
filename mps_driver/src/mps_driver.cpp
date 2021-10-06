/*********************************************************************************************************
*                                       Copyright Notice
*
*********************************************************************************************************/

/*********************************************************************************************************
*                                  COMMAND & RESPONSE OPERATIONS
* Filename      : mps_driver.cpp
* Version       : V1.0.1
* Programmers(s): Xiang He

**********************************************************************************************************
* Notes         : (1) n/a
*/
/* Includes ---------------------------------------------------------------------------------------------*/
#include "commandResponse.h"
//ROS

/* Defines ----------------------------------------------------------------------------------------------*/
#define RUN_ONCE               1
//#define DEBUG

typedef struct {
  float temp;
  float pressure;
  float humidity;
  float absHumidity;
  float humidAirDensity;
} enviro_reply_t;

typedef struct {
  float concentration;          /* gas concentration */
  char ID[64];                  /* gas ID string */
  enviro_reply_t enviro;
} uav_answer_t;

typedef struct {
  uint32_t op;
} answerEngine_rqst_t;

typedef struct {
  uint16_t status;
} answerEngine_reply_t;

/* Variables --------------------------------------------------------------------------------------------*/
dji_sdk::GlobalPosition c_gps;
dji_sdk::LocalPosition  c_local_position;

/* Functions --------------------------------------------------------------------------------------------*/
void gps_callback(const dji_sdk::GlobalPosition &new_message)
{
    c_gps = new_message;
}

void local_position_callback(const dji_sdk::LocalPosition &new_message)
{
    c_local_position = new_message;
}

void init_MPS()
{
    // Turn on MPS through power control
    cout<<"Starting MPS."<<endl;
    system("gpio mode 4 OUTPUT");
    system("gpio write 4 0");
    usleep(20000);
    system("gpio write 4 1");
    sleep(1);
}

int main(int argc, char *argv[]){

	ros::init(argc, argv, "MPS_Driver");
	ros::NodeHandle nh;

    // Publish MPS message
    ros::Publisher MPS_pub = nh.advertise<mps_driver::MPS>("mps_data", 1);
    ros::Subscriber subscribe_GPS = nh.subscribe("dji_sdk/global_position",1,gps_callback);
    ros::Subscriber subscribe_local_position = nh.subscribe("dji_sdk/local_position",1,local_position_callback);
    ros::Rate loop_rate(1);

    // Local variables
	answerEngine_rqst_t  request;
  	answerEngine_reply_t reply;
  	uav_answer_t         answer;
	uint8_t             *rxData;
	uint16_t             rxDataLength = 0;
	uint8_t             *txData;
	uint16_t             txDataLength;
  	int                  count = 0;

  	init_MPS();

    int fd, result = 0;
    // Register the MPS at address 0x3E
    fd = wiringPiI2CSetup(0x3e);
    cout << "MPS initialized at handle: "<< fd << endl;

  	request.op = RUN_ONCE;

  	// Main loop
	while(ros::ok()){

		/* send command packet */
		sendCommand_I2C(CTRL_ANS_ENGINE, (uint8_t *) &request, sizeof(answerEngine_rqst_t), fd);

	  	/* receive data back */
	  	rxData = receiveData_I2C(CTRL_ANS_ENGINE, &rxDataLength, 2, fd);
		if(rxData == NULL) {
			close(fd);
			return 1;
		} else {
			if(rxDataLength < sizeof(answerEngine_reply_t)) {
		    	printf("Incomplete reply (%d vs %d bytes) received.\r\n", rxDataLength, (int)sizeof(answerEngine_reply_t));
		    	free(rxData);
		    	close(fd);
		    	return 1;
		    }
		}
		reply = *((answerEngine_reply_t *) rxData);
		free(rxData);
		if(reply.status == SUCCESS) {
			// Do nothing
		} else {
			close(fd);
		    return 1;
		}

		//Sleep for 2 sec, wating for MPS to get ready
		sleep(2);

		/* send command packet */
		sendCommand_I2C(UAV_ANSWER, NULL, 0, fd);

		/* receive data back */
		rxData = receiveData_I2C(UAV_ANSWER, &rxDataLength, 2, fd);
		if(rxData == NULL) {
			printf("Error reading I2C.");
			close(fd);
			return 1;
		} else {
		    if(rxDataLength < sizeof(uav_answer_t)) {
		      	printf("Incomplete reply (%d vs %d bytes) received.\r\n", rxDataLength, (int)sizeof(uav_answer_t));
		      	free(rxData);
		      	close(fd);
		      	return 1;
		    }
		}

		answer = *((uav_answer_t *) rxData);
		#ifdef DEBUG
		printf("Analyte ID, Concentration [%%LEL], Temp [C], Pressure [kPa], Humidity [%%RH]\n");
		printf("%s, %f, %f, %f, %f\n", answer.ID, answer.concentration, answer.enviro.temp, answer.enviro.pressure, answer.enviro.humidity);
		#endif

		if(answer.concentration > 100 || (answer.enviro.pressure == 0 && answer.enviro.humidity == 0)){
		  return -1;//Reading error
		}
		mps_driver::MPS mps_data;
		mps_data.gasID           = answer.ID;
		mps_data.percentLEL      = answer.concentration;
		mps_data.temperature     = answer.enviro.temp;
		mps_data.pressure        = answer.enviro.pressure;
		mps_data.humidity        = answer.enviro.humidity;
		mps_data.absHumidity     = 0;
		mps_data.humidAirDensity = 0;
		mps_data.local_x         = c_local_position.x;
		mps_data.local_y         = c_local_position.y;
		mps_data.local_z         = c_local_position.z;
		mps_data.GPS_latitude    = c_gps.latitude;
		mps_data.GPS_longitude   = c_gps.longitude;
		mps_data.GPS_altitude    = c_gps.altitude;
		MPS_pub.publish(mps_data);

		free(rxData);

	    ros::spinOnce();
	    loop_rate.sleep();
	    ++count;
	}
}
