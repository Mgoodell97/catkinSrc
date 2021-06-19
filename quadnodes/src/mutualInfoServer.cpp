#include <ros/ros.h>
#include <ros/console.h>
#include <particle_filter/particles.h>
#include <quadnodes/mutualInfo.h>

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <cstdio>
#include <ctime>

using namespace Eigen;

// ##################
// # Global variables
// ##################

float sigma;
float ztMin;
float ztMax;

std::vector<float> xGuass = { -1/sqrt(3), 1/sqrt(3) };
std::vector<float> wGuass = { 1, 1 };

bool particles_filter_cb_flag = false;

particle_filter::particles current_particles;

// ##################
// # Functions
// ##################

float gaussFunc(float xFunc, float yFunc, float zFunc, float QFunc, float vFunc, float DyFunc, float DzFunc) {
	// returns concentration at x,y,z cordinates in plume frame
	return (QFunc/(4 * M_PI * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * (pow(yFunc,2)/DyFunc + pow(zFunc,2)/DzFunc));
}

float getReading(float xRobotDef, float yRobotDef, float thetaFunc, float xPlumeFunc, float yPlumeFunc, float zFunc, float QFunc, float vFunc, float DyFunc, float DzFunc) {
	float Stheta = sin(thetaFunc);
	float Ctheta = cos(thetaFunc);

	float det = Ctheta*Ctheta - (Stheta*-Stheta);

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

float pdf(float x, float u, float sigma){
	return  (exp( -pow(((x - u) / sigma),2)/2))/(sigma*sqrt(M_PI*2));
}

std::vector<float> conditionalEntropyAndMeasurementEntropy(std::vector<float> xInfoDes, float zt, float sigma){
	Eigen::VectorXf partileLikelihood(current_particles.X.size());
	Eigen::VectorXf probabilityZt(current_particles.X.size());
	Eigen::VectorXf conditionalEntropyVector(current_particles.X.size());

	for (int i = 0; i < current_particles.X.size(); i++) {
		                                                                           // theta      x        y        z        Q        v        Dy      Dz
		float sensorReading = getReading(xInfoDes[0], xInfoDes[1], current_particles.theta[i], current_particles.X[i], current_particles.Y[i], current_particles.Z[i] - xInfoDes[2], current_particles.Q[i], current_particles.v[i], current_particles.Dy[i], current_particles.Dz[i] );
		partileLikelihood(i) = pdf(zt, sensorReading, sigma);
		probabilityZt(i) = current_particles.weights[i] * partileLikelihood(i);
		conditionalEntropyVector(i) = probabilityZt(i) * log(partileLikelihood(i));
	}

	float probabilityZtSum = probabilityZt.sum();
	float measurementEntropySum = probabilityZtSum * log(probabilityZtSum);
	float conditionalEntropySum;

	conditionalEntropySum = conditionalEntropyVector.sum();

	std::vector<float> entropyResults = { -measurementEntropySum , -conditionalEntropySum };
	return entropyResults;
}

// xInfoDes       = {x,y,z}
// xp (particles) = {theta, x, y, z, Q, v, Dy, Dz, weight}
// sigma          = float
// ztMin          = float
// ztMax          = float
// xGuass         = {x1,x2}
// wGuass         = {w1,w2}
// particles are global variables
float informationAtX(std::vector<float> xInfoDes, float sigma, float ztMin, float ztMax, std::vector<float> xGuass, std::vector<float> wGuass){
	float integralConditional = 0;
	float integralMeasurment = 0;

	for (int i = 0; i < xGuass.size(); i++){
		float convertedZt = ((ztMin-ztMax)/2) * xGuass[i] + ((ztMin+ztMax)/2);

		std::vector<float> entropyResults;
		entropyResults = conditionalEntropyAndMeasurementEntropy(xInfoDes, convertedZt, sigma);

		integralMeasurment = integralMeasurment +  wGuass[i] * entropyResults[0];
		integralConditional = integralConditional + wGuass[i] * entropyResults[1];
	}

	return integralMeasurment - integralConditional;
}

// ##################
// # Callbacks
// ##################

void particlesFilter_cb(const particle_filter::particles::ConstPtr& particlesFilterMsg){
	current_particles = *particlesFilterMsg;
	particles_filter_cb_flag = true;
}

bool computeMutualInfoAtLoc(quadnodes::mutualInfo::Request &req, quadnodes::mutualInfo::Response &resp){
	std::clock_t start = std::clock();
	float duration = 0;

	// ros::spinOnce(); // update particles
	std::vector<float> xInfoDes = {req.x, req.y, req.z};

	resp.mutualInfoResponse = informationAtX(xInfoDes, sigma, ztMin, ztMax, xGuass, wGuass);
	// std::cout << "Response : " << std::endl;
	// std::cout << resp.mutualInfoResponse << std::endl;

	// duration = ( std::clock() - start ) / (float) CLOCKS_PER_SEC;
	// ROS_INFO("Finished in : %f",duration);

	return true;
}

// ##################
// # Main
// ##################

int main(int argc, char **argv){
	ros::init(argc, argv, "MutualInfoServer");
	ros::NodeHandle nh;

	ros::Subscriber particlesFilter_sub = nh.subscribe<particle_filter::particles>("particles",1, particlesFilter_cb);

	ros::param::get("mutualInfoServer/sigma", sigma);
  ros::param::get("mutualInfoServer/ztMin", ztMin);
  ros::param::get("mutualInfoServer/ztMax", ztMax);

	// ros::Rate rate(10000);

	while(ros::ok() && !particles_filter_cb_flag){
		ros::spinOnce();
		if (particles_filter_cb_flag){
			break;
		}
	}

	// Now particles have been recived start server.
	ros::ServiceServer server = nh.advertiseService("computeMutualInfoAtLoc",&computeMutualInfoAtLoc);

	// while(ros::ok()){
	// }

	ros::spin();

	return 0;
}
