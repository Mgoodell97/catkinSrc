#include <ros/ros.h>
#include <ros/console.h>
#include <particle_filter/particles.h>

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <cstdio>
#include <ctime>

using namespace Eigen;

// ##################
// # Global variables
// ##################

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

	float XplumeFrame = (Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc) / det;
	float YplumeFrame = (-Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc) / det;

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
		float sensorReading = getReading(xInfoDes[0], xInfoDes[1], current_particles.theta[i], current_particles.X[i], current_particles.Y[i], current_particles.Z[i], current_particles.Q[i], current_particles.v[i], current_particles.Dy[i], current_particles.Dz[i] );
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
// sigma          = double
// ztMin          = double
// ztMax          = double
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

// ##################
// # Main
// ##################

int main(int argc, char **argv){
	ros::init(argc, argv, "MutualInfoMapTest");
	ros::NodeHandle nh;

	ros::Subscriber particlesFilter_sub = nh.subscribe<particle_filter::particles>("fake_particles",1, particlesFilter_cb);

	ros::Rate rate(5);

	std::clock_t start;
	float duration;

	start = std::clock();

	float sigma = 6000;
	float ztMin = 0;
	float ztMax = 100000;

	while(ros::ok() && !particles_filter_cb_flag){
		ros::spinOnce();
		if (particles_filter_cb_flag){
			break;
		}
		rate.sleep();
	}


	std::vector<float> xInfoDes;

	// while(ros::ok()){
	ros::spinOnce(); // update particles


	// rate.sleep();
	// }

	std::vector<float> xGuass = { -1/sqrt(3), 1/sqrt(3) };
	std::vector<float> wGuass = { 1, 1 };

	//	InfoMap
	int infoResX = 10;

	xInfoDes = {50, 50, 0};

	std::cout << "Info at 50,50,0 : " << std::endl;
	std::cout << informationAtX(xInfoDes, sigma, ztMin, ztMax, xGuass, wGuass) << std::endl;


	Eigen::ArrayXf xPlumePlot = Eigen::ArrayXf::LinSpaced(infoResX, 0, 100);
	Eigen::ArrayXf yPlumePlot = Eigen::ArrayXf::LinSpaced(infoResX, 0, 100);

	Eigen::MatrixXf infoMap(yPlumePlot.size(), xPlumePlot.size());

	//	start = std::clock();
	for (int xCurrentIndex = 0; xCurrentIndex < xPlumePlot.size(); xCurrentIndex++) {
		for (int yCurrentIndex = 0; yCurrentIndex < yPlumePlot.size(); yCurrentIndex++) {
			xInfoDes = {xPlumePlot[xCurrentIndex], yPlumePlot[yCurrentIndex], 0};
			infoMap(yCurrentIndex, xCurrentIndex) = informationAtX(xInfoDes, sigma, ztMin, ztMax, xGuass, wGuass);
		}
	}

	std::cout << "infoMap : " << std::endl;
	std::cout << infoMap << std::endl;

	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	ROS_INFO("Finished in : %f",duration);

	duration = 0;

	return 0;
}
