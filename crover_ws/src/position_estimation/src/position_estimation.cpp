#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include <string>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_datatypes.h>

using namespace std;
using Eigen::MatrixXd;
using namespace Eigen;
using namespace message_filters;
using namespace nav_msgs;

double StDev(0.7);          //measurements standard deviation

string mapFrame, odomFrame, realFrame;
int seqPos, seqVel, seqReal;

double dt;                  //sampling time
VectorXd x;                 // measured state vector 6 states x, y, theta, xdot, ydot, thetadot
VectorXd xest;              // estimated state vector 6 states x, y, theta, xdot, ydot, thetadot
VectorXd xReal;

MatrixXd odom2map;          //transformation from odom frame to map frame
VectorXd o2m;               //intermediary vector to get velocities in map frame

MatrixXd A; //Plan model
MatrixXd P; //estimate covariance matrix
MatrixXd R; //covariance of the observation noise
MatrixXd S;
MatrixXd K;

tf::Quaternion quaternion, quaternionReal;
double yaw, pitch, roll;

//CALLBACK FUNCTION/

void callback(const Odometry::ConstPtr& odmetryGnss, const Odometry::ConstPtr& velocity, const Odometry::ConstPtr& groundTruth)
{
		
	seqPos = odmetryGnss->header.seq;
	mapFrame = odmetryGnss->header.frame_id;
	
	x[0]  = odmetryGnss->pose.pose.position.x;                //xpos
	x[1]  = odmetryGnss->pose.pose.position.y;                //ypos
	
	quaternion[0] = odmetryGnss->pose.pose.orientation.x;
	quaternion[1] = odmetryGnss->pose.pose.orientation.y;
	quaternion[2] = odmetryGnss->pose.pose.orientation.z;
	quaternion[3] = odmetryGnss->pose.pose.orientation.w;
	 
	tf::Matrix3x3 mat(quaternion);
	mat.getEulerYPR(yaw, pitch, roll);
	x[2] = yaw;
		
	seqVel = velocity->header.seq;
	odomFrame = velocity->header.frame_id;
	
	//velocity is transformed in the map frame
	x[3]     = velocity->twist.twist.linear.x;                //xdot
	x[4]     = velocity->twist.twist.linear.y;                //ydot
	x[5]     = velocity->twist.twist.angular.z;               //thetadot
	
	o2m = VectorXd::Identity(3, 1);
	o2m[0] = x[3];
	o2m[1] = x[4];
	o2m[2] = 0;
	odom2map = MatrixXd::Identity(3, 3);
	odom2map(0,0) = cos(yaw);
	odom2map(1,1) = sin(yaw);
	o2m = odom2map*o2m;
	x[3] = o2m[0];
	x[4] = o2m[1];
	
	seqReal = groundTruth->header.seq;
	realFrame = groundTruth->header.frame_id;
	
	xReal[0]  = groundTruth->pose.pose.position.x;            //xposReal
	xReal[1]  = groundTruth->pose.pose.position.y;            //yposReal
	
	quaternionReal[0] = groundTruth->pose.pose.orientation.x;
	quaternionReal[1] = groundTruth->pose.pose.orientation.y;
	quaternionReal[2] = groundTruth->pose.pose.orientation.z;
	quaternionReal[3] = groundTruth->pose.pose.orientation.w;
	
	tf::Matrix3x3 mat1(quaternionReal);
	mat1.getEulerYPR(yaw, pitch, roll);
	xReal[2] = yaw;
	
	//velocity is transformed in the map frame
	xReal[3]     = groundTruth->twist.twist.linear.x;         //xdotReal
	xReal[4]     = groundTruth->twist.twist.linear.y;         //ydotReal
	xReal[5]     = groundTruth->twist.twist.angular.z;        //thetadotReal
	
	o2m[0] = xReal[3];
	o2m[1] = xReal[4];
	odom2map(0,0) = cos(yaw);
	odom2map(1,1) = sin(yaw);
	o2m = odom2map*o2m;
	xReal[3] = o2m[0];
	xReal[4] = o2m[1];

}
//END CALLBACKS

int main(int argc, char **argv)
{

	ros::init(argc, argv, "position_estimation");
	ros::NodeHandle nh;

	ros::Publisher Estimation = nh.advertise<nav_msgs::Odometry>("Estimation", 1000);
	nav_msgs::Odometry estiPos;
	
	message_filters::Subscriber<Odometry> gnss(nh, "sensors/gnss/odom", 1000);
	message_filters::Subscriber<Odometry> vel(nh, "sensors/odom", 1000);
	message_filters::Subscriber<Odometry> truth(nh, "sensors/odom/ground_truth", 1000);
	
	//sensors data needs to be synchronised
	message_filters::TimeSynchronizer<Odometry, Odometry, Odometry> sync(gnss, vel, truth, 1000);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3));
	
	dt = 2;   //sampling rate for every sensor 2 sec
	ros::Rate loop_rate(dt);
	
	//Sensor data fused with a Kalman filter using a state space model
	
	//State initialisation
	x = VectorXd::Identity(6, 1);
	xest = VectorXd::Identity(6, 1);
	xReal = VectorXd::Identity(6, 1);
	
	//State space model matrix
	A = MatrixXd::Identity(6, 6);
	A(0,2) = dt;
	A(1,3) = dt;
	A(4,5) = dt;
	
	//estimate covariance matrix
	P = StDev*MatrixXd::Identity(6, 6);
	
	//covariance of the observation noise
	R = StDev*MatrixXd::Identity(6, 6);
	
	// H observation model matrix is the identity since all states are measured
	// Q the covariance of the process noise is not evaluated
	

  while (ros::ok())
  {
	  
    //Kalman filter update   
    S = P+R;
    K = P*S.inverse();
    xest = xest + K*(x-xest);
    P = (MatrixXd::Identity(6, 6) - K)*P;
  
    estiPos.header.seq = seqPos;
    estiPos.header.frame_id = mapFrame;
    
    estiPos.pose.pose.position.x = xest[0];
    estiPos.pose.pose.position.y = xest[1];

    quaternion = tf::createQuaternionFromRPY(0, 0, xest[2]);
    
    estiPos.pose.pose.orientation.x = quaternion[0];
    estiPos.pose.pose.orientation.y = quaternion[1];
    estiPos.pose.pose.orientation.z = quaternion[2];
    estiPos.pose.pose.orientation.w = quaternion[3];
    
    estiPos.twist.twist.linear.x = xest[3];
    estiPos.twist.twist.linear.y = xest[4];
    estiPos.twist.twist.angular.z = xest[5];

    Estimation.publish(estiPos);

    ros::spinOnce();
    loop_rate.sleep();
    
    //Kalman filter prediction 
	xest = A*xest;   //no control cmd the noise is included in the data
	P = A*P*A.transpose(); //no noise covariance
    
    //plot real pose against its estimation in rviz TODO

  }


  return 0;
}
