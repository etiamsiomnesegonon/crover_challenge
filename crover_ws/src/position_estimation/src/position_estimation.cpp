#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <Eigen/Eigen>
#include <cmath>
#include <string>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using Eigen::MatrixXd;
using namespace Eigen;
using namespace message_filters;
using namespace nav_msgs;

double StDev(0.7);          //measurements standard deviation

string mapFrame, odomFrame, realFrame;
int seqPos, seqVel, seqReal;

double dt;            //sampling time
vector <double> x(6); //state vector 6 states x, y, theta, xdot, ydot, thetadot
vector <double> xReal(6);

MatrixXd F;//Plan model

vector <double> quaternion(4);
vector <double> quaternionReal(4);

//tfScalar yaw, pitch, roll;

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
	
	/*tf::Matrix3x3 mat(quaternion);
	mat.getEulerYPR(&yaw, &pitch, &roll);
	x[2] = yaw;*/
		
	seqVel = velocity->header.seq;
	odomFrame = velocity->header.frame_id;
	
	//velocity is transformed in the map frame
	x[3]     = velocity->twist.twist.linear.x;                //xdot
	x[4]     = velocity->twist.twist.linear.y;                //ydot
	x[5]     = velocity->twist.twist.angular.z;               //thetadot
	
	seqReal = groundTruth->header.seq;
	realFrame = groundTruth->header.frame_id;
	
	xReal[0]  = groundTruth->pose.pose.position.x;            //xposReal
	xReal[1]  = groundTruth->pose.pose.position.y;            //yposReal
	
	quaternionReal[0] = groundTruth->pose.pose.orientation.x;
	quaternionReal[1] = groundTruth->pose.pose.orientation.y;
	quaternionReal[2] = groundTruth->pose.pose.orientation.z;
	quaternionReal[3] = groundTruth->pose.pose.orientation.w;
	
	/*tf::Matrix3x3 mat(quaternionReal);
	mat.getEulerYPR(&yaw, &pitch, &roll);
	xReal[2] = yaw;*/
	
	//velocity is transformed in the map frame
	xReal[3]     = groundTruth->twist.twist.linear.x;         //xdotReal
	xReal[4]     = groundTruth->twist.twist.linear.y;         //ydotReal
	xReal[5]     = groundTruth->twist.twist.angular.z;        //thetadotReal

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
	
	message_filters::TimeSynchronizer<Odometry, Odometry> sync(gnss, vel, 1000);
	//message_filters::TimeSynchronizer<Odometry, Odometry, Odometry> sync(gnss, vel, truth, 1000);
	/*sync.registerCallback(boost::bind(&callback, _1, _2, _3));*/
	
	
	/*message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
  TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));*/

	
	dt = 2;
	ros::Rate loop_rate(dt);
	
	//State space model matrix
	F = MatrixXd::Identity(6, 6);
	F(0,2) = dt;
	F(1,3) = dt;
	F(4,5) = dt;
	
	//Sensor data fused with a Kalman filter using a state space model
	


  while (ros::ok())
  {
  
    estiPos.header.seq = seqPos;
    estiPos.header.frame_id = mapFrame;
    
    estiPos.pose.pose.position.x = x[0];
    estiPos.pose.pose.position.y = x[1];
    estiPos.pose.pose.orientation.x = quaternion[0];
    estiPos.pose.pose.orientation.y = quaternion[1];
    estiPos.pose.pose.orientation.z = quaternion[2];
    estiPos.pose.pose.orientation.w = quaternion[3];
    
    estiPos.twist.twist.linear.x = x[3];
    estiPos.twist.twist.linear.y = x[4];
    estiPos.twist.twist.angular.z = x[5];

    Estimation.publish(estiPos);

    ros::spinOnce();
    loop_rate.sleep();
    
    //plot real pose against its estimation in rviz TODO

  }


  return 0;
}
