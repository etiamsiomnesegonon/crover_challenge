#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>

using namespace std;

double StDev(0.7);          //measurements standard deviation
double xpos;
double ypos;
double theta;
double xdot;
double ydot;
double thetadot;
string mapFrame; 
string odomFrame;
int seqPos, seqVel;

vector <double> quaternion(4);
/////////////CALLBACK FUNCTIONS///////////////////////////////////////////////////
void gnssData(const nav_msgs::Odometry::ConstPtr& odmetryGnss){
	//sensorLeft=(distMeasuredLeft.data<=sensing_threshold);
	
	seqPos = odmetryGnss->header.seq;
	mapFrame = odmetryGnss->header.frame_id;
	
	xpos  = odmetryGnss->pose.pose.position.x;
	ypos  = odmetryGnss->pose.pose.position.y;
	
	quaternion[0] = odmetryGnss->pose.pose.orientation.x;
	quaternion[1] = odmetryGnss->pose.pose.orientation.y;
	quaternion[2] = odmetryGnss->pose.pose.orientation.z;
	quaternion[3] = odmetryGnss->pose.pose.orientation.w;
	
	} //TO FILTER UNSIGNIFICANT READINGS!!!
	
void velocityData (const nav_msgs::Odometry::ConstPtr& velocity){
	
	seqVel = velocity->header.seq;
	odomFrame = velocity->header.frame_id;
	
	xdot     = velocity->twist.twist.linear.x;
	ydot     = velocity->twist.twist.linear.y;
	thetadot = velocity->twist.twist.angular.z;
 
}
/////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	
	/*cout<<"Sensing Noise StDev: ";
	cin>>StDev;*/

	ros::init(argc, argv, "position_estimation");
	ros::NodeHandle nh;

	ros::Publisher Estimation = nh.advertise<nav_msgs::Odometry>("Estimation", 1000);
	/*ros::Publisher initial_pose = nh.advertise<geometry_msgs::Pose2D>("groupFlash/initial_pose", 1000);
	ros::Publisher vel_ctrl_cmd = nh.advertise<geometry_msgs::Twist>("groupFlash/velocity_cmd", 1000);
	ros::Publisher lets_rumble = nh.advertise<std_msgs::Bool>("lets_rumble", 1000);*/
  
	ros::Subscriber gnss = nh.subscribe("/sensors/gnss/odom", 1000, gnssData);
	ros::Subscriber vel = nh.subscribe("/sensors/odom", 1000, velocityData);
	ros::Subscriber truth = nh.subscribe("/sensors/odom/ground_truth", 1000, realData);

	ros::Rate loop_rate(2);


  while (ros::ok())
  {

    nav_msgs::Odometry estiPos;
    
    estiPos.header.seq = seqPos;
    estiPos.header.frame_id = mapFrame;
    
    estiPos.pose.pose.position.x = xpos;
    estiPos.pose.pose.position.y = ypos;
    estiPos.pose.pose.orientation.x = quaternion[0];
    estiPos.pose.pose.orientation.y = quaternion[1];
    estiPos.pose.pose.orientation.z = quaternion[2];
    estiPos.pose.pose.orientation.w = quaternion[3];
    
    estiPos.twist.twist.linear.x = xdot;
    estiPos.twist.twist.linear.y = ydot;
    estiPos.twist.twist.angular.z = thetadot;

    Estimation.publish(estiPos);

    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}
