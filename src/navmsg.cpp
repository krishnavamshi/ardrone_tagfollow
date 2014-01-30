//#include <Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
//#include <vector>
using namespace std;
//using namespace Eigen;

void chatterCallback(const ardrone_autonomy::NavdataConstPtr& nav_msg)
{
	double_t difference;
	if(!nav_msg->tags_width.empty())
	{
	ROS_INFO("I heard: [%u]", nav_msg->tags_width[0]);
	ROS_INFO("tag width in pixels: [%u]", nav_msg->tags_xc[0]);
	cout << "the time stamp is " << nav_msg->header.stamp ;
	difference = ros::Time::now().toSec() - nav_msg->header.stamp.toSec() ;
	cout << "the difference is " << difference ;
	}
/*
uint32[] tags_type
uint32[] tags_xc
uint32[] tags_yc
uint32[] tags_width
uint32[] tags_height
float32[] tags_orientation
float32[] tags_distance
*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navdatalistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/ardrone/navdata", 1000, chatterCallback);

  ros::spin();

  return 0;
}
