#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/Joy.h>

using namespace std;

class Tag_follow_class
{

private:
  ros::NodeHandle nh;   // so that we dont make a copy while passing this as reference, and work on this one directly
  ros::Subscriber joy_sub, nav_sub;  //  subscribing to the joy message and the navdata from the joystick and ardrone topic
  ros::Publisher vel_pub;        // this will be publishing the command velocity to the drone
//ros::Publisher vel_directions; // for later purposes, when i want to show markers on opencv using this array; will define messagetype later
  geometry_msgs::Twist twist;  //the message we use to send command to the drone
  ros::Time tnow, time_last;              // current time as per roscore initialization
  ros::Duration t_diff;        // duration track of t_diff.  we can sleep and run the loop at the rate we want
  bool joy_automode;           // the flag we use to keep track of the state we are in ,{Manual or Autonomous}
  float goal_x, goal_y, goal_yaw; // the goal positon for our PID, in our case the center of image (320,160,0)
  void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
  void nav_callback(const ardrone_autonomy::NavdataConstPtr& nav_msg);
  void publishtwist();
 
  
public:
  Tag_follow_class();//only the constructor is public
 };


Tag_follow_class::Tag_follow_class():joy_automode(true)
{
    ROS_INFO("Tag_follow start");
    this->joy_sub = this->nh.subscribe("joy", 10, &Tag_follow_class::joy_callback, this);
    this->nav_sub = this->nh.subscribe("/ardrone/navdata", 10, &Tag_follow_class::nav_callback, this);
    this->vel_pub = this->nh.advertise<geometry_msgs::Twist>("/cmd/velo", 1);
 
 
    this->time_last = ros::Time::now();
    this->twist.linear.x = this->twist.linear.y = this->twist.linear.z = 0;
    this->twist.angular.x = this->twist.angular.y = this->twist.angular.z = 0; 
    
    publishtwist();
}
 
void Tag_follow_class::joy_callback(const sensor_msgs::Joy::ConstPtr& msg) 
{
    cout << "we are in the joy callback"  << endl;
	//ROS_INFO("I heard: [%u]", nav_msg->tags_width[0]);
	ROS_INFO("Axis is now at position %f", msg->axes[2]);
}
 
void Tag_follow_class::nav_callback(const ardrone_autonomy::NavdataConstPtr& nav_msg)
{
  	if(!nav_msg->tags_width.empty())
	{
	ROS_INFO("I heard: [%u]", nav_msg->tags_width[0]);
	ROS_INFO("tag width in pixels: [%u]", nav_msg->tags_xc[0]);
	publishtwist();  
	}  
	   
}
	
void Tag_follow_class::publishtwist()
{
	if(joy_automode)
    vel_pub.publish(twist);
 }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tagfollower");   //we are initializing the node
  	Tag_follow_class  follower;           //the class the we have defined above
  	ros::spin();
	return 0;
}
