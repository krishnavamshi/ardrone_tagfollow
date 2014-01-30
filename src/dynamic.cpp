#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <ardrone_tagfollow/dynamicConfig.h>

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
  double vel_x_kp, vel_x_kd, vel_y_kp, vel_y_kd, yaw_kp, yaw_kd, thrust_kp, thrust_kd; // parameters for the pid controls
  
public:
  Tag_follow_class();//only the constructor is public
  void configCallback(ardrone_tagfollow::dynamicConfig &config, uint32_t level);
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
    cout << "the parameters of the pid are " << vel_x_kp << "  and  " << vel_x_kd<< " " <<vel_y_kp << " " << vel_y_kd << endl;
 }


void Tag_follow_class::configCallback(ardrone_tagfollow::dynamicConfig &config, uint32_t level)
{
  this->vel_x_kp = config.vel_x_kp;
  this->vel_x_kd = config.vel_x_kd;
  this->vel_y_kp = config.vel_y_kp;
  this->vel_y_kd = config.vel_y_kd;
  this->yaw_kp = config.yaw_kp;
  this->yaw_kd = config.yaw_kd;
  this->thrust_kp = config.thrust_kp;
  this->thrust_kd = config.thrust_kd;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tagfollow");   //we are initializing the node
  	Tag_follow_class  tagfollower;           //the class the we have defined above
  	dynamic_reconfigure::Server<ardrone_tagfollow::dynamicConfig> dr_srv;
    dynamic_reconfigure::Server<ardrone_tagfollow::dynamicConfig>::CallbackType dr_cb;
    dr_cb = boost::bind(&Tag_follow_class::configCallback, &tagfollower, _1, _2);
	dr_srv.setCallback(dr_cb);
  	ros::spin();
	return 0;
}
