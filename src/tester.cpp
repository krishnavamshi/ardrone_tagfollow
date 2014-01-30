#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <ardrone_tagfollow/dynamicConfig.h>
#include <math.h>


#define errorIdxX 	0
#define errorIdxY 	1
#define errorIdxYaw	2
#define errorIdxZ	3

#define movementPID		0
#define constMovePosX	1
#define constMoveNegX	2
#define constMovePosY	3
#define constMoveNegY	4
#define constMovePosW	5
#define constMoveNegW	6
#define constMovePosZ	7
#define constMoveNegZ	8

#define SET_HEIGHT 1700 

using namespace std;

class Tag_follow_class
{

private:
  ros::NodeHandle nh;     // so that we dont make a copy while passing this as reference, and work on this one directly
  ros::Subscriber joy_sub, nav_sub,joy_drone_sub;//  subscribing to the joy message and the navdata from the joystick and ardrone topic
  ros::Publisher vel_pub;          // this will be publishing the command velocity to the drone
  ros::Publisher err_pub;
//ros::Publisher vel_directions;   // for later purposes, when i want to show markers on opencv using this array; will define messagetype later
  geometry_msgs::Twist twist_auto,twist_manual,twist_error;      //the message we use to send command to the drone
  double_t tnow, t_last,tdiff, t_prev_tag;     // current time as per roscore initialization, convert to double using .toSec() fxn
  //int tdiff;
  //ros::Duration t_diff;          // duration track of t_diff.  we can sleep and run the loop at the rate we want
  bool joy_automode;              // the flag we use to keep track of the state we are in ,{Manual or Autonomous}
  bool tag_in_sight;
 
  float goal_x, goal_y, goal_yaw; // the goal positon for our PID, in our case the center of image (320,160,0)
  void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
  void nav_callback(const ardrone_autonomy::NavdataConstPtr& nav_msg);
  void joy_drone_callback(const geometry_msgs::Twist::ConstPtr& msg);
  void publishtwist();
  double vel_x_kp, vel_x_kd, vel_y_kp, vel_y_kd, yaw_kp, yaw_kd, thrust_kp, thrust_kd; // parameters for the pid controls
  void pidcontroller();
  float  error[4],error_prev[4];
  
  uint8_t isConstVel;
  
public:
  Tag_follow_class();//only the constructor is public
  void configCallback(ardrone_tagfollow::dynamicConfig &config, uint32_t level);
 };

Tag_follow_class::Tag_follow_class():joy_automode(true)
{
    ROS_INFO("Tag_follow start");
    
    this->t_last =ros::Time::now().toSec(); //initialized the time_last to the time at which we start the system.
    //we assume that the tag is in the view of the bottom camera since the begining so this should not be a problem
    this->joy_sub       = this->nh.subscribe("joy", 1, &Tag_follow_class::joy_callback, this);
    this->nav_sub       = this->nh.subscribe("/ardrone/navdata", 1, &Tag_follow_class::nav_callback, this);
    this->joy_drone_sub = this->nh.subscribe("/cmd_vel_drone", 1, &Tag_follow_class::joy_drone_callback, this);
    this->vel_pub       = this->nh.advertise<geometry_msgs::Twist>("/cmd_vel_tester", 1);
	this->err_pub       = this->nh.advertise<geometry_msgs::Twist>("/pid_tune_err", 1);
	/******************   For Testing Purpose   **********************/
	isConstVel = movementPID;
	/*****************************************************************/
	
    //this->t_last = ros::Time::now();
    t_prev_tag = -10;
    tag_in_sight = false;
    this->twist_auto.linear.x = this->twist_auto.linear.y = this->twist_auto.linear.z = 0;
    this->twist_auto.angular.x = this->twist_auto.angular.y = this->twist_auto.angular.z = 0; 
    this ->error[0] = this ->error[1]=this ->error[2]=this ->error[3]= this ->error_prev[0]=this ->error_prev[1]=this ->error_prev[2]=this ->error_prev[3]=0;  
    publishtwist();
}
 
void Tag_follow_class::joy_callback(const sensor_msgs::Joy::ConstPtr& msg) 
{
    if (msg->buttons[4] == 1) 
	{
        if (this->joy_automode) 
        {
            this->joy_automode = false;
            cout << "We have now entered manual mode" << endl;
        }
    }
}
 
void Tag_follow_class::nav_callback(const ardrone_autonomy::NavdataConstPtr& nav_msg)
{
  	this->error[errorIdxZ]  = SET_HEIGHT - nav_msg->altd;	// Distance in millimeters
  	this->twist_error.linear.z  = this ->error[errorIdxZ];
  	if(!nav_msg->tags_width.empty())
	{
	this->tag_in_sight       = true;
	this->joy_automode       = true;
	this->error[errorIdxX]   = 500 - float_t(nav_msg->tags_yc[0]);		// y in image is along x-direction of motion
	this->error[errorIdxY]   = 500 - float_t(nav_msg->tags_xc[0]);		// x in image is along y-direction of motion
	this->error[errorIdxYaw] = 270 - float_t(nav_msg->tags_orientation[0]);
	if (this->error[errorIdxYaw] > 180)
		this->error[errorIdxYaw] -= 360;
	
	
	////////////////// TRY CUBIC ERROR ////////////////////////////
	//this->error[errorIdxX] /= 500;
	//this->error[errorIdxX] = pow(this->error[errorIdxX], 3.0);
	//this->error[errorIdxY] /= 500;
	//this->error[errorIdxY] = pow(this->error[errorIdxY], 3.0);
	///////////////////////////////////////////////////////////////	
	this->twist_error.linear.x  = this ->error[errorIdxX];
    this->twist_error.linear.y  = this ->error[errorIdxY];
    
    this->twist_error.angular.x = 0;
    this->twist_error.angular.y = 0;
    this->twist_error.angular.z = this ->error[errorIdxYaw];

	
	//cout << "pos'X': " << float_t(nav_msg->tags_yc[0]);
	//cout << "pos'y': " << float_t(nav_msg->tags_xc[0]); 
	
	this->tnow			= nav_msg->header.stamp.toSec() ;
	pidcontroller();
	this->t_last		= this->tnow; 
	this->t_prev_tag	= this->tnow;
	memcpy(this->error_prev, this->error,sizeof(this->error));
	}  
	
	else
	{
	this->tag_in_sight = false;
	this->error[errorIdxX]=this->error[errorIdxY]=this->error[errorIdxYaw]=0;
	
	this->twist_error.linear.x  = this ->error[errorIdxX];
    this->twist_error.linear.y  = this ->error[errorIdxY];
    this->twist_error.linear.z  = this ->error[errorIdxZ];
    this->twist_error.angular.x = 0;//this ->error[errorIdxY]
    this->twist_error.angular.y = 0;//this ->error[errorIdxX]
    this->twist_error.angular.z = this ->error[errorIdxYaw];
	
	this->tnow     = nav_msg->header.stamp.toSec() ;
	pidcontroller();	
	this->t_last = this->tnow;
	memcpy(this->error_prev, this->error,sizeof(this->error));
	}
}

void Tag_follow_class::joy_drone_callback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    this->twist_manual.linear.x  = msg->linear.x ; 
    this->twist_manual.linear.y  = msg->linear.y ; 
    this->twist_manual.linear.z  = msg->linear.z ;//this->twist_auto.linear.z ;
    this->twist_manual.angular.x = msg->angular.x;
    this->twist_manual.angular.y = msg->angular.y;
    this->twist_manual.angular.z = msg->angular.z;
}

	
void Tag_follow_class::publishtwist()
{
	if(joy_automode)
	{
		vel_pub.publish(twist_auto);
		err_pub.publish(twist_error);
		cout<< "auto mode " << endl;
	}
    else if(!joy_automode)
    {
		vel_pub.publish(twist_manual);
		err_pub.publish(twist_error);
		cout<<"manual mode" <<  endl;
	}
 }
 
 void Tag_follow_class::pidcontroller()
{
	if (this->tag_in_sight == true || (this->tag_in_sight == false && this->tnow - this->t_prev_tag > 1.0)) 
	{
		cout << "we are in the pid loop" << endl;
		this->twist_auto.linear.x   	= this->vel_x_kp  	* this->error[errorIdxX]	+ (this-> vel_x_kd)	* ( this->error[errorIdxX]   - this-> error_prev[errorIdxX])   / (this->tnow - this->t_last);// +/b;
		this->twist_auto.linear.y   	= this->vel_y_kp  	* this->error[errorIdxY]	+ (this-> vel_y_kd)	* ( this->error[errorIdxY]   - this-> error_prev[errorIdxY])   / (this->tnow - this->t_last);
		this->twist_auto.angular.z  	= this->yaw_kp	   	* this->error[errorIdxYaw]	+ (this->yaw_kd)	* ( this->error[errorIdxYaw] - this-> error_prev[errorIdxYaw]) / (this->tnow - this->t_last);
		this->twist_auto.linear.z 	    = this->thrust_kp	* this->error[errorIdxZ]	+ (this->thrust_kd)	* ( this->error[errorIdxZ]   - this-> error_prev[errorIdxZ])   / (this->tnow - this->t_last);
		this->twist_auto.angular.x 	= 0;
		this->twist_auto.angular.y 	= 0;
	
		if (this->twist_auto.linear.x > 1) {
			this->twist_auto.linear.x = 1;
		}
		if (this->twist_auto.linear.x < -1){
			this->twist_auto.linear.x = -1;
		}
	
		if (this->twist_auto.linear.y > 1) {
			this->twist_auto.linear.y = 1;
		}
		if (this->twist_auto.linear.y < -1){
			this->twist_auto.linear.y = -1;
		}
	
		if (this->twist_auto.angular.z > 1) {
			this->twist_auto.angular.z = 1;
		}
		if (this->twist_auto.angular.z < -1){
			this->twist_auto.angular.z = -1;
		}
	
		if (this->twist_auto.linear.z > 1) {
			this->twist_auto.linear.z = 1;
		}
		if (this->twist_auto.linear.z < -1){
			this->twist_auto.linear.z = -1;
		}
	
		switch (isConstVel)
		{
			case constMovePosX:
				this->twist_auto.linear.x   	= 0.5;
				this->twist_auto.linear.y   	= 0;
				this->twist_auto.angular.z  	= 0;
				this->twist_auto.linear.z 		= 0;
				break;
			
			case constMoveNegX:
				this->twist_auto.linear.x   	= -0.5;
				this->twist_auto.linear.y   	= 0;
				this->twist_auto.angular.z  	= 0;
				this->twist_auto.linear.z 		= 0;
				break;
			
			case constMovePosY:
				this->twist_auto.linear.x   	= 0;
				this->twist_auto.linear.y   	= 0.5;
				this->twist_auto.angular.z  	= 0;
				this->twist_auto.linear.z 		= 0;
				break;
			
			case constMoveNegY:
				this->twist_auto.linear.x   	= 0;
				this->twist_auto.linear.y   	= -0.5;
				this->twist_auto.angular.z  	= 0;
				this->twist_auto.linear.z 		= 0;
				break;
				
			case constMovePosW:
				this->twist_auto.linear.x   	= 0;
				this->twist_auto.linear.y   	= 0;
				this->twist_auto.angular.z  	= 0.5;
				this->twist_auto.linear.z 		= 0;
				break;
				
			case constMoveNegW:
				this->twist_auto.linear.x   	= 0;
				this->twist_auto.linear.y   	= 0;
				this->twist_auto.angular.z  	= -0.5;
				this->twist_auto.linear.z 		= 0;
				break;
				
			case constMovePosZ:
				this->twist_auto.linear.x   	= 0;
				this->twist_auto.linear.y   	= 0;
				this->twist_auto.angular.z  	= 0;
				this->twist_auto.linear.z 		= 0.5;
				break;
				
			case constMoveNegZ:
				this->twist_auto.linear.x   	= 0;
				this->twist_auto.linear.y   	= 0;
				this->twist_auto.angular.z  	= 0;
				this->twist_auto.linear.z 		= -0.5;
				break;
				
			default:
				break;
		}
	}
	
	cout << "vX : " << this->twist_auto.linear.x << " vY : " << this->twist_auto.linear.y << " wZ : " << this->twist_auto.angular.z << " vZ : " << this->twist_auto.linear.z << endl;
	cout  << " vxkp: "<<this->vel_x_kp<< " vxkd: "<<this->vel_x_kd<< " vykp: "<<this->vel_y_kp<< " vykd: "<<this->vel_y_kd << endl;
	publishtwist();
}


void Tag_follow_class::configCallback(ardrone_tagfollow::dynamicConfig &config, uint32_t level)
{
  this->vel_x_kp  = config.vel_x_kp;
  this->vel_x_kd  = config.vel_x_kd;
  this->vel_y_kp  = config.vel_x_kp;
  this->vel_y_kd  = config.vel_x_kd;
  this->yaw_kp    = config.yaw_kp;
  this->yaw_kd    = config.yaw_kd;
  this->thrust_kp = config.thrust_kp;
  this->thrust_kd = config.thrust_kd;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tagfollow");   //we are initializing the node
  	Tag_follow_class  tagfollower;           //the class the we have defined above
  	dynamic_reconfigure::Server<ardrone_tagfollow::dynamicConfig> dr_srv;  //belongs to dynamic reconfigure {changes kp kd online}
    dynamic_reconfigure::Server<ardrone_tagfollow::dynamicConfig>::CallbackType dr_cb;// you can ignore this for the program above
    dr_cb = boost::bind(&Tag_follow_class::configCallback, &tagfollower, _1, _2);   // dynamic recongif 
	dr_srv.setCallback(dr_cb);                                                        //  dynamic reconfig
  	ros::spin(); 
	return 0;
}
