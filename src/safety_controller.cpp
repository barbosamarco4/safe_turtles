#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Duration.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "safe_turtles/Timeout.h"

turtlesim::Pose pose;

bool returning;
float theta;
float center = 5.544444561;
float error  = 0.000000001;

void getVel(const turtlesim::Pose::ConstPtr& pose_msg)
{
    pose = *pose_msg;
}

geometry_msgs::Twist nextPos(float theta, float pose_theta, float dist)
{
    geometry_msgs::Twist vel;
    vel.linear.x = 1.5;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    if (abs(theta - pose_theta) >= error)
        vel.angular.z = 4 * (theta - pose_theta);
    else
        vel.angular.z = 0;
    return vel;
}

void toSafety(ros::Publisher& pub)
{
    float max_r = 2;
    float min_r = 1;
    float xdif  = pow((center - pose.x) , 2);
    float ydif  = pow((center - pose.y) , 2);
    float dist  = sqrt(xdif + ydif);
    ROS_INFO("Dist %f \n x %f \n y %f \n theta %f pose.theta %f \n", 
		    dist,
		    pose.x,
		    pose.y,
		    theta,
		    pose.theta);
    if (dist >= max_r && !returning) { 
        safe_turtles::Timeout srv;
        srv.request.time = ros::Duration(5);
        theta = atan2((center - pose.y),(center - pose.x));
        returning = true;
        ROS_INFO("Calling timeout!");
    }
    else if (dist >= min_r && returning) {
	geometry_msgs::Twist vel;
        vel = nextPos(theta,pose.theta,dist);
        pub.publish(vel);
    }
    else if (dist <= min_r && returning) {
        returning = false;
    }
    ROS_INFO("The turtle is at (%f,%f), with a rotation of: %f",pose.x,pose.y, pose.theta);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "safety_controller");
    ros::NodeHandle n;

    ros::Subscriber sub; 
    sub = n.subscribe("input", 1, getVel);
    ros::Publisher pub; 
    pub = n.advertise<geometry_msgs::Twist>("output", 1000);
    ros::ServiceClient client; 
    client = n.serviceClient<safe_turtles::Timeout>("timeout");
    
    ros::Publisher& pub_ref = pub; 

    pose.x = center;
    pose.y = center;

    ros::Rate pub_rate(500);
    while(ros::ok()){
	    ros::spinOnce();
	    toSafety(pub_ref);
	    pub_rate.sleep();
    }
    return 0;
}
