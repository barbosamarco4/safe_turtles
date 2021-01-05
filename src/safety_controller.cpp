#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Duration.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "safe_turtles/Timeout.h"


bool returning = false;

float center = 5.544445;
float error  = 0.2;

float mod2pi(float x){ 
    while(x > 2*M_PI){
        x = x-2*M_PI;
    }
    return x;
}

geometry_msgs::Twist nextPos(const turtlesim::Pose::ConstPtr& pose)
{
    geometry_msgs::Twist vel;
    float theta = atan2( - (pose->y - center), - (pose->x - center));
    if(mod2pi(abs(theta - pose->theta)) >= error)
            vel.angular.z = 2 * (theta - pose->theta);
//    else{ 
        vel.linear.x = 1.5;
 //   }
    return vel;
}

void returnToSafety(const turtlesim::Pose::ConstPtr& pose)
{
    float max_r = 2;
    float min_r = 1;
    float theta = atan2( - (pose->y - center), - (pose->x - center));
    if((pow((pose->x-center),2) + pow((pose->y - center),2) >= pow(max_r,2)) && !returning){ 
        returning = true;
        safe_turtles::Timeout srv;
        srv.request.time = ros::Duration(5);
        ROS_INFO("Calling timeout!");
        if(!client.call(srv))
                ROS_INFO("Deu merda!");
        else
                ROS_INFO("Partyyyyy!");
    }
    else if((pow((pose->x-center),2) + pow((pose->y - center),2) >= pow(min_r,2)) && returning){
            geometry_msgs::Twist vel = nextPos(pose);
            pub.publish(vel);
    }
    else if((pow((pose->x-center),2) + pow((pose->y - center),2) <= pow(min_r,2)) && returning)
            returning = false;
    ROS_INFO("The turtle is at (%f,%f), with a rotation of: %f",pose->x,pose->y, pose->theta);

}


int main(int argc, char** argv){
    ros::init(argc,argv, "safety_controller");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("input", 1, returnToSafety);

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("output",1000);
    ros::ServiceClient client = n.serviceClient<safe_turtles::Timeout>("timeout");
    
    ros::Rate pub_rate(10);

    while(ros::ok()){
	    if(fora){
	        returnToSafety();
	    }
	    ros::spinOnce();
	    pub_rate.sleep();
    }
    return 0;
}
