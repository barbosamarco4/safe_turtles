
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "safe_turtles/Timeout.h"

ros::NodeHandle* nodePtr;
ros::Publisher pub;
ros::Subscriber cmd_sub;
ros::Timer safeTimer;

bool timeout(safe_turtles::Timeout::Request  &req,
             safe_turtles::Timeout::Response &resp)
{
    ROS_INFO("Safe Mode");
    cmd_sub.shutdown();
    safeTimer.setPeriod(req.time);
    safeTimer.start();
    return true;
}

void redirect(const geometry_msgs::Twist::ConstPtr& vel)
{
    pub.publish(vel);
}

void cmdMode(const ros::TimerEvent& te)
{
    cmd_sub  = nodePtr->subscribe("cmd_input",1,redirect);
    ROS_INFO("Normal mode");
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "mux");
    ros::NodeHandle n;
    nodePtr = &n;
    safeTimer = n.createTimer(ros::Duration(1), cmdMode, true, false);
    cmd_sub  = n.subscribe("cmd_input",1,redirect);
    ros::Subscriber safe_sub = n.subscribe("safe_input", 1000, redirect);

    ros::ServiceServer service = n.advertiseService("timeout", timeout);
    
    pub = n.advertise<geometry_msgs::Twist>("output",1000);
    ROS_INFO("Setup complete. Starting to pump callbacks");

    ros::spin();
    return 0;
}
