
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "safe_turtles/Timeout.h"

bool cmd_mode;
ros::Publisher pub;
ros::Timer safeTimer;

bool timeout(safe_turtles::Timeout::Request  &req,
             safe_turtles::Timeout::Response &resp)
{
    cmd_mode = false;
    ROS_INFO("Safe Mode");
    safeTimer.setPeriod(req.time);
    safeTimer.start();
    return true;
}

void safe_redirect(const geometry_msgs::Twist::ConstPtr& vel)
{
    pub.publish(vel);
}

void cmd_redirect(const geometry_msgs::Twist::ConstPtr& vel)
{
    if(cmd_mode){
        pub.publish(vel);
    }
}

void cmdMode(const ros::TimerEvent& te)
{
    cmd_mode = true;
    ROS_INFO("Normal mode");
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "mux");
    ros::NodeHandle n;
    safeTimer = n.createTimer(ros::Duration(1), cmdMode, true, false);
    ros::Subscriber cmd_sub  = n.subscribe("cmd_input",1,cmd_redirect);
    ros::Subscriber safe_sub = n.subscribe("safe_input", 1000, safe_redirect);

    ros::ServiceServer service = n.advertiseService("timeout", timeout);
    
    pub = n.advertise<geometry_msgs::Twist>("output",1000);
    
    cmd_mode=true;
    ROS_INFO("Setup complete. Starting to pump callbacks");

    ros::spin();
    return 0;
}
