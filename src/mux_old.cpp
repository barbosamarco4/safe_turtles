#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "safe_turles/Timeout.h"

ros::Publisher pub;
ros::Timer safeTimer;
ros::CallbackQueue cmd_queue;
bool safeMode;

void redirect(const geometry_msgs::Twist::ConstPtr& vel)
{
    ROS_INFO("vel: %p", *vel);
    pub.publish(vel);
}

void safe_redirect(const geometry_msgs::Twist::ConstPtr& vel)
{
    ROS_INFO("svel: %p", vel);
    pub.publish(vel);
}

void cmdMode(const ros::TimerEvent& te)
{
    safeTimer.stop();
    safeMode = false;
}

bool timeout(safe_turles::Timeout::Request  &req,
             safe_turles::Timeout::Response &resp)
{
    ROS_INFO("Safe");
    safeMode = true;
    safeTimer.setPeriod(req.time);
    safeTimer.start();
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "mux");
    ros::NodeHandle n;
    safeMode = false;
    safeTimer = n.createTimer(ros::Duration(1), cmdMode, true, false);
    ros::SubscribeOptions opt = ros::SubscribeOptions::create<geometry_msgs::Twist>(
                                    "cmd_input",
                                    1,
                                    redirect,
                                    ros::VoidPtr(),
                                    &cmd_queue
                                );
    ros::Subscriber cmd_sub  = n.subscribe(opt);
    ros::Subscriber safe_sub = n.subscribe("safe_input", 1000, safe_redirect);

    ros::ServiceServer service = n.advertiseService("timeout", timeout);
    
    pub = n.advertise<geometry_msgs::Twist>("output",1000);
    ROS_INFO("Setup complete. Starting to pump callbacks");

    while(ros::ok()){
        ROS_INFO("%d",safeMode);
        ros::spinOnce();
        if(!safeMode)
            cmd_queue.callOne();
    }
    return 0;
}
