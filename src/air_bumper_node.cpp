#include <ros/ros.h>
#include <air_bumper.h>

using namespace air_bumper;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "air_bumper_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    AirBumper air_bumper(nh);

    // wait for FCU connection
    while (ros::ok() && !air_bumper.current_state.connected)
    {
        ROS_INFO_ONCE("connecting to FCU...");
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO_ONCE("FCU Connected!");

    while(ros::ok() && !air_bumper.auto_take_off())
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Main Loop
    while (ros::ok())
    {
        air_bumper.crash_recover_FSM();

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}