// bluerov2heavy_dob_node.cpp
// ===========================
// ROS node for the BlueROV2 Heavy DOBMPC.
// Only change from bluerov2_dob_node.cpp:
//   - node name:  bluerov2heavy_dob_node
//   - class name: BLUEROV2HEAVY_DOB
//   - header:     bluerov2heavy_dobmpc/bluerov2heavy_dob.h

#include <ros/ros.h>
#include "bluerov2heavy_dobmpc/bluerov2heavy_dob.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2heavy_dob_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);  // 20 Hz — same as standard

    ros::Time start_time = ros::Time::now();
    ros::Duration duration(50.0);  // run for 50 s

    BLUEROV2HEAVY_DOB br(nh);

    while (ros::ok())
    {
        ros::Time  current_time  = ros::Time::now();
        ros::Duration elapsed    = current_time - start_time;
        if (elapsed >= duration) {
            ROS_INFO("Reached time limit. Stopping.");
            break;
        }

        if (br.is_start == true) {
            br.EKF();    // 1. estimate disturbances
            br.solve();  // 2. solve MPC with estimated disturbances injected
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}