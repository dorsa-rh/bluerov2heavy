// bluerov2heavy_ampc_node.cpp
// ============================
// ROS node for the BlueROV2 Heavy Adaptive MPC (EAOB + RLS-VFF).
// Changes from bluerov2_ampc_node.cpp:
//   - node name:  bluerov2heavy_ampc_node
//   - class name: BLUEROV2HEAVY_AMPC
//   - header:     bluerov2heavy_dobmpc/bluerov2heavy_ampc.h

#include <ros/ros.h>
#include "bluerov2heavy_dobmpc/bluerov2heavy_ampc.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2heavy_ampc_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    ros::Time start_time = ros::Time::now();
    ros::Duration duration(40.0);

    BLUEROV2HEAVY_AMPC br(nh);

    while (ros::ok())
    {
        ros::Duration elapsed = ros::Time::now() - start_time;
        if (elapsed >= duration) {
            ROS_INFO("Reached time limit. Stopping.");
            break;
        }

        if (br.is_start == true) {
            br.EKF();    // 1. EAOB: estimate total disturbance
            br.RLSFF();  // 2. RLS-VFF: decompose disturbance → update model params
            br.solve();  // 3. MPC with updated params
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}