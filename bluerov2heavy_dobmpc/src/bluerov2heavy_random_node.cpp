// bluerov2heavy_random_node.cpp
// ==============================
// Random excitation node for the BlueROV2 Heavy.
// Used to collect persistence-of-excitation data for RLS-VFF system identification.
//
// CHANGES vs bluerov2_random_node.cpp:
//  1. ROS topics:   /bluerov2heavy/...  (was /bluerov2/...)
//  2. Thrusters:    8 publishers (was 6)
//  3. Control DOFs: 6 (surge, sway, heave, roll, pitch, yaw) — was 4
//  4. Thrust mapping: direct u[i] / rotor_constant (no inverse allocation)
//                     because the heavy MPC takes individual thruster forces.
//  5. Control inputs: 8 echo publishers (was 4)

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <cmath>
#include <cstdlib>

using namespace Eigen;

class RandomMotionHeavy
{
private:
    ros::Subscriber pose_gt_sub;

    // 8 thruster publishers
    ros::Publisher thrust_pub[8];

    // 8 control-input echo publishers
    ros::Publisher control_input_pub[8];

    // State
    struct Euler { double phi, theta, psi; };
    struct Pose  { double x, y, z, u, v, w, p, q, r; };

    Euler local_euler;
    Pose  local_pos;
    tf::Quaternion tf_quaternion;

    // ── Parameters ────────────────────────────────────────────────────────────
    // rotor_constant: converts force [N] → thruster input command
    // Same value as the standard model.
    const double rotor_constant = 0.026546960744430276;
    const double dt = 0.05;

    // ── Thruster allocation matrix K (6×8) ───────────────────────────────────
    // Mirrors bluerov2heavy.py exactly.
    // Rows: [X, Y, Z, K(roll), M(pitch), N(yaw)]
    // Cols: thrusters 0..7
    Matrix<double,6,8> K;

    // Random excitation state
    int    random_dof   = 1;
    double random_input = 0.0;
    int    cout_counter = 0;

    // Control inputs [N] per thruster (direct, 8-dim)
    double u_cmd[8] = {0};

public:
    RandomMotionHeavy(ros::NodeHandle& nh)
    {
        // ── Thruster allocation ───────────────────────────────────────────────
        K << 0.707,  0.707, -0.707, -0.707,  0.000,  0.000,  0.000,  0.000,
             0.707, -0.707,  0.707, -0.707,  0.000,  0.000,  0.000,  0.000,
             0.000,  0.000,  0.000,  0.000,  1.000,  1.000,  1.000,  1.000,
             0.067, -0.067,  0.067, -0.067, -0.218,  0.218, -0.218,  0.218,
            -0.067, -0.067,  0.067,  0.067, -0.120, -0.120,  0.120,  0.120,
             0.1888,-0.1888,-0.1888, 0.1888,  0.000,  0.000,  0.000,  0.000;

        // ── Subscriber ────────────────────────────────────────────────────────
        pose_gt_sub = nh.subscribe<nav_msgs::Odometry>(
            "/bluerov2heavy/pose_gt", 20,
            &RandomMotionHeavy::poseGtCallback, this);

        // ── Publishers ────────────────────────────────────────────────────────
        for (int i = 0; i < 8; i++) {
            thrust_pub[i] = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(
                "/bluerov2heavy/thrusters/" + std::to_string(i) + "/input", 20);
            control_input_pub[i] = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(
                "/bluerov2heavy/control_input/" + std::to_string(i), 20);
        }
    }

    void poseGtCallback(const nav_msgs::Odometry::ConstPtr& pose)
    {
        local_pos.x = pose->pose.pose.position.x;
        local_pos.y = pose->pose.pose.position.y;
        local_pos.z = pose->pose.pose.position.z;
        local_pos.u = pose->twist.twist.linear.x;
        local_pos.v = pose->twist.twist.linear.y;
        local_pos.w = pose->twist.twist.linear.z;
        local_pos.p = pose->twist.twist.angular.x;
        local_pos.q = pose->twist.twist.angular.y;
        local_pos.r = pose->twist.twist.angular.z;
        tf::quaternionMsgToTF(pose->pose.pose.orientation, tf_quaternion);
        tf::Matrix3x3(tf_quaternion).getRPY(
            local_euler.phi, local_euler.theta, local_euler.psi);
    }

    // ── Generate a random generalised force in a single DOF ──────────────────
    // DOF mapping:  1=surge(X), 2=sway(Y), 3=heave(Z),
    //               4=roll(K),  5=pitch(M), 6=yaw(N)
    void single_dof_motion(int dof)
    {
        srand(static_cast<unsigned>(time(0)));
        random_input = -15.0 + static_cast<double>(rand()) / RAND_MAX * 30.0;

        // Build a 6×1 generalised-force vector
        Matrix<double,6,1> F = Matrix<double,6,1>::Zero();
        switch (dof) {
            case 1:  F(0) = random_input;  break;  // surge
            case 2:  F(1) = random_input;  break;  // sway
            case 3:
                // Prevent surfacing: if near z=0 and input would push upward, flip
                if (local_pos.z > -1.0 && random_input > 0) random_input = -random_input;
                F(2) = random_input;
                break;
            case 4:  F(3) = random_input;  break;  // roll  — NEW vs standard
            case 5:  F(4) = random_input;  break;  // pitch — NEW vs standard
            case 6:  F(5) = random_input;  break;  // yaw
            default: ROS_WARN("Invalid DOF: %d", dof); return;
        }

        // Compute thruster forces: u = K^† * F  (pseudo-inverse allocation)
        // K is 6×8 → K† is 8×6
        // Using Eigen's built-in SVD pseudo-inverse
        Matrix<double,8,6> K_pinv =
            K.jacobiSvd(ComputeThinU | ComputeThinV).solve(
                Matrix<double,6,6>::Identity()).transpose();
        // NOTE: for the heavy config with 8 thrusters and 6 DOF,
        // the system is over-actuated. We use the minimum-norm solution:
        //   u = K^T * (K*K^T)^{-1} * F
        Matrix<double,6,6> KKT = K * K.transpose();
        Matrix<double,8,1> u_vec = K.transpose() * KKT.inverse() * F;

        for (int i = 0; i < 8; i++) u_cmd[i] = u_vec(i);
    }

    void controller()
    {
        // Pick a random DOF (1..6 for the heavy 6-DOF vehicle)
        srand(static_cast<unsigned>(time(0)));
        random_dof = 1 + rand() % 6;  // was % 4 in standard (4-DOF)
        single_dof_motion(random_dof);

        // Publish thruster commands
        for (int i = 0; i < 8; i++) {
            uuv_gazebo_ros_plugins_msgs::FloatStamped msg;
            msg.data = u_cmd[i] / rotor_constant;
            thrust_pub[i].publish(msg);

            uuv_gazebo_ros_plugins_msgs::FloatStamped ci_msg;
            ci_msg.data = u_cmd[i];
            control_input_pub[i].publish(ci_msg);
        }

        // Console output (throttled)
        if (cout_counter > 2) {
            std::cout << "────────────────────────────────────\n";
            std::cout << "random DOF: " << random_dof
                      << "  input: " << random_input << "\n";
            std::cout << "x: " << local_pos.x << "  y: " << local_pos.y
                      << "  z: " << local_pos.z << "\n";
            std::cout << "thrusts:";
            for (int i = 0; i < 8; i++) std::cout << "  t" << i << "=" << u_cmd[i]/rotor_constant;
            std::cout << "\n────────────────────────────────────\n";
            cout_counter = 0;
        } else { cout_counter++; }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2heavy_random_node");
    ros::NodeHandle nh;
    RandomMotionHeavy rm(nh);
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        rm.controller();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}