#ifndef BLUEROV2HEAVY_DOB_H
#define BLUEROV2HEAVY_DOB_H

/*
 * bluerov2heavy_dob.h
 * ====================
 * Header for the BlueROV2 Heavy DOBMPC (EAOB + MPC).
 *
 * CHANGES vs bluerov2_dob.h:
 *  1.  Include guard:   BLUEROV2HEAVY_DOB_H
 *  2.  Class name:      BLUEROV2HEAVY_DOB
 *  3.  ACADOS headers:  bluerov2heavy_model.h + acados_solver_bluerov2heavy.h
 *  4.  Package include: bluerov2heavy_dobmpc/Reference.h
 *  5.  ACADOS constants: BLUEROV2HEAVY_NX/NU/NP/N/NY  (set by generate_c_code_dob.py)
 *  6.  ControlInputs enum: u1..u8  (8 controls)
 *  7.  thrust struct: t0..t7  (8 thrusters)
 *  8.  K matrix:  Matrix<double,6,8>  (was 6×6)
 *  9.  meas_u:    Matrix<double,8,1>  (was 6×1)
 *  10. Thrust publishers: thrust0..thrust7_pub  (8 channels)
 *  11. Control input publishers: control_input0..7_pub
 *  12. Physical parameters: mass, inertia, buoyancy, added_mass, Dl, Dnl
 *      updated to Ng 2024 heavy-config values.
 *  13. SolverParam: same 6 disturbance fields (unchanged structure).
 *  14. RLS buffers for K and M channels added (Kerror_n/d, Merror_n/d).
 *  15. acados_param: [BLUEROV2HEAVY_N+1][BLUEROV2HEAVY_NP]
 *  16. mpc_capsule type: bluerov2heavy_solver_capsule*
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <nav_msgs/Odometry.h>
#include <bluerov2heavy_dobmpc/Reference.h>
#include <gazebo_msgs/GetLinkState.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/LinkState.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <tuple>
#include <iomanip>
#include <random>
#include <numeric>

// ACADOS generated headers — produced by generate_c_code_dob.py
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"
#include "bluerov2heavy_model/bluerov2heavy_model.h"
#include "acados_solver_bluerov2heavy.h"

using namespace Eigen;

class BLUEROV2HEAVY_DOB {
private:

    // ── State indices (same as standard — 12-dim state unchanged) ────────────
    enum SystemStates {
        x = 0, y = 1, z = 2,
        phi = 3, theta = 4, psi = 5,
        u = 6, v = 7, w = 8,
        p = 9, q = 10, r = 11,
    };

    // ── Control indices (8 direct thruster forces) ────────────────────────────
    enum ControlInputs {
        u1 = 0, u2 = 1, u3 = 2, u4 = 3,
        u5 = 4, u6 = 5, u7 = 6, u8 = 7,   // NEW: vertical thrusters
    };

    // ── ACADOS solver I/O ─────────────────────────────────────────────────────
    struct SolverInput {
        double x0[BLUEROV2HEAVY_NX];                      // 12
        double yref[BLUEROV2HEAVY_N+1][BLUEROV2HEAVY_NY]; // (N+1) × 20
    };

    struct SolverOutput {
        double u0[BLUEROV2HEAVY_NU];   // 8
        double x1[BLUEROV2HEAVY_NX];   // 12
        double status, kkt_res, cpu_time;
    };

    // ── Supporting structs ────────────────────────────────────────────────────
    struct Euler { double phi, theta, psi; };

    struct pos {
        double x, y, z;
        double u, v, w;
        double p, q, r;
    };

    struct acc {
        double x, y, z;
        double phi, theta, psi;
    };

    struct SolverParam {
        double disturbance_x, disturbance_y, disturbance_z;
        double disturbance_phi, disturbance_theta, disturbance_psi;
    };

    // 8 thrusters (was 6)
    struct thrust {
        double t0, t1, t2, t3, t4, t5;
        double t6, t7;   // NEW: extra vertical thrusters
    };

    struct wrench { double fx, fy, fz, tx, ty, tz; };

    // ── ROS message members ───────────────────────────────────────────────────
    // Thruster command messages (8)
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0, thrust1, thrust2, thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4, thrust5;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust6, thrust7;   // NEW

    // Control input echo messages (8)
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input0, control_input1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input2, control_input3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input4, control_input5;  // NEW
    uuv_gazebo_ros_plugins_msgs::FloatStamped control_input6, control_input7;  // NEW

    Euler local_euler;
    pos   local_pos, body_pos, pre_body_pos;
    acc   local_acc, body_acc;
    thrust current_t;
    wrench applied_wrench;

    nav_msgs::Odometry ref_pose, error_pose;
    nav_msgs::Odometry esti_pose, esti_disturbance, applied_disturbance;
    gazebo_msgs::ApplyBodyWrench body_wrench;

    std::vector<ros::Subscriber> subscribers;  // sized to 8 in constructor

    // ── ACADOS ────────────────────────────────────────────────────────────────
    SolverInput  acados_in;
    SolverOutput acados_out;
    double acados_param[BLUEROV2HEAVY_N+1][BLUEROV2HEAVY_NP];  // NP=6
    int    acados_status;
    bluerov2heavy_solver_capsule* mpc_capsule = bluerov2heavy_acados_create_capsule();

    // =========================================================================
    // PHYSICAL PARAMETERS  (Ng 2024 / Blue Robotics heavy spec)
    // =========================================================================
    double dt              = 0.05;
    double mass            = 11.5;      // kg   (was 11.26)
    double Ix              = 0.8571;    // kg·m² (was 0.3)
    double Iy              = 1.0;       // kg·m² (was 0.63)
    double Iz              = 1.0;       // kg·m² (was 0.58)
    double ZG              = 0.02;      // m
    double g               = 9.81;      // m/s²
    double buoyancy        = 6.675;     // N    (was 0.661618)
    double compensate_coef = 0.032546960744430276;
    double rotor_constant  = 0.026546960744430276;

    // Added mass (Ng 2024 Table 4.1)
    // Standard was {1.7182, 0, 5.468, 0, 1.2481, 0.4006}
    double added_mass[6] = {10.77, 24.86, 34.60, 0.103, 0.120, 0.120};

    // Linear damping (Ng 2024) — K_p, M_q, N_r experimentally zero
    // Standard was {-11.7391, -20, -31.8678, -25, -44.9085, -5}
    double Dl[6]  = {-38.95, -60.11, -49.50,   0.0,   0.0,   0.0};

    // Quadratic damping (Ng 2024) — absent in standard model
    // Standard was {-18.18, -21.66, -36.99, -1.55, -1.55, -1.55}
    double Dnl[6] = {-31.01, -36.95, -113.86, -2.08, -2.42, -2.42};

    // ── Dynamics matrices ─────────────────────────────────────────────────────
    Matrix<double,1,6> M_values;
    Matrix<double,6,6> M;       // mass matrix (6×6, same structure)
    Matrix<double,6,6> invM;

    // K is now 6×8 (was 6×6)
    Matrix<double,6,8> K;       // thruster allocation matrix
    Matrix<double,6,1> KAu;     // resulting generalised forces

    Matrix<double,3,1> v_linear_body;
    Matrix<double,3,1> v_angular_body;
    Matrix<double,3,3> R_ib;
    Matrix<double,3,3> T_ib;

    // ── EKF parameters (18-dim augmented state: unchanged from standard) ──────
    Matrix<double,6,1> wf_disturbance;
    Matrix<double,8,1> meas_u;      // 8×1 (was 6×1) — 8 thruster readings
    int n = 18;
    int m = 18;
    Matrix<double,18,1> meas_y;
    MatrixXd P0       = MatrixXd::Identity(18, 18);
    Matrix<double,18,1>  esti_x;
    Matrix<double,18,18> esti_P;
    Matrix<double,1,18>  Q_cov;
    Matrix<double,18,18> noise_Q;
    MatrixXd noise_R  = MatrixXd::Identity(18, 18) * (pow(0.05, 4) / 4.0);

    // ── ROS parameters ────────────────────────────────────────────────────────
    std::string REF_TRAJ, WRENCH_FX, WRENCH_FY, WRENCH_FZ, WRENCH_TZ;
    bool AUTO_YAW;
    int  READ_WRENCH;
    bool COMPENSATE_D;
    SolverParam solver_param;

    // ── Miscellaneous ─────────────────────────────────────────────────────────
    tf::Quaternion tf_quaternion;
    int    cout_counter = 0, rand_counter = 0, fx_counter = 0;
    double dis_time = 0, periodic_counter = 0;
    double amplitudeScalingFactor_X, amplitudeScalingFactor_Y;
    double amplitudeScalingFactor_Z, amplitudeScalingFactor_N;
    double logger_time;

    float yaw_sum = 0, pre_yaw = 0, yaw_diff;
    float yaw_ref, yaw_error;

    ros::Time current_time;

    // ── ROS publishers / subscribers ──────────────────────────────────────────
    ros::Subscriber pose_sub;
    ros::Subscriber imu_sub;
    ros::ServiceClient client;

    // 8 thrust publishers
    ros::Publisher thrust0_pub, thrust1_pub, thrust2_pub, thrust3_pub;
    ros::Publisher thrust4_pub, thrust5_pub;
    ros::Publisher thrust6_pub, thrust7_pub;   // NEW

    ros::Publisher ref_pose_pub, error_pose_pub;

    // 8 control input echo publishers
    ros::Publisher control_input0_pub, control_input1_pub;
    ros::Publisher control_input2_pub, control_input3_pub;
    ros::Publisher control_input4_pub, control_input5_pub;  // NEW
    ros::Publisher control_input6_pub, control_input7_pub;  // NEW

    ros::Publisher esti_pose_pub, esti_disturbance_pub, applied_disturbance_pub;

    // ── Trajectory ────────────────────────────────────────────────────────────
    std::vector<std::vector<double>> trajectory;
    int line_number = 0, number_of_steps = 0;

public:
    bool is_start;

    BLUEROV2HEAVY_DOB(ros::NodeHandle&);
    Euler q2rpy(const geometry_msgs::Quaternion&);
    geometry_msgs::Quaternion rpy2q(const Euler&);
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>>& data);
    void ref_cb(int line_to_read);
    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void solve();

    void thrusts_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, int index);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void applyBodyWrench();
    void EKF();
    MatrixXd RK4(MatrixXd x, MatrixXd u);
    MatrixXd f(MatrixXd x, MatrixXd u);
    MatrixXd h(MatrixXd x);
    MatrixXd compute_jacobian_F(MatrixXd x, MatrixXd u);
    MatrixXd compute_jacobian_H(MatrixXd x);
    MatrixXd dynamics_C(MatrixXd v);
    MatrixXd dynamics_D(MatrixXd v);
    MatrixXd dynamics_g(MatrixXd euler);
};

#endif // BLUEROV2HEAVY_DOB_H