/*
 * bluerov2heavy_dob.cpp
 * =====================
 * Disturbance Observer + MPC for the BlueROV2 Heavy (8-thruster, 6-DOF).
 *
 * CHANGES vs bluerov2_dob.cpp:
 *  1.  Physical parameters — mass, inertia, buoyancy updated to heavy values (Ng 2024).
 *  2.  Added mass — all 6 DOF updated; heave Z_wdot is piecewise at runtime.
 *  3.  Linear damping  Dl[6] — Ng 2024 experimental values; K_p=M_q=N_r=0.
 *  4.  Quadratic damping Dnl[6] — Ng 2024; absent in the standard model.
 *  5.  K matrix — 6×8 (was 6×6). Direct thruster-force allocation; no inverse step.
 *  6.  meas_u — 8×1 vector (was 6×1).
 *  7.  Thrust publishers/subscribers — 8 channels (was 6).
 *  8.  Control-input publishers — 8 channels (was 4).
 *  9.  All 6 disturbance DOFs compensated in solve() (was only x,y,z,psi).
 *  10. solve() — direct pass-through: u0[i]/rotor_constant (no inverse allocation).
 *  11. ROS topics — /bluerov2heavy/... (was /bluerov2/...).
 *  12. ACADOS capsule — bluerov2heavy_acados_* (was bluerov2_acados_*).
 *  13. EKF initial depth — z = -5 m (ENU, was -20 m NED).
 *  14. f() / h() — damping terms and piecewise heave added.
 *
 * HEADER CHANGES REQUIRED (bluerov2heavy_dob.h):
 *  - Rename class to BLUEROV2HEAVY_DOB
 *  - Include "bluerov2heavy_acados/acados_solver_bluerov2heavy.h"
 *  - Change BLUEROV2_NX, BLUEROV2_NU, BLUEROV2_NP, BLUEROV2_N, BLUEROV2_NY
 *    to BLUEROV2HEAVY_NX (=12), BLUEROV2HEAVY_NU (=8), etc.
 *  - Add thrust6_pub, thrust7_pub, control_input4..7_pub members
 *  - Change K to Matrix<double,6,8>
 *  - Change meas_u to Matrix<double,8,1>
 *  - Add struct member current_t.t6, .t7
 *  - Change Dl, Dnl arrays to double[6] with Ng 2024 values
 */

#include <bluerov2heavy_dobmpc/bluerov2heavy_dob.h>

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
BLUEROV2HEAVY_DOB::BLUEROV2HEAVY_DOB(ros::NodeHandle& nh)
{
    // ── ROS parameters ────────────────────────────────────────────────────────
    nh.getParam("/bluerov2heavy_dob_node/auto_yaw",       AUTO_YAW);
    nh.getParam("/bluerov2heavy_dob_node/read_wrench",    READ_WRENCH);
    nh.getParam("/bluerov2heavy_dob_node/compensate_d",   COMPENSATE_D);
    nh.getParam("/bluerov2heavy_dob_node/ref_traj",       REF_TRAJ);
    nh.getParam("/bluerov2heavy_dob_node/applied_forcex", WRENCH_FX);
    nh.getParam("/bluerov2heavy_dob_node/applied_forcey", WRENCH_FY);
    nh.getParam("/bluerov2heavy_dob_node/applied_forcez", WRENCH_FZ);
    nh.getParam("/bluerov2heavy_dob_node/applied_torquez",WRENCH_TZ);
    nh.getParam("/bluerov2heavy_dob_node/disturbance_x",  solver_param.disturbance_x);
    nh.getParam("/bluerov2heavy_dob_node/disturbance_y",  solver_param.disturbance_y);
    nh.getParam("/bluerov2heavy_dob_node/disturbance_z",  solver_param.disturbance_z);
    nh.getParam("/bluerov2heavy_dob_node/disturbance_phi",  solver_param.disturbance_phi);
    nh.getParam("/bluerov2heavy_dob_node/disturbance_theta",solver_param.disturbance_theta);
    nh.getParam("/bluerov2heavy_dob_node/disturbance_psi",  solver_param.disturbance_psi);

    // ── Pre-load trajectory ───────────────────────────────────────────────────
    const char* c = REF_TRAJ.c_str();
    number_of_steps = readDataFromFile(c, trajectory);
    if (number_of_steps == 0)
        ROS_WARN("Cannot load trajectory file!");
    else
        ROS_INFO_STREAM("Trajectory steps loaded: " << number_of_steps);

    // ── ACADOS initialisation ─────────────────────────────────────────────────
    // NOTE: capsule type is bluerov2heavy_solver_capsule* from the generated code
    int create_status = bluerov2heavy_acados_create(mpc_capsule);
    if (create_status != 0) {
        ROS_ERROR_STREAM("bluerov2heavy_acados_create returned " << create_status);
        exit(1);
    }

    // =========================================================================
    // PHYSICAL PARAMETERS  —  Heavy configuration (Ng 2024 / Blue Robotics)
    // =========================================================================
    // mass = 11.5 kg  (defined in header)
    // Ix   = 0.8571   (defined in header)
    // Iy   = 1.0      (defined in header)
    // Iz   = 1.0      (defined in header)
    // ZG   = 0.02     (defined in header)
    // g    = 9.81     (defined in header)
    // buoyancy = 6.675 N  (defined in header, was 1.962 N in standard)

    // ── Added mass (Ng 2024 Table 4.1) ───────────────────────────────────────
    // Heavy values differ significantly from the standard model:
    //   Standard:  [1.7182, 0,      5.468,  0,      1.2481, 0.4006]
    //   Heavy:     [10.77,  24.86,  34.60,  0.103,  0.120,  0.120 ]
    // Heave Z_wdot is piecewise: dive (w<0) = 34.60, rise (w>0) = 22.45
    // The constructor uses the dive value as the nominal; runtime piecewise
    // is handled inside f() and h().
    // added_mass[] is declared in the header:
    //   double added_mass[6] = {10.77, 24.86, 34.60, 0.103, 0.120, 0.120};

    // ── Mass matrix M (6×6) ───────────────────────────────────────────────────
    // M = diag(m+Xu̇, m+Yv̇, m+Zẇ, Ix+Kṗ, Iy+Mq̇, Iz+Nṙ) + off-diag ZG terms
    M_values << mass + added_mass[0],   // surge:  11.5 + 10.77  = 22.27
                mass + added_mass[1],   // sway:   11.5 + 24.86  = 36.36
                mass + added_mass[2],   // heave:  11.5 + 34.60  = 46.10  (nominal dive)
                Ix   + added_mass[3],   // roll:   0.8571 + 0.103= 0.9601
                Iy   + added_mass[4],   // pitch:  1.0 + 0.120   = 1.120
                Iz   + added_mass[5];   // yaw:    1.0 + 0.120   = 1.120
    M = M_values.asDiagonal();
    // Off-diagonal ZG coupling (same as standard)
    M(0,4) =  mass * ZG;
    M(1,3) = -mass * ZG;
    M(3,1) = -mass * ZG;
    M(4,0) =  mass * ZG;
    invM = M.inverse();

    // ── Linear damping Dl (Ng 2024 Table 4.1) ────────────────────────────────
    // Standard used: [-11.7391, -20, -31.8678, -25, -44.9085, -5]
    // Ng 2024:       X_u=38.95, Y_v=60.11, Z_w=49.50, K_p=0, M_q=0, N_r=0
    // Sign convention: negative because damping opposes motion
    // Dl[6] declared in header:
    //   double Dl[6] = {-38.95, -60.11, -49.50, 0.0, 0.0, 0.0};

    // ── Quadratic damping Dnl (Ng 2024 Table 4.1) ────────────────────────────
    // Standard had no quadratic damping (all zeros).
    // Heavy:  X_uu=31.01, Y_vv=36.95, Z_ww=113.86, K_pp=2.08, M_qq=2.42, N_rr=2.42
    // Dnl[6] declared in header:
    //   double Dnl[6] = {-31.01, -36.95, -113.86, -2.08, -2.42, -2.42};

    // =========================================================================
    // THRUSTER ALLOCATION MATRIX  K  (6×8)
    // =========================================================================
    // Standard was 6×6 (and used a 2-step u→t→K·t path with an inverse map).
    // Heavy:  K is 6×8; u0[0..7] from ACADOS are DIRECT thruster forces [N].
    //
    // Rows: [X, Y, Z, K(roll), M(pitch), N(yaw)]
    // Cols: thrusters 0..7
    //   Thrusters 0-3: horizontal (surge/sway/yaw + small roll/pitch moments)
    //   Thrusters 4-7: vertical   (heave/roll/pitch only)
    K << 0.707,  0.707, -0.707, -0.707,  0.000,  0.000,  0.000,  0.000,   // X
         0.707, -0.707,  0.707, -0.707,  0.000,  0.000,  0.000,  0.000,   // Y
         0.000,  0.000,  0.000,  0.000,  1.000,  1.000,  1.000,  1.000,   // Z
         0.067, -0.067,  0.067, -0.067, -0.218,  0.218, -0.218,  0.218,   // K
        -0.067, -0.067,  0.067,  0.067, -0.120, -0.120,  0.120,  0.120,   // M
         0.1888,-0.1888,-0.1888, 0.1888,  0.000,  0.000,  0.000,  0.000;  // N

    // ── EKF noise covariance (same structure as standard) ─────────────────────
    Q_cov << pow(dt,4)/4, pow(dt,4)/4, pow(dt,4)/4,
             pow(dt,4)/4, pow(dt,4)/4, pow(dt,4)/4,
             pow(dt,2),   pow(dt,2),   pow(dt,2),
             pow(dt,2),   pow(dt,2),   pow(dt,2),
             pow(dt,2),   pow(dt,2),   pow(dt,2),
             pow(dt,2),   pow(dt,2),   pow(dt,2);
    noise_Q = Q_cov.asDiagonal();

    // ── EKF initial state ─────────────────────────────────────────────────────
    // x(0..5)  = pose [x,y,z,phi,theta,psi]  — z=-5 m (ENU, was -20 NED)
    // x(6..11) = body velocities [u,v,w,p,q,r]
    // x(12..17)= disturbances [Xw,Yw,Zw,Kw,Mw,Nw] — small non-zero seed
    esti_x << 0, 0, -5, 0, 0, 0,
               0, 0,  0, 0, 0, 0,
               6, 6,  6, 0, 0, 0;
    esti_P = P0;

    // ── Applied wrench init ───────────────────────────────────────────────────
    applied_wrench.fx = applied_wrench.fy = applied_wrench.fz = 0.0;
    applied_wrench.tx = applied_wrench.ty = applied_wrench.tz = 0.0;

    // =========================================================================
    // ROS PUBLISHERS / SUBSCRIBERS
    // =========================================================================
    pose_sub = nh.subscribe<nav_msgs::Odometry>(
        "/bluerov2heavy/pose_gt", 20, &BLUEROV2HEAVY_DOB::pose_cb, this);

    // ── Thruster command publishers (8 channels) ──────────────────────────────
    thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/0/input",20);
    thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/1/input",20);
    thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/2/input",20);
    thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/3/input",20);
    thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/4/input",20);
    thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/5/input",20);
    thrust6_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/6/input",20);  // NEW
    thrust7_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/7/input",20);  // NEW

    // ── Diagnostic / reference publishers ────────────────────────────────────
    ref_pose_pub          = nh.advertise<nav_msgs::Odometry>("/bluerov2heavy/mpc/reference",20);
    error_pose_pub        = nh.advertise<nav_msgs::Odometry>("/bluerov2heavy/mpc/error",20);
    esti_pose_pub         = nh.advertise<nav_msgs::Odometry>("/bluerov2heavy/ekf/pose",20);
    esti_disturbance_pub  = nh.advertise<nav_msgs::Odometry>("/bluerov2heavy/ekf/disturbance",20);
    applied_disturbance_pub= nh.advertise<nav_msgs::Odometry>("/bluerov2heavy/applied_disturbance",20);

    // ── Control-input echo publishers (8 channels) ────────────────────────────
    control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/0",20);
    control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/1",20);
    control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/2",20);
    control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/3",20);
    control_input4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/4",20);  // NEW
    control_input5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/5",20);  // NEW
    control_input6_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/6",20);  // NEW
    control_input7_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/7",20);  // NEW

    // ── Thrust feedback subscribers (8 channels) ──────────────────────────────
    // Used by EKF to form the measurement tau = K*t
    subscribers.resize(8);   // was 6
    for (int i = 0; i < 8; i++) {
        std::string topic = "/bluerov2heavy/thrusters/" + std::to_string(i) + "/thrust";
        subscribers[i] = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(
            topic, 20, boost::bind(&BLUEROV2HEAVY_DOB::thrusts_cb, this, _1, i));
    }

    client    = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    imu_sub   = nh.subscribe<sensor_msgs::Imu>("/bluerov2heavy/imu", 20, &BLUEROV2HEAVY_DOB::imu_cb, this);

    // ── ACADOS init ───────────────────────────────────────────────────────────
    for (unsigned int i = 0; i < BLUEROV2HEAVY_NU; i++) acados_out.u0[i] = 0.0;
    for (unsigned int i = 0; i < BLUEROV2HEAVY_NX; i++) acados_in.x0[i]  = 0.0;
    is_start = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// pose_cb  —  unchanged logic; same coordinate handling
// ─────────────────────────────────────────────────────────────────────────────
void BLUEROV2HEAVY_DOB::pose_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
    is_start = true;
    local_pos.x = pose->pose.pose.position.x;
    local_pos.y = pose->pose.pose.position.y;
    local_pos.z = pose->pose.pose.position.z;

    tf::quaternionMsgToTF(pose->pose.pose.orientation, tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);

    local_pos.u = pose->twist.twist.linear.x;
    local_pos.v = pose->twist.twist.linear.y;
    local_pos.w = pose->twist.twist.linear.z;
    local_pos.p = pose->twist.twist.angular.x;
    local_pos.q = pose->twist.twist.angular.y;
    local_pos.r = pose->twist.twist.angular.z;

    Matrix<double,3,1> v_linear_inertial, v_angular_inertial;
    v_linear_inertial  << local_pos.u, local_pos.v, local_pos.w;
    v_angular_inertial << local_pos.p, local_pos.q, local_pos.r;

    R_ib << cos(local_euler.psi)*cos(local_euler.theta),
            -sin(local_euler.psi)*cos(local_euler.phi)+cos(local_euler.psi)*sin(local_euler.theta)*sin(local_euler.phi),
             sin(local_euler.psi)*sin(local_euler.phi)+cos(local_euler.psi)*cos(local_euler.phi)*sin(local_euler.theta),
             sin(local_euler.psi)*cos(local_euler.theta),
             cos(local_euler.psi)*cos(local_euler.phi)+sin(local_euler.phi)*sin(local_euler.theta)*sin(local_euler.psi),
            -cos(local_euler.psi)*sin(local_euler.phi)+sin(local_euler.theta)*sin(local_euler.psi)*cos(local_euler.phi),
            -sin(local_euler.theta),
             cos(local_euler.theta)*sin(local_euler.phi),
             cos(local_euler.theta)*cos(local_euler.phi);

    T_ib << 1, sin(local_euler.phi)*sin(local_euler.theta)/cos(local_euler.theta),
                cos(local_euler.phi)*sin(local_euler.theta)/cos(local_euler.theta),
             0, cos(local_euler.phi), -sin(local_euler.phi),
             0, sin(local_euler.phi)/cos(local_euler.theta),
                cos(local_euler.phi)/cos(local_euler.theta);

    v_linear_body  = R_ib.inverse() * v_linear_inertial;
    v_angular_body = T_ib.inverse() * v_angular_inertial;

    body_acc.x     = (v_linear_body[0]  - pre_body_pos.u) / dt;
    body_acc.y     = (v_linear_body[1]  - pre_body_pos.v) / dt;
    body_acc.z     = (v_linear_body[2]  - pre_body_pos.w) / dt;
    body_acc.phi   = (v_angular_body[0] - pre_body_pos.p) / dt;
    body_acc.theta = (v_angular_body[1] - pre_body_pos.q) / dt;
    body_acc.psi   = (v_angular_body[2] - pre_body_pos.r) / dt;

    pre_body_pos.u = v_linear_body[0];
    pre_body_pos.v = v_linear_body[1];
    pre_body_pos.w = v_linear_body[2];
    pre_body_pos.p = v_angular_body[0];
    pre_body_pos.q = v_angular_body[1];
    pre_body_pos.r = v_angular_body[2];
}

// ─────────────────────────────────────────────────────────────────────────────
// thrusts_cb  —  8 channels (was 6)
// ─────────────────────────────────────────────────────────────────────────────
void BLUEROV2HEAVY_DOB::thrusts_cb(
    const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, int index)
{
    switch (index) {
        case 0: current_t.t0 = msg->data; break;
        case 1: current_t.t1 = msg->data; break;
        case 2: current_t.t2 = msg->data; break;
        case 3: current_t.t3 = msg->data; break;
        case 4: current_t.t4 = msg->data; break;
        case 5: current_t.t5 = msg->data; break;
        case 6: current_t.t6 = msg->data; break;   // NEW
        case 7: current_t.t7 = msg->data; break;   // NEW
        default: ROS_WARN("Invalid thruster index: %d", index); break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// imu_cb  —  unchanged
// ─────────────────────────────────────────────────────────────────────────────
void BLUEROV2HEAVY_DOB::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    local_acc.x = round(msg->linear_acceleration.x * 10000) / 10000;
    local_acc.y = round(msg->linear_acceleration.y * 10000) / 10000;
    local_acc.z = round(msg->linear_acceleration.z * 10000) / 10000 - g;
}

// ─────────────────────────────────────────────────────────────────────────────
// EKF
// ─────────────────────────────────────────────────────────────────────────────
void BLUEROV2HEAVY_DOB::EKF()
{
    // ── Assemble meas_u (8×1) from thrust feedback ────────────────────────────
    // meas_u are the RAW thruster-force values [N] returned by UUV simulator
    meas_u << current_t.t0, current_t.t1, current_t.t2, current_t.t3,
               current_t.t4, current_t.t5, current_t.t6, current_t.t7;

    // tau = K (6×8) * meas_u (8×1) → 6×1 generalised force/torque
    Matrix<double,6,1> tau = K * meas_u;

    // ── Measurement vector y (18×1) ───────────────────────────────────────────
    // Layout: [pos(6), vel(6), tau(6)]
    meas_y << local_pos.x, local_pos.y, local_pos.z,
               local_euler.phi, local_euler.theta, local_euler.psi,
               v_linear_body[0], v_linear_body[1], v_linear_body[2],
               v_angular_body[0], v_angular_body[1], v_angular_body[2],
               tau(0), tau(1), tau(2), tau(3), tau(4), tau(5);

    // ── EKF matrices ─────────────────────────────────────────────────────────
    Matrix<double,18,18> F, H, Kal;
    Matrix<double,18,1>  x_pred, y_pred, y_err;
    Matrix<double,18,18> P_pred;

    // Prediction step (discrete, using RK4 propagation of the 18-dim state)
    F      = compute_jacobian_F(esti_x, meas_u);
    x_pred = RK4(esti_x, meas_u);
    P_pred = F * esti_P * F.transpose() + noise_Q;

    // Update step
    H     = compute_jacobian_H(x_pred);
    y_pred= h(x_pred);
    y_err = meas_y - y_pred;
    Kal   = P_pred * H.transpose() * (H * P_pred * H.transpose() + noise_R).inverse();
    esti_x= x_pred + Kal * y_err;
    esti_P= (MatrixXd::Identity(n,n) - Kal*H) * P_pred
          * (MatrixXd::Identity(n,n) - Kal*H).transpose()
          + Kal * noise_R * Kal.transpose();

    // ── Body→world frame rotation for disturbance publishing ─────────────────
    // esti_x(12..14) = body-frame force disturbance  [Xw, Yw, Zw]
    // esti_x(15..17) = body-frame torque disturbance [Kw, Mw, Nw]
    wf_disturbance <<
        (cos(meas_y(5))*cos(meas_y(4)))*esti_x(12)
            +(-sin(meas_y(5))*cos(meas_y(3))+cos(meas_y(5))*sin(meas_y(4))*sin(meas_y(3)))*esti_x(13)
            +(sin(meas_y(5))*sin(meas_y(3))+cos(meas_y(5))*cos(meas_y(3))*sin(meas_y(4)))*esti_x(14),
        (sin(meas_y(5))*cos(meas_y(4)))*esti_x(12)
            +(cos(meas_y(5))*cos(meas_y(3))+sin(meas_y(3))*sin(meas_y(4))*sin(meas_y(5)))*esti_x(13)
            +(-cos(meas_y(5))*sin(meas_y(3))+sin(meas_y(4))*sin(meas_y(5))*cos(meas_y(3)))*esti_x(14),
        (-sin(meas_y(4)))*esti_x(12)
            +(cos(meas_y(4))*sin(meas_y(3)))*esti_x(13)
            +(cos(meas_y(4))*cos(meas_y(3)))*esti_x(14),
        esti_x(15) + (sin(meas_y(5))*sin(meas_y(4))/cos(meas_y(4)))*esti_x(16)
                   +  cos(meas_y(3))*sin(meas_y(4))/cos(meas_y(4))*esti_x(17),
        cos(meas_y(3))*esti_x(16) + sin(meas_y(3))*esti_x(17),
        (sin(meas_y(3))/cos(meas_y(4)))*esti_x(16)
            + (cos(meas_y(3))/cos(meas_y(4)))*esti_x(17);

    // ── Publish EKF pose ──────────────────────────────────────────────────────
    tf2::Quaternion quat;
    quat.setRPY(esti_x(3), esti_x(4), esti_x(5));
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);

    esti_pose.pose.pose.position.x    = esti_x(0);
    esti_pose.pose.pose.position.y    = esti_x(1);
    esti_pose.pose.pose.position.z    = esti_x(2);
    esti_pose.pose.pose.orientation   = quat_msg;
    esti_pose.twist.twist.linear.x    = esti_x(6);
    esti_pose.twist.twist.linear.y    = esti_x(7);
    esti_pose.twist.twist.linear.z    = esti_x(8);
    esti_pose.twist.twist.angular.x   = esti_x(9);
    esti_pose.twist.twist.angular.y   = esti_x(10);
    esti_pose.twist.twist.angular.z   = esti_x(11);
    esti_pose.header.stamp            = ros::Time::now();
    esti_pose.header.frame_id         = "odom_frame";
    esti_pose.child_frame_id          = "base_link";
    esti_pose_pub.publish(esti_pose);

    // ── Publish estimated disturbance (body frame) ────────────────────────────
    esti_disturbance.pose.pose.position.x    = wf_disturbance(0);
    esti_disturbance.pose.pose.position.y    = wf_disturbance(1);
    esti_disturbance.pose.pose.position.z    = wf_disturbance(2);
    esti_disturbance.twist.twist.angular.x   = wf_disturbance(3);
    esti_disturbance.twist.twist.angular.y   = wf_disturbance(4);
    esti_disturbance.twist.twist.angular.z   = wf_disturbance(5);
    esti_disturbance.header.stamp            = ros::Time::now();
    esti_disturbance.header.frame_id         = "odom_frame";
    esti_disturbance.child_frame_id          = "base_link";
    esti_disturbance_pub.publish(esti_disturbance);

    // ── Publish applied wrench (ground truth for comparison) ──────────────────
    applied_disturbance.pose.pose.position.x  = applied_wrench.fx;
    applied_disturbance.pose.pose.position.y  = applied_wrench.fy;
    applied_disturbance.pose.pose.position.z  = applied_wrench.fz;
    applied_disturbance.twist.twist.angular.x = applied_wrench.tx;
    applied_disturbance.twist.twist.angular.y = applied_wrench.ty;
    applied_disturbance.twist.twist.angular.z = applied_wrench.tz;
    applied_disturbance.header.stamp          = ros::Time::now();
    applied_disturbance.header.frame_id       = "odom_frame";
    applied_disturbance.child_frame_id        = "base_link";
    applied_disturbance_pub.publish(applied_disturbance);

    if (cout_counter > 2) {
        std::cout << "─────────────────────────────────────────────────────\n";
        std::cout << "tau: " << tau.transpose() << "\n";
        std::cout << "body_acc: " << body_acc.x <<" "<< body_acc.y <<" "<< body_acc.z <<"\n";
        std::cout << "disturbance (body):  " << esti_x(12) <<" "<< esti_x(13) <<" "
                  << esti_x(14) <<" "<< esti_x(15) <<" "<< esti_x(16) <<" "<< esti_x(17) <<"\n";
        std::cout << "disturbance (world): " << wf_disturbance.transpose() <<"\n";
        cout_counter = 0;
    } else { cout_counter++; }
}

// ─────────────────────────────────────────────────────────────────────────────
// solve  —  MPC solution + thrust publication
// ─────────────────────────────────────────────────────────────────────────────
void BLUEROV2HEAVY_DOB::solve()
{
    // ── Yaw accumulation (unwrapping) — unchanged logic ───────────────────────
    if (pre_yaw >= 0 && local_euler.psi >= 0)
        yaw_diff = local_euler.psi - pre_yaw;
    else if (pre_yaw >= 0 && local_euler.psi < 0)
        yaw_diff = (2*M_PI+local_euler.psi-pre_yaw >= pre_yaw+std::abs(local_euler.psi))
                   ? -(pre_yaw + std::abs(local_euler.psi))
                   : 2*M_PI + local_euler.psi - pre_yaw;
    else if (pre_yaw < 0 && local_euler.psi >= 0)
        yaw_diff = (2*M_PI-local_euler.psi+pre_yaw >= std::abs(pre_yaw)+local_euler.psi)
                   ? std::abs(pre_yaw)+local_euler.psi
                   : -(2*M_PI-local_euler.psi+pre_yaw);
    else
        yaw_diff = local_euler.psi - pre_yaw;

    yaw_sum += yaw_diff;
    pre_yaw  = local_euler.psi;

    // ── Set initial state (12-dim: pos + body vel) ────────────────────────────
    acados_in.x0[0]  = local_pos.x;
    acados_in.x0[1]  = local_pos.y;
    acados_in.x0[2]  = local_pos.z;
    acados_in.x0[3]  = local_euler.phi;
    acados_in.x0[4]  = local_euler.theta;
    acados_in.x0[5]  = yaw_sum;
    acados_in.x0[6]  = v_linear_body[0];
    acados_in.x0[7]  = v_linear_body[1];
    acados_in.x0[8]  = v_linear_body[2];
    acados_in.x0[9]  = v_angular_body[0];
    acados_in.x0[10] = v_angular_body[1];
    acados_in.x0[11] = v_angular_body[2];
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                                  mpc_capsule->nlp_in, mpc_capsule->nlp_out, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                                  mpc_capsule->nlp_in, mpc_capsule->nlp_out, 0, "ubx", acados_in.x0);

    // ── Set disturbance parameters (6-DOF, all compensated) ──────────────────
    // Unlike the standard model which only compensated x,y,z,psi,
    // the heavy model compensates ALL 6 DOF including roll (Kw) and pitch (Mw).
    for (int i = 0; i <= BLUEROV2HEAVY_N; i++) {
        if (!COMPENSATE_D) {
            acados_param[i][0] = solver_param.disturbance_x;
            acados_param[i][1] = solver_param.disturbance_y;
            acados_param[i][2] = solver_param.disturbance_z;
            acados_param[i][3] = solver_param.disturbance_phi;    // roll
            acados_param[i][4] = solver_param.disturbance_theta;  // pitch
            acados_param[i][5] = solver_param.disturbance_psi;    // yaw
        } else {
            // EKF estimates esti_x(12..17) are body-frame disturbance forces/torques [N, Nm]
            // Divide by rotor_constant to convert to thruster-equivalent input units
            // (same scaling convention as the standard DOB)
            acados_param[i][0] = esti_x(12) / rotor_constant;  // Xw
            acados_param[i][1] = esti_x(13) / rotor_constant;  // Yw
            acados_param[i][2] = esti_x(14) / rotor_constant;  // Zw
            acados_param[i][3] = esti_x(15) / rotor_constant;  // Kw (roll)  — NEW
            acados_param[i][4] = esti_x(16) / rotor_constant;  // Mw (pitch) — NEW
            acados_param[i][5] = esti_x(17) / rotor_constant;  // Nw (yaw)
        }
        bluerov2heavy_acados_update_params(mpc_capsule, i, acados_param[i], BLUEROV2HEAVY_NP);
    }

    // ── Set reference trajectory ──────────────────────────────────────────────
    if (sin(acados_in.yref[0][5]) >= 0)
        yaw_ref =  fmod(acados_in.yref[0][5],  M_PI);
    else
        yaw_ref = -M_PI + fmod(acados_in.yref[0][5], M_PI);

    ref_cb(line_number);
    line_number++;
    for (unsigned int i = 0; i <= BLUEROV2HEAVY_N; i++)
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                               mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);

    // ── Solve OCP ─────────────────────────────────────────────────────────────
    acados_status = bluerov2heavy_acados_solve(mpc_capsule);
    if (acados_status != 0)
        ROS_WARN_STREAM("bluerov2heavy_acados_solve status: " << acados_status);

    acados_out.status   = acados_status;
    acados_out.kkt_res  = (double)mpc_capsule->nlp_out->inf_norm_res;
    ocp_nlp_get(mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);
    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                    mpc_capsule->nlp_out, 0, "u", (void*)acados_out.u0);

    // =========================================================================
    // THRUST PUBLICATION  —  CRITICAL CHANGE vs standard model
    // =========================================================================
    // Standard: u0[0..3] = generalised forces → converted via inverse allocation:
    //   thrust0 = (-u0[0]+u0[1]+u0[3]) / rotor_constant
    //
    // Heavy: u0[0..7] = DIRECT thruster forces [N] from ACADOS (K is already
    //        baked into the model). Just scale by rotor_constant.
    thrust0.data = acados_out.u0[0] / rotor_constant;
    thrust1.data = acados_out.u0[1] / rotor_constant;
    thrust2.data = acados_out.u0[2] / rotor_constant;
    thrust3.data = acados_out.u0[3] / rotor_constant;
    thrust4.data = acados_out.u0[4] / rotor_constant;
    thrust5.data = acados_out.u0[5] / rotor_constant;
    thrust6.data = acados_out.u0[6] / rotor_constant;   // NEW
    thrust7.data = acados_out.u0[7] / rotor_constant;   // NEW

    thrust0_pub.publish(thrust0);
    thrust1_pub.publish(thrust1);
    thrust2_pub.publish(thrust2);
    thrust3_pub.publish(thrust3);
    thrust4_pub.publish(thrust4);
    thrust5_pub.publish(thrust5);
    thrust6_pub.publish(thrust6);   // NEW
    thrust7_pub.publish(thrust7);   // NEW

    // ── Reference pose publish ────────────────────────────────────────────────
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw_ref);
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    ref_pose.pose.pose.position.x  = acados_in.yref[0][0];
    ref_pose.pose.pose.position.y  = acados_in.yref[0][1];
    ref_pose.pose.pose.position.z  = acados_in.yref[0][2];
    ref_pose.pose.pose.orientation = quat_msg;
    ref_pose.header.stamp          = ros::Time::now();
    ref_pose.header.frame_id       = "odom_frame";
    ref_pose.child_frame_id        = "base_link";
    ref_pose_pub.publish(ref_pose);

    // ── Error pose publish ────────────────────────────────────────────────────
    tf2::Quaternion quat_error;
    yaw_error = yaw_sum - acados_in.yref[0][5];
    quat_error.setRPY(0, 0, yaw_error);
    geometry_msgs::Quaternion quat_error_msg;
    tf2::convert(quat_error, quat_error_msg);
    error_pose.pose.pose.position.x  = acados_in.x0[0] - acados_in.yref[0][0];
    error_pose.pose.pose.position.y  = acados_in.x0[1] - acados_in.yref[0][1];
    error_pose.pose.pose.position.z  = acados_in.x0[2] - acados_in.yref[0][2];
    error_pose.pose.pose.orientation = quat_error_msg;
    error_pose.header.stamp          = ros::Time::now();
    error_pose.header.frame_id       = "odom_frame";
    error_pose.child_frame_id        = "base_link";
    error_pose_pub.publish(error_pose);

    // ── Control input echo (8 channels) ──────────────────────────────────────
    control_input0.data = acados_out.u0[0];
    control_input1.data = acados_out.u0[1];
    control_input2.data = acados_out.u0[2];
    control_input3.data = acados_out.u0[3];
    control_input4.data = acados_out.u0[4];   // NEW
    control_input5.data = acados_out.u0[5];   // NEW
    control_input6.data = acados_out.u0[6];   // NEW
    control_input7.data = acados_out.u0[7];   // NEW
    control_input0_pub.publish(control_input0);
    control_input1_pub.publish(control_input1);
    control_input2_pub.publish(control_input2);
    control_input3_pub.publish(control_input3);
    control_input4_pub.publish(control_input4);
    control_input5_pub.publish(control_input5);
    control_input6_pub.publish(control_input6);
    control_input7_pub.publish(control_input7);
}

// ─────────────────────────────────────────────────────────────────────────────
// RK4  —  4th-order Runge–Kutta integration (18-dim state unchanged)
// ─────────────────────────────────────────────────────────────────────────────
MatrixXd BLUEROV2HEAVY_DOB::RK4(MatrixXd x, MatrixXd u)
{
    Matrix<double,18,1> k1 = f(x,       u) * dt;
    Matrix<double,18,1> k2 = f(x+k1/2,  u) * dt;
    Matrix<double,18,1> k3 = f(x+k2/2,  u) * dt;
    Matrix<double,18,1> k4 = f(x+k3,    u) * dt;
    return x + (k1 + 2*k2 + 2*k3 + k4) / 6;
}

// ─────────────────────────────────────────────────────────────────────────────
// f  —  augmented system dynamics (18-dim: 12 kinematic + 6 disturbance)
//
// CHANGES vs standard:
//  - K is now 6×8; u is 8×1  →  KAu = K*u is still 6×1 (no dimension change)
//  - All 6 DOF have linear AND quadratic damping (standard had none)
//  - Heave effective mass M_z is piecewise on x(8) [heave velocity]
//  - buoyancy = 6.675 N (was 1.962 N)
// ─────────────────────────────────────────────────────────────────────────────
MatrixXd BLUEROV2HEAVY_DOB::f(MatrixXd x, MatrixXd u)
{
    Matrix<double,18,1> xdot;
    KAu = K * u;    // 6×1 generalised force

    // ── Piecewise heave effective mass ────────────────────────────────────────
    // x(8) is heave velocity w; dive (w<0) has higher added mass
    double Z_wdot_rt = (x(8) < 0.0) ? 34.60 : 22.45;
    double M_z_rt    = mass + Z_wdot_rt;

    // ── Piecewise heave linear damping ────────────────────────────────────────
    double Z_w_rt  = (x(8) < 0.0) ? -49.50 : -50.62;
    double Z_ww_rt = (x(8) < 0.0) ? -113.86: -76.69;

    // Standard effective masses (non-piecewise DOFs)
    double M_x   = mass + added_mass[0];  // 22.27
    double M_y   = mass + added_mass[1];  // 36.36
    double M_phi = Ix   + added_mass[3];  // 0.9601
    double M_th  = Iy   + added_mass[4];  // 1.120
    double M_psi = Iz   + added_mass[5];  // 1.120

    xdot <<
    // ── Kinematics (positions) — identical to standard ────────────────────────
    (cos(x(5))*cos(x(4)))*x(6)
        +(-sin(x(5))*cos(x(3))+cos(x(5))*sin(x(4))*sin(x(3)))*x(7)
        +(sin(x(5))*sin(x(3))+cos(x(5))*cos(x(3))*sin(x(4)))*x(8),
    (sin(x(5))*cos(x(4)))*x(6)
        +(cos(x(5))*cos(x(3))+sin(x(3))*sin(x(4))*sin(x(5)))*x(7)
        +(-cos(x(5))*sin(x(3))+sin(x(4))*sin(x(5))*cos(x(3)))*x(8),
    (-sin(x(4)))*x(6) + (cos(x(4))*sin(x(3)))*x(7) + (cos(x(4))*cos(x(3)))*x(8),
    x(9)+(sin(x(3))*sin(x(4))/cos(x(4)))*x(10)+(cos(x(3))*sin(x(4))/cos(x(4)))*x(11),
    cos(x(3))*x(10) - sin(x(3))*x(11),
    (sin(x(3))/cos(x(4)))*x(10) + (cos(x(3))/cos(x(4)))*x(11),

    // ── Translational dynamics — CHANGED: damping added, piecewise M_z ────────
    // du:  surge
    (KAu(0) + mass*x(11)*x(7) - mass*x(10)*x(8)
             - buoyancy*sin(x(4))
             + x(12)                        // disturbance Xw
             + Dl[0]*x(6)                   // linear damping  (Dl[0]=-38.95)
             + Dnl[0]*std::abs(x(6))*x(6)  // quadratic damping
    ) / M_x,

    // dv:  sway
    (KAu(1) - mass*x(11)*x(6) + mass*x(9)*x(8)
             + buoyancy*cos(x(4))*sin(x(3))
             + x(13)                        // Yw
             + Dl[1]*x(7)
             + Dnl[1]*std::abs(x(7))*x(7)
    ) / M_y,

    // dw:  heave  (piecewise M_z and Z_w)
    (KAu(2) + mass*x(10)*x(6) - mass*x(9)*x(7)
             + buoyancy*cos(x(4))*cos(x(3))
             + x(14)                        // Zw
             + Z_w_rt*x(8)
             + Z_ww_rt*std::abs(x(8))*x(8)
    ) / M_z_rt,

    // ── Rotational dynamics — CHANGED: full damping, buoyancy restoring ────────
    // dp:  roll  (heavy: Kw estimated and compensated)
    (KAu(3) + (Iy-Iz)*x(10)*x(11)
             - mass*ZG*g*cos(x(4))*sin(x(3))
             + x(15)                        // Kw
             + Dl[3]*x(9)                   // K_p = 0 (experimentally zero)
             + Dnl[3]*std::abs(x(9))*x(9)
    ) / M_phi,

    // dq:  pitch
    (KAu(4) + (Iz-Ix)*x(9)*x(11)
             - mass*ZG*g*sin(x(4))
             + x(16)                        // Mw
             + Dl[4]*x(10)
             + Dnl[4]*std::abs(x(10))*x(10)
    ) / M_th,

    // dr:  yaw
    (KAu(5) - (Iy-Ix)*x(9)*x(10)
             + x(17)                        // Nw
             + Dl[5]*x(11)
             + Dnl[5]*std::abs(x(11))*x(11)
    ) / M_psi,

    // ── Disturbance model: ẇ = 0 (slow time-varying assumption) ──────────────
    0, 0, 0, 0, 0, 0;

    return xdot;
}

// ─────────────────────────────────────────────────────────────────────────────
// h  —  measurement model  z = h(x)
//
// z = [η(6), v_body(6), τ(6)]
// The τ back-calculation inverts the Newton–Euler equations:
//   τ = M*v̇ + C(v)v + D(v)v + g(η) - w
// Here expressed as: τ ≈ M*body_acc + C-terms + D-terms + g - w
//
// CHANGES vs standard: full damping, piecewise M_z, buoyancy=6.675 N
// ─────────────────────────────────────────────────────────────────────────────
MatrixXd BLUEROV2HEAVY_DOB::h(MatrixXd x)
{
    Matrix<double,18,1> y;

    double Z_wdot_rt = (x(8) < 0.0) ? 34.60 : 22.45;
    double M_z_rt    = mass + Z_wdot_rt;
    double Z_w_rt    = (x(8) < 0.0) ? -49.50 : -50.62;
    double Z_ww_rt   = (x(8) < 0.0) ? -113.86 : -76.69;

    double M_x   = mass + added_mass[0];
    double M_y   = mass + added_mass[1];
    double M_phi = Ix   + added_mass[3];
    double M_th  = Iy   + added_mass[4];
    double M_psi = Iz   + added_mass[5];

    y <<
    // Pose and velocity measurements (trivially identity)
    x(0), x(1), x(2), x(3), x(4), x(5),
    x(6), x(7), x(8), x(9), x(10), x(11),

    // τ back-calculation for each DOF:
    //  τ_i = M_i * acc_i - Coriolis_i - g_i - D_i - w_i
    // (rearranged to match measurement: measured_tau = model prediction)

    // X (surge)
    M_x*body_acc.x - mass*x(11)*x(7) + mass*x(10)*x(8)
        + buoyancy*sin(x(4))
        - x(12)                        // subtract disturbance Xw
        - Dl[0]*x(6) - Dnl[0]*std::abs(x(6))*x(6),

    // Y (sway)
    M_y*body_acc.y + mass*x(11)*x(6) - mass*x(9)*x(8)
        - buoyancy*cos(x(4))*sin(x(3))
        - x(13)
        - Dl[1]*x(7) - Dnl[1]*std::abs(x(7))*x(7),

    // Z (heave, piecewise)
    M_z_rt*body_acc.z - mass*x(10)*x(6) + mass*x(9)*x(7)
        - buoyancy*cos(x(4))*cos(x(3))
        - x(14)
        - Z_w_rt*x(8) - Z_ww_rt*std::abs(x(8))*x(8),

    // K (roll)
    M_phi*body_acc.phi - (Iy-Iz)*x(10)*x(11)
        + mass*ZG*g*cos(x(4))*sin(x(3))
        - x(15)
        - Dl[3]*x(9) - Dnl[3]*std::abs(x(9))*x(9),

    // M (pitch)
    M_th*body_acc.theta - (Iz-Ix)*x(9)*x(11)
        + mass*ZG*g*sin(x(4))
        - x(16)
        - Dl[4]*x(10) - Dnl[4]*std::abs(x(10))*x(10),

    // N (yaw)
    M_psi*body_acc.psi + (Iy-Ix)*x(9)*x(10)
        - x(17)
        - Dl[5]*x(11) - Dnl[5]*std::abs(x(11))*x(11);

    return y;
}

// ─────────────────────────────────────────────────────────────────────────────
// Jacobians — numerical finite-difference (unchanged method, same dimensions)
// ─────────────────────────────────────────────────────────────────────────────
MatrixXd BLUEROV2HEAVY_DOB::compute_jacobian_F(MatrixXd x, MatrixXd u)
{
    Matrix<double,18,18> F;
    const double d = 1e-6;
    VectorXd f0 = RK4(x, u);
    for (int i = 0; i < n; i++) {
        VectorXd x1 = x; x1(i) += d;
        F.col(i) = (RK4(x1, u) - f0) / d;
    }
    return F;
}

MatrixXd BLUEROV2HEAVY_DOB::compute_jacobian_H(MatrixXd x)
{
    Matrix<double,18,18> H;
    const double d = 1e-6;
    VectorXd f0 = h(x);
    for (int i = 0; i < n; i++) {
        VectorXd x1 = x; x1(i) += d;
        H.col(i) = (h(x1) - f0) / d;
    }
    return H;
}

// ─────────────────────────────────────────────────────────────────────────────
// Coriolis, damping, restoring helpers (used externally if needed)
// ─────────────────────────────────────────────────────────────────────────────
MatrixXd BLUEROV2HEAVY_DOB::dynamics_C(MatrixXd v)
{
    Matrix<double,6,6> C;
    C << 0, 0, 0, 0, mass*v(2)+added_mass[2]*v(2), -mass*v(1)+added_mass[1]*v(1),
         0, 0, 0, -mass*v(2)-added_mass[2]*v(2), 0, mass*v(0)-added_mass[0]*v(0),
         0, 0, 0, mass*v(1)-added_mass[1]*v(1), -mass*v(0)+added_mass[0]*v(0), 0,
         0, mass*v(2)-added_mass[2]*v(2), -mass*v(1)+added_mass[1]*v(1),
            0, Iz*v(5)-added_mass[5]*v(5), -Iy*v(4)+added_mass[4]*v(4),
         -mass*v(2)+added_mass[2]*v(2), 0, mass*v(0)-added_mass[0]*v(0),
            -Iz*v(5)+added_mass[5]*v(5), 0, Ix*v(3)-added_mass[3]*v(3),
         mass*v(1)-added_mass[1]*v(1), -mass*v(0)+added_mass[0]*v(0), 0,
            Iy*v(4)-added_mass[4]*v(4), -Ix*v(3)+added_mass[3]*v(3), 0;
    return C;
}

MatrixXd BLUEROV2HEAVY_DOB::dynamics_D(MatrixXd v)
{
    // Full linear + quadratic damping for all 6 DOF
    double Z_w_rt  = (v(2) < 0.0) ? -49.50 : -50.62;
    double Z_ww_rt = (v(2) < 0.0) ? -113.86 : -76.69;

    Matrix<double,1,6> D_diag;
    D_diag << Dl[0] + Dnl[0]*std::abs(v(0)),
               Dl[1] + Dnl[1]*std::abs(v(1)),
               Z_w_rt + Z_ww_rt*std::abs(v(2)),
               Dl[3] + Dnl[3]*std::abs(v(3)),
               Dl[4] + Dnl[4]*std::abs(v(4)),
               Dl[5] + Dnl[5]*std::abs(v(5));
    Matrix<double,6,6> D = D_diag.asDiagonal();
    return D;
}

MatrixXd BLUEROV2HEAVY_DOB::dynamics_g(MatrixXd euler)
{
    // buoyancy = 6.675 N (was 1.962 in standard)
    Matrix<double,6,1> gvec;
    gvec <<  buoyancy*sin(euler(1)),
            -buoyancy*cos(euler(1))*sin(euler(0)),
            -buoyancy*cos(euler(1))*cos(euler(0)),
             mass*ZG*g*cos(euler(1))*sin(euler(0)),
             mass*ZG*g*sin(euler(1)),
             0;
    return gvec;
}

// ─────────────────────────────────────────────────────────────────────────────
// applyBodyWrench, readDataFromFile, ref_cb, q2rpy, rpy2q
// — identical logic to bluerov2_dob.cpp; only body_name changes
// ─────────────────────────────────────────────────────────────────────────────
void BLUEROV2HEAVY_DOB::applyBodyWrench()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distribution(0.5, 1.0);

    if (READ_WRENCH == 0) {
        if (dis_time > periodic_counter * M_PI) {
            amplitudeScalingFactor_X = distribution(gen) * 6;
            amplitudeScalingFactor_Y = distribution(gen) * 6;
            amplitudeScalingFactor_Z = distribution(gen) * 6;
            amplitudeScalingFactor_N = distribution(gen) * 6;
            periodic_counter++;
        }
        applied_wrench.fx = sin(dis_time) * amplitudeScalingFactor_X;
        applied_wrench.fy = sin(dis_time) * amplitudeScalingFactor_Y;
        applied_wrench.fz = sin(dis_time) * amplitudeScalingFactor_Z;
        applied_wrench.tz = sin(dis_time) * amplitudeScalingFactor_N / 3.0;
        dis_time += dt * 2.5;
    } else if (READ_WRENCH == 1) {
        applied_wrench.fx = 10;
        applied_wrench.fy = 10;
        applied_wrench.fz = 10;
        applied_wrench.tz = 0;
    }

    // ── ROS service call ─────────────────────────────────────────────────────
    body_wrench.request.body_name       = "bluerov2heavy/base_link";  // CHANGED
    body_wrench.request.start_time      = ros::Time(0.0);
    body_wrench.request.reference_frame = "world";
    body_wrench.request.duration        = ros::Duration(1000);
    body_wrench.request.reference_point.x = 0.0;
    body_wrench.request.reference_point.y = 0.0;
    body_wrench.request.reference_point.z = 0.0;
    body_wrench.request.wrench.force.x  = applied_wrench.fx;
    body_wrench.request.wrench.force.y  = applied_wrench.fy;
    body_wrench.request.wrench.force.z  = applied_wrench.fz;
    body_wrench.request.wrench.torque.x = applied_wrench.tx;
    body_wrench.request.wrench.torque.y = applied_wrench.ty;
    body_wrench.request.wrench.torque.z = applied_wrench.tz;
    client.call(body_wrench);
}

int BLUEROV2HEAVY_DOB::readDataFromFile(
    const char* fileName, std::vector<std::vector<double>>& data)
{
    std::ifstream file(fileName);
    if (!file.is_open()) { std::cout << "file not open\n"; return 0; }
    std::string line;
    int n_lines = 0;
    while (std::getline(file, line)) {
        n_lines++;
        std::istringstream ss(line);
        std::vector<double> row;
        double num;
        while (ss >> num) row.push_back(num);
        data.push_back(row);
    }
    file.close();
    return n_lines;
}

void BLUEROV2HEAVY_DOB::ref_cb(int line_to_read)
{
    if (BLUEROV2HEAVY_N + line_to_read + 1 <= number_of_steps) {
        for (unsigned int i = 0; i <= BLUEROV2HEAVY_N; i++)
            for (unsigned int j = 0; j <= BLUEROV2HEAVY_NY; j++)
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
    } else if (line_to_read < number_of_steps) {
        for (unsigned int i = 0; i < (unsigned)(number_of_steps-line_to_read); i++)
            for (unsigned int j = 0; j <= BLUEROV2HEAVY_NY; j++)
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
        for (unsigned int i = number_of_steps-line_to_read; i <= BLUEROV2HEAVY_N; i++)
            for (unsigned int j = 0; j <= BLUEROV2HEAVY_NY; j++)
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
    } else {
        for (unsigned int i = 0; i <= BLUEROV2HEAVY_N; i++)
            for (unsigned int j = 0; j <= BLUEROV2HEAVY_NY; j++)
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
    }
}