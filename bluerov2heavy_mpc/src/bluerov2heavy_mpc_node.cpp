#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>            // added
#include <algorithm>          // for std::min, std::max
#include <cstring>           // for memcpy

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "bluerov2heavy_model/bluerov2heavy_model.h"
#include "acados_solver_bluerov2heavy.h"
bool start_sub;
class NMPC
    {
        private:

        //This part is new:  now we add the logging to be able to plot
        
        // --- NEW LOGGING VARIABLES ---
        std::ofstream log_file;
        double prev_predicted_state[BLUEROV2HEAVY_NX] = {0};
        bool first_log_step = true;
        ros::Time start_time;
        // -----------------------------

        // The old ones:
        float yaw_sum = 0;      // yaw degree as continous number
        float pre_yaw = 0;      // former state yaw degree
        float yaw_diff;         // yaw degree difference in every step
        float yaw_ref;          // yaw degree reference in form of (-pi, pi)
        float yaw_error;        // yaw degree error
        

        enum SystemStates{
            x = 0,
            y = 1,
            z = 2,
            phi = 3,
            theta = 4,
            psi = 5,
            u = 6,
            v = 7,
            w = 8,
            p = 9,
            q = 10,
            r = 11,
        };

        enum ControlInputs{
            u1 = 0,
            u2 = 1,
            u3 = 2,
            u4 = 3,
            u5 = 4,
            u6 = 5,
        };

        struct SolverInput{
            double x0[BLUEROV2HEAVY_NX];                      // changed macro
            double yref[BLUEROV2HEAVY_N+1][BLUEROV2HEAVY_NY]; // changed macros
        };

        struct SolverOutput{
            double u0[BLUEROV2HEAVY_NU];  // changed macro
            double x1[BLUEROV2HEAVY_NX];  // changed macro
            double status, kkt_res, cpu_time;
        };

        struct Euler{
            double phi;
            double theta;
            double psi;
        };

        // ROS subscriber and publisher
        ros::Subscriber pose_gt_sub;
        ros::Subscriber trajectory_sub;  // subscriber for MultiDOFJointTrajectory

        ros::Publisher thrust0_pub;
        ros::Publisher thrust1_pub;
        ros::Publisher thrust2_pub;
        ros::Publisher thrust3_pub;
        ros::Publisher thrust4_pub;
        ros::Publisher thrust5_pub;
        ros::Publisher thrust6_pub;
        ros::Publisher thrust7_pub;

        //ros::Publisher marker_pub;

        ros::Publisher ref_pose_pub;

        ros::Publisher error_pose_pub;

        // Visualization publishers for MPC predictions
        ros::Publisher mpc_trajectory_pub;          // Path visualization
        ros::Publisher mpc_markers_pub;             // Marker array for predicted poses

        // Visualization publishers for reference trajectory
        ros::Publisher ref_trajectory_pub;          // Reference path visualization
        ros::Publisher ref_markers_pub;             // Marker array for reference poses

        ros::Publisher control_input0_pub;
        ros::Publisher control_input1_pub;
        ros::Publisher control_input2_pub;
        ros::Publisher control_input3_pub;
        ros::Publisher control_input4_pub;
        ros::Publisher control_input5_pub;

        // ROS message variables
        nav_msgs::Odometry pose_gt,pose_hold;
        Euler local_euler;

        trajectory_msgs::MultiDOFJointTrajectory received_trajectory;  // received reference trajectory
        bool trajectory_received = false;  // flag to track if new trajectory received
        
        // Reference mode: 0 = file-based (readtxt), 1 = message-based (receive_ref)
        int ref_mode = 0;
        std::string ref_mode_str = "readtxt";  // default mode
        
        // Stored trajectory for message-based mode (with timestamps)
        std::vector<double> stored_trajectory_times;  // timestamps for each stored point
        std::vector<std::vector<double>> stored_trajectory_states;  // [x,y,z,phi,theta,psi,u,v,w,p,q,r]
        ros::Time trajectory_start_time;  // when trajectory was received/started
        bool trajectory_available = false;  // whether we have a valid stored trajectory
        bool receive_odom = false; //
        bool pose_hold_initialized = false;  // flag to track if pose_hold has been initialized

        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust6;
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrust7;

        
        //visualization_msgs::Marker marker;

        nav_msgs::Odometry ref_pose;

        nav_msgs::Odometry error_pose;

        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input0;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input1;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input2;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input3;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input4;
        uuv_gazebo_ros_plugins_msgs::FloatStamped control_input5;

        // Acados variables
        SolverInput acados_in;
        SolverOutput acados_out;
        int acados_status;   
        bluerov2heavy_solver_capsule * mpc_capsule = bluerov2heavy_acados_create_capsule();
        
        // Trajectory variables
        std::vector<std::vector<double>> trajectory;
        int line_number = 0;
        int number_of_steps = 0;

        // Other variables
        tf::Quaternion tf_quaternion;
        int cout_counter = 0;
        double logger_time;

        public:
        
        // This part is also new:
        ~NMPC()
        {
            if(log_file.is_open()){
                log_file.close();
                ROS_INFO("MPC Log file saved and closed.");
            }
        }


        NMPC(ros::NodeHandle& nh, const std::string& ref_traj, const std::string& mode = "readtxt")
        {

            // This part is new too:
            // --- NEW LOGGER INITIALIZATION ---
            log_file.open("/home/dorsa/rov_mpc_log.csv");
            if (log_file.is_open()) {
                log_file << "time,"
                         << "ref_x,ref_y,ref_z,ref_phi,ref_theta,ref_psi,"
                         << "real_x,real_y,real_z,real_phi,real_theta,real_psi,"
                         << "pred_x,pred_y,pred_z,pred_phi,pred_theta,pred_psi,"
                         << "u1,u2,u3,u4,u5,u6,u7,u8\n";
            } else {
                ROS_WARN("Failed to open log file at /tmp/rov_mpc_log.csv");
            }
            first_log_step = true;
            // ---------------------------------


            // Set reference mode
            ref_mode_str = mode;
            if (mode == "receive_ref")
            {
                ref_mode = 1;
                ROS_INFO("MPC Reference Mode: RECEIVE_REF (message-based)");
            }
            else
            {
                ref_mode = 0;
                ROS_INFO("MPC Reference Mode: READTXT (file-based)");
            }

            // Pre-load the trajectory (only needed for file-based mode)
            if (ref_mode == 0)
            {
                const char * c = ref_traj.c_str();
                number_of_steps = readDataFromFile(c, trajectory);
                if (number_of_steps == 0){
                    ROS_WARN("Cannot load CasADi optimal trajectory!");
                }
                else{
                    ROS_INFO_STREAM("Number of steps of selected trajectory: " << number_of_steps << std::endl);
                }
            }

            // Initialize MPC
            int create_status = 1;
            create_status = bluerov2heavy_acados_create(mpc_capsule);
            if (create_status != 0){
                ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
                exit(1);
            }

            // ROS Subscriber & Publisher
            pose_gt_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2heavy/pose_gt", 20, &NMPC::pose_gt_cb, this);
            
            // Subscribe to trajectory only in receive_ref mode
            if (ref_mode == 1)
            {
                trajectory_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
                    "/gcopter/traj_enu", 1, &NMPC::trajectory_cb, this);
            }
            
            thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/0/input",20);
            thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/1/input",20);
            thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/2/input",20);
            thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/3/input",20);
            thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/4/input",20);
            thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/5/input",20);
            thrust6_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/6/input",20);
            thrust7_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/thrusters/7/input",20);
            //marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 20);
            ref_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2heavy/mpc/reference",20);
            error_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2heavy/mpc/error",20);
            
            // Initialize visualization publishers
            mpc_trajectory_pub = nh.advertise<nav_msgs::Path>("/bluerov2heavy/mpc/predicted_trajectory", 1);
            mpc_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/bluerov2heavy/mpc/predicted_poses", 1);
            
            // Initialize reference trajectory publishers
            ref_trajectory_pub = nh.advertise<nav_msgs::Path>("/bluerov2heavy/mpc/reference_trajectory", 1);
            ref_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/bluerov2heavy/mpc/reference_poses", 1);
            
            control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/0",20);
            control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/1",20);
            control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/2",20);
            control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/3",20);
            control_input4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/4",20);
            control_input5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2heavy/control_input/5",20);
            // Initialize
            for(unsigned int i=0; i < BLUEROV2HEAVY_NU; i++) acados_out.u0[i] = 0.0;
            for(unsigned int i=0; i < BLUEROV2HEAVY_NX; i++) acados_in.x0[i] = 0.0;

        }
        void pose_gt_cb(const nav_msgs::Odometry::ConstPtr& pose)
        {
            start_sub = true;
            receive_odom = true;

            // ---------------- Position (world frame) ----------------
            pose_gt.pose.pose.position = pose->pose.pose.position;

            // ---------------- Orientation ----------------
            tf::Quaternion q_wb;  // body -> world rotation
            tf::quaternionMsgToTF(pose->pose.pose.orientation, q_wb);

            // Euler (optional)
            tf::Matrix3x3(q_wb).getRPY(local_euler.phi,
                                    local_euler.theta,
                                    local_euler.psi);

            // Rotation matrix world -> body
            tf::Matrix3x3 R_bw(q_wb.inverse());

            // ---------------- Linear velocity ----------------
            tf::Vector3 vel_world(pose->twist.twist.linear.x,
                                pose->twist.twist.linear.y,
                                pose->twist.twist.linear.z);

            tf::Vector3 vel_body = R_bw * vel_world;

            pose_gt.twist.twist.linear.x = vel_body.x();   // u
            pose_gt.twist.twist.linear.y = vel_body.y();   // v
            pose_gt.twist.twist.linear.z = vel_body.z();   // w

            // ---------------- Angular velocity ----------------
            tf::Vector3 ang_world(pose->twist.twist.angular.x,
                                pose->twist.twist.angular.y,
                                pose->twist.twist.angular.z);

            tf::Vector3 ang_body = R_bw * ang_world;

            pose_gt.twist.twist.angular.x = ang_body.x();  // p
            pose_gt.twist.twist.angular.y = ang_body.y();  // q
            pose_gt.twist.twist.angular.z = ang_body.z();  // r

            // ---------------- Pose hold logic ----------------
            if (!pose_hold_initialized)
            {
                pose_hold = pose_gt;
                pose_hold_initialized = true;
            }
            else
            {
                double dx = pose_gt.pose.pose.position.x - pose_hold.pose.pose.position.x;
                double dy = pose_gt.pose.pose.position.y - pose_hold.pose.pose.position.y;
                double dz = pose_gt.pose.pose.position.z - pose_hold.pose.pose.position.z;
                double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

                if (distance > 0.2)
                {
                    pose_hold = pose_gt;
                }
            }
        }
        void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
        {
            received_trajectory = *msg;
            trajectory_received = true;
            ROS_INFO_STREAM("Received reference trajectory with " << msg->points.size() << " points");
        }

        void fillReferenceFromMessage()
        {
            // 6-DOF trajectory: [x, y, z, phi, theta, psi] + velocities if available
            const int horizon = BLUEROV2HEAVY_N + 1;  // 101 points
            const double total_time = 5.0;            // 5 seconds
            const double dt = total_time / BLUEROV2HEAVY_N;
            
            if (received_trajectory.points.empty())
            {
                ROS_WARN("Received empty trajectory!");
                trajectory_received = false;
                return;
            }

            // Extract time and state data from message
            std::vector<double> traj_times;
            std::vector<std::vector<double>> traj_states;
            
            for (const auto& point : received_trajectory.points)
            {
                double time = point.time_from_start.toSec();
                traj_times.push_back(time);

                std::vector<double> state(BLUEROV2HEAVY_NY, 0.0);
                
                // Extract 6-DOF pose directly from transforms[0]
                if (!point.transforms.empty())
                {
                    const geometry_msgs::Transform& tf = point.transforms[0];
                    
                    // Position: x, y, z
                    state[0] = tf.translation.x;
                    state[1] = tf.translation.y;
                    state[2] = tf.translation.z;
                    
                    // Orientation: Convert quaternion to Euler angles (phi, theta, psi)
                    tf::Quaternion tf_quat;
                    tf::quaternionMsgToTF(tf.rotation, tf_quat);
                    tf::Matrix3x3 mat(tf_quat);
                    mat.getRPY(state[3], state[4], state[5]);  // phi, theta, psi
                    
                    // Extract velocities if available and convert from world frame to body frame
                    if (!point.velocities.empty())
                    {
                        // Velocities in world frame
                        double vel_world_x = point.velocities[0].linear.x;
                        double vel_world_y = point.velocities[0].linear.y;
                        double vel_world_z = point.velocities[0].linear.z;
                        double ang_vel_world_x = point.velocities[0].angular.x;
                        double ang_vel_world_y = point.velocities[0].angular.y;
                        double ang_vel_world_z = point.velocities[0].angular.z;
                        
                        // Convert to body frame using R^T (transpose of rotation matrix)
                        // R represents rotation from body to world, so R^T converts world to body
                        tf::Vector3 vel_world(vel_world_x, vel_world_y, vel_world_z);
                        tf::Vector3 ang_vel_world(ang_vel_world_x, ang_vel_world_y, ang_vel_world_z);
                        
                        // Apply inverse rotation (transpose)
                        tf::Vector3 vel_body = mat.transpose() * vel_world;
                        tf::Vector3 ang_vel_body = mat.transpose() * ang_vel_world;
                        
                        // Store body frame velocities
                        state[6] = vel_body.x();   // u
                        state[7] = vel_body.y();   // v
                        state[8] = vel_body.z();   // w
                        state[9] = ang_vel_body.x();  // p
                        state[10] = ang_vel_body.y(); // q
                        state[11] = ang_vel_body.z(); // r
                    }
                }
                
                traj_states.push_back(state);
            }

            // Store the trajectory with timestamps
            stored_trajectory_times = traj_times;
            stored_trajectory_states = traj_states;
            trajectory_start_time = ros::Time::now();
            trajectory_available = true;
            
            ROS_INFO_STREAM("Stored trajectory with " << traj_states.size() << " points, duration: " 
                           << traj_times.back() << "s");
            
            trajectory_received = false;
        }

        void updateReferenceFromStoredTrajectory()
        {
            // Use current time relative to trajectory start
            ros::Time current_time = ros::Time::now();
            double elapsed_time = (current_time - trajectory_start_time).toSec();
            
            const int horizon = BLUEROV2HEAVY_N + 1;
            const double total_time = 5.0;
            const double dt = total_time / BLUEROV2HEAVY_N;
            
            // For each MPC horizon point, get the trajectory state
            for (int i = 0; i < horizon; i++)
            {
                // Time in the stored trajectory
                double target_time = elapsed_time + i * dt;
                
                // If target time is beyond stored trajectory, hold last point
                if (target_time >= stored_trajectory_times.back())
                {
                    for (unsigned int j = 0; j < BLUEROV2HEAVY_NY; j++)
                    {
                        acados_in.yref[i][j] = stored_trajectory_states.back()[j];
                    }
                }
                // If target time is before trajectory start, use first point
                else if (target_time < stored_trajectory_times.front())
                {
                    for (unsigned int j = 0; j < BLUEROV2HEAVY_NY; j++)
                    {
                        acados_in.yref[i][j] = stored_trajectory_states.front()[j];
                    }
                }
                // Otherwise, interpolate within stored trajectory
                else
                {
                    // Find the two trajectory points to interpolate between
                    size_t idx = 0;
                    for (size_t j = 0; j < stored_trajectory_times.size() - 1; j++)
                    {
                        if (target_time >= stored_trajectory_times[j] && 
                            target_time <= stored_trajectory_times[j + 1])
                        {
                            idx = j;
                            break;
                        }
                    }

                    double t0 = stored_trajectory_times[idx];
                    double t1 = stored_trajectory_times[idx + 1];
                    double alpha = (t1 - t0 < 1e-6) ? 0.0 : (target_time - t0) / (t1 - t0);

                    // Linear interpolation
                    for (unsigned int j = 0; j < BLUEROV2HEAVY_NY; j++)
                    {
                        acados_in.yref[i][j] = 
                            (1.0 - alpha) * stored_trajectory_states[idx][j] + 
                            alpha * stored_trajectory_states[idx + 1][j];
                    }
                }
            }
        }
        void updateReferenceFromcurrentodom()
        {
            // use current odometry as reference
            for (unsigned int i = 0; i <= BLUEROV2HEAVY_N; i++)  // Fill all horizon with the last point
                {
                    
                        //acados_in.yref[i][0] = trajectory[number_of_steps-1][j];
                        acados_in.yref[i][0]= pose_hold.pose.pose.position.x;
                        acados_in.yref[i][1] = pose_hold.pose.pose.position.y;
                        acados_in.yref[i][2] = pose_hold.pose.pose.position.z;
                        // acados_in.yref[i][3] = local_euler.phi;
                        // acados_in.yref[i][4] = local_euler.theta;
                        // acados_in.yref[i][5] = local_euler.psi;
                        // acados_in.yref[i][0]= 0;
                        // acados_in.yref[i][1] = 0;
                        // acados_in.yref[i][2] = 0;
                        acados_in.yref[i][3] = 0;
                        acados_in.yref[i][4] = 0;
                        acados_in.yref[i][5] = 0;
                        
                        acados_in.yref[i][6] = 0;
                        acados_in.yref[i][7] = 0;
                        acados_in.yref[i][8] = 0;
                        acados_in.yref[i][9] = 0;
                        acados_in.yref[i][10] = 0;
                        acados_in.yref[i][11] = 0;
                        acados_in.yref[i][12] = 0;
                        acados_in.yref[i][13] = 0;
                        acados_in.yref[i][14] = 0;
                        acados_in.yref[i][15] = 0;
                        acados_in.yref[i][16] = 0;
                        acados_in.yref[i][17] = 0;
                        acados_in.yref[i][18] = 0;
                        acados_in.yref[i][19] = 0;
                    
                }
            
        }
        int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
		{
		    std::ifstream file(fileName);
		    std::string line;
		    int number_of_lines = 0;

		    if (file.is_open())
			{
			    while(getline(file, line)){
				    number_of_lines++;
				    std::istringstream linestream( line );
				    std::vector<double> linedata;
				    double number;

				    while( linestream >> number ){
					    linedata.push_back( number );
				    }
				    data.push_back( linedata );
			    }

			    file.close();
			}
		    else
			{
			    return 0;
			}

		    return number_of_lines;
		}

        void ref_cb(int line_to_read)
        {
            if (BLUEROV2HEAVY_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
            {
                for (unsigned int i = 0; i <= BLUEROV2HEAVY_N; i++)  // Fill all horizon with file data
                {
                    for (unsigned int j = 0; j < BLUEROV2HEAVY_NY; j++)
                    {
                        acados_in.yref[i][j] = trajectory[i+line_to_read][j];
                    }
                }
            }
            else if(line_to_read < number_of_steps)    // Part of ref points within the file
            {
                for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
                {
                    for (unsigned int j = 0; j < BLUEROV2HEAVY_NY; j++)
                    {
                        acados_in.yref[i][j] = trajectory[i+line_to_read][j];
                    }
                }

                for (unsigned int i = number_of_steps-line_to_read; i <= BLUEROV2HEAVY_N; i++)  // Fill the rest horizon with the last point
                {
                    for (unsigned int j = 0; j < BLUEROV2HEAVY_NY; j++)
                    {
                        acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
                    }
                }
            }
            else    // none of ref points within the file
            {
                for (unsigned int i = 0; i <= BLUEROV2HEAVY_N; i++)  // Fill all horizon with the last point
                {
                    for (unsigned int j = 0; j < BLUEROV2HEAVY_NY; j++)
                    {
                        acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
                    }
                }
            }
        }

        

        void publishMPCPrediction()
        {
            // Extract predicted states from Acados and publish as visualization
            const int horizon = BLUEROV2HEAVY_N + 1;  // 101 points
            const double dt = 5.0 / BLUEROV2HEAVY_N;   // 0.02 seconds
            
            // Create path message for trajectory visualization
            nav_msgs::Path predicted_path;
            predicted_path.header.stamp = ros::Time::now();
            predicted_path.header.frame_id = "world";
            
            // Create marker array for predicted poses (with orientation)
            visualization_msgs::MarkerArray marker_array;
            
            // Extract predicted states at each horizon step
            for (int i = 0; i < horizon; i++)
            {
                // Get predicted state at step i
                double predicted_state[BLUEROV2HEAVY_NX];
                ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, 
                               mpc_capsule->nlp_out, i, "x", (void *)predicted_state);
                
                // Extract position and orientation
                double x = predicted_state[0];   // position x
                double y = predicted_state[1];   // position y
                double z = predicted_state[2];   // position z
                double phi = predicted_state[3];     // roll
                double theta = predicted_state[4];   // pitch
                double psi = predicted_state[5];     // yaw
                
                // Add to path
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time::now() + ros::Duration(i * dt);
                pose_stamped.header.frame_id = "world";
                pose_stamped.pose.position.x = x;
                pose_stamped.pose.position.y = y;
                pose_stamped.pose.position.z = z;
                
                // Convert Euler angles to quaternion
                tf::Quaternion quat;
                quat.setRPY(phi, theta, psi);
                geometry_msgs::Quaternion quat_msg;
                tf::quaternionTFToMsg(quat, quat_msg);
                pose_stamped.pose.orientation = quat_msg;
                
                predicted_path.poses.push_back(pose_stamped);
                
                // Create marker for this predicted pose (every 5th point to avoid clutter)
                if (i % 5 == 0)
                {
                    visualization_msgs::Marker pose_marker;
                    pose_marker.header.stamp = ros::Time::now();
                    pose_marker.header.frame_id = "world";
                    pose_marker.ns = "mpc_predicted_poses";
                    pose_marker.id = i / 5;
                    pose_marker.type = visualization_msgs::Marker::ARROW;
                    pose_marker.action = visualization_msgs::Marker::ADD;
                    
                    // Position
                    pose_marker.pose.position.x = x;
                    pose_marker.pose.position.y = y;
                    pose_marker.pose.position.z = z;
                    pose_marker.pose.orientation = quat_msg;
                    
                    // Scale (arrow size)
                    pose_marker.scale.x = 0.1;  // length
                    pose_marker.scale.y = 0.02;  // width
                    pose_marker.scale.z = 0.02;  // height
                    
                    // Color: gradient from green (near) to red (far)
                    float t = (float)i / (float)horizon;
                    pose_marker.color.r = t;           // Red increases
                    pose_marker.color.g = 1.0 - t;     // Green decreases
                    pose_marker.color.b = 0.0;
                    pose_marker.color.a = 0.8;
                    
                    pose_marker.lifetime = ros::Duration(0.1);  // Disappear after 100ms
                    
                    marker_array.markers.push_back(pose_marker);
                }
            }
            
            // Publish path
            mpc_trajectory_pub.publish(predicted_path);
            
            // Publish markers
            mpc_markers_pub.publish(marker_array);
        }

        void publishReferenceTrajectory()
        {
            // Publish the reference trajectory (what the controller is trying to follow)
            const int horizon = BLUEROV2HEAVY_N + 1;  // 101 points
            const double dt = 5.0 / BLUEROV2HEAVY_N;   // 0.05 seconds
            
            // Create path message for reference trajectory visualization
            nav_msgs::Path reference_path;
            reference_path.header.stamp = ros::Time::now();
            reference_path.header.frame_id = "world";  // Use 'map' frame which typically exists
            
            // Create marker array for reference poses
            visualization_msgs::MarkerArray ref_marker_array;
            
            // Extract reference states at each horizon step from acados_in.yref
            for (int i = 0; i < horizon; i++)
            {
                // Extract position and orientation from reference
                double x = acados_in.yref[i][0];        // position x
                double y = acados_in.yref[i][1];        // position y
                double z = acados_in.yref[i][2];        // position z
                double phi = acados_in.yref[i][3];      // roll
                double theta = acados_in.yref[i][4];    // pitch
                double psi = acados_in.yref[i][5];      // yaw
                
                // Add to path
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time::now() + ros::Duration(i * dt);
                pose_stamped.header.frame_id = "world";
                pose_stamped.pose.position.x = x;
                pose_stamped.pose.position.y = y;
                pose_stamped.pose.position.z = z;
                
                // Convert Euler angles to quaternion
                tf::Quaternion quat;
                quat.setRPY(phi, theta, psi);
                geometry_msgs::Quaternion quat_msg;
                tf::quaternionTFToMsg(quat, quat_msg);
                pose_stamped.pose.orientation = quat_msg;
                
                reference_path.poses.push_back(pose_stamped);
                
                // Create marker for reference pose (every 5th point to avoid clutter)
                if (i % 5 == 0)
                {
                    visualization_msgs::Marker ref_marker;
                    ref_marker.header.stamp = ros::Time::now();
                    ref_marker.header.frame_id = "world";
                    ref_marker.ns = "reference_poses";
                    ref_marker.id = i / 5;
                    ref_marker.type = visualization_msgs::Marker::ARROW;
                    ref_marker.action = visualization_msgs::Marker::ADD;
                    
                    // Position
                    ref_marker.pose.position.x = x;
                    ref_marker.pose.position.y = y;
                    ref_marker.pose.position.z = z;
                    ref_marker.pose.orientation = quat_msg;
                    
                    // Scale (arrow size) - slightly smaller than prediction for distinction
                    ref_marker.scale.x = 0.15;  // length
                    ref_marker.scale.y = 0.02;  // width
                    ref_marker.scale.z = 0.02;  // height
                    
                    // Color: gradient from blue (near) to cyan (far) - different from prediction
                    float t = (float)i / (float)horizon;
                    ref_marker.color.r = 0.0;
                    ref_marker.color.g = t;             // Green increases
                    ref_marker.color.b = 1.0 - t * 0.5; // Blue stays high
                    ref_marker.color.a = 0.7;
                    
                    ref_marker.lifetime = ros::Duration(0.1);  // Disappear after 100ms
                    
                    ref_marker_array.markers.push_back(ref_marker);
                }
            }
            
            // Publish reference path and markers
            ref_trajectory_pub.publish(reference_path);
            ref_markers_pub.publish(ref_marker_array);
        }

        void run()
        {
            if (!receive_odom || !pose_hold_initialized)
            {
                return;
            }
            
            // Update reference based on selected mode
            if (ref_mode == 1)
            {
                // Message-based mode
                if (trajectory_received)
                {
                    // New trajectory arrived - store it with timestamps
                    fillReferenceFromMessage();
                }
                else if (trajectory_available)
                {
                    // Use stored trajectory, shifted by current elapsed time
                    updateReferenceFromStoredTrajectory();
                }
                else
                {
                    // No trajectory available yet
                    updateReferenceFromcurrentodom();
                    ROS_WARN_THROTTLE(1.0, "Waiting for first trajectory message...");
                }
            }
            else
            {
                // File-based mode (existing behavior)
                // ref_cb() is called separately if needed
            }

            // identify turning direction
            if (pre_yaw >= 0 && local_euler.psi >=0)
            {
                yaw_diff = local_euler.psi - pre_yaw;
            }
            else if (pre_yaw >= 0 && local_euler.psi <0)
            {
                if (2*M_PI+local_euler.psi-pre_yaw >= pre_yaw+abs(local_euler.psi))
                {
                    yaw_diff = -(pre_yaw + abs(local_euler.psi));
                }
                else
                {
                    yaw_diff = 2 * M_PI + local_euler.psi - pre_yaw;
                }
            }
            else if (pre_yaw < 0 && local_euler.psi >= 0)
            {
                if (2*M_PI-local_euler.psi+pre_yaw >= abs(pre_yaw)+local_euler.psi)
                {
                    yaw_diff = abs(pre_yaw)+local_euler.psi;
                }
                else
                {
                    yaw_diff = -(2*M_PI-local_euler.psi+pre_yaw);
                }
            }
            else
            {
                yaw_diff = local_euler.psi - pre_yaw;
            }

            yaw_sum = yaw_sum + yaw_diff;
            pre_yaw = local_euler.psi;
            
            acados_in.x0[x] = pose_gt.pose.pose.position.x;
            acados_in.x0[y] = pose_gt.pose.pose.position.y;
            acados_in.x0[z] = pose_gt.pose.pose.position.z;
            acados_in.x0[phi] = local_euler.phi;
            acados_in.x0[theta] = local_euler.theta;
            acados_in.x0[psi] = yaw_sum;
            acados_in.x0[u] = pose_gt.twist.twist.linear.x;
            acados_in.x0[v] = pose_gt.twist.twist.linear.y;
            acados_in.x0[w] = pose_gt.twist.twist.linear.z;
            acados_in.x0[p] = pose_gt.twist.twist.angular.x;
            acados_in.x0[q] = pose_gt.twist.twist.angular.y;
            acados_in.x0[r] = pose_gt.twist.twist.angular.z;
            //test
            /*acados_in.x0[x] = 0;
            acados_in.x0[y] = 0;
            acados_in.x0[z] = -5;
            acados_in.x0[phi] = 0;
            acados_in.x0[theta] = 0;
            acados_in.x0[psi] = 0;
            acados_in.x0[u] = 0;
            acados_in.x0[v] = 0;
            acados_in.x0[w] = 0;
            acados_in.x0[p] = 0;
            acados_in.x0[q] = 0;
            acados_in.x0[r] = 0;
            */


            // change into form of (-pi, pi)
            if(sin(acados_in.yref[0][5]) >= 0)
            {
                yaw_ref = fmod(acados_in.yref[0][5],M_PI);
            }
            else{
                yaw_ref = -M_PI + fmod(acados_in.yref[0][5],M_PI);
            }

            ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, mpc_capsule->nlp_out, 0, "lbx", acados_in.x0);
            ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, mpc_capsule->nlp_out, 0, "ubx", acados_in.x0);

            // Only call ref_cb in file-based mode
            if (ref_mode == 0)
            {
                ref_cb(line_number);
                line_number++;
            }
            
            for (unsigned int i = 0; i <= BLUEROV2HEAVY_N; i++)
                {            
                ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
                }
            
            acados_status = bluerov2heavy_acados_solve(mpc_capsule);
            
            if (acados_status != 0){
                ROS_INFO_STREAM("acados returned status " << acados_status << std::endl);
            }
 
            acados_out.status = acados_status;
            acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

            ocp_nlp_get(mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

            ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);

            // Publish MPC prediction visualization
            publishMPCPrediction();
            
            // Publish reference trajectory visualization
            publishReferenceTrajectory();



            // Start of the new part in run is here: NEW LOGGING SEQUENCE ---
            if (first_log_step) {
                // Initialize start time on the very first loop iteration
                start_time = ros::Time::now();
            }

            double current_time = (ros::Time::now() - start_time).toSec();

            // Get the state predicted for the NEXT step (horizon index 1)
            double next_predicted_state[BLUEROV2HEAVY_NX] = {0};
            ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, 
                            mpc_capsule->nlp_out, 1, "x", (void *)next_predicted_state);

            if (!first_log_step && log_file.is_open()) {
                log_file << current_time << ",";
                
                // 1. Reference States (x, y, z, phi, theta, psi)
                for(int i=0; i<6; i++) log_file << acados_in.yref[0][i] << ",";
                
                // 2. Real States (Ground Truth)
                for(int i=0; i<6; i++) log_file << acados_in.x0[i] << ",";
                
                // 3. Predicted States (What we thought would happen last step)
                for(int i=0; i<6; i++) log_file << prev_predicted_state[i] << ",";
                
                // 4. Control Inputs (Thrusters 0 through 7)
                for(int i=0; i<8; i++) log_file << acados_out.u0[i] << (i==7 ? "" : ",");
                
                log_file << "\n";
            }

            // Save this step's prediction to check against reality in the next step
            memcpy(prev_predicted_state, next_predicted_state, sizeof(double)*BLUEROV2HEAVY_NX);
            first_log_step = false;
            // ----------------------------


            //rotor constant 0.026546960744430276
            /*
            thrust0.data=80/(1+exp(-4*pow((-acados_out.u0[0]+acados_out.u0[1]+acados_out.u0[3]),3)))-40;
            thrust1.data=80/(1+exp(-4*pow((-acados_out.u0[0]-acados_out.u0[1]-acados_out.u0[3]),3)))-40;
            thrust2.data=80/(1+exp(-4*pow((acados_out.u0[0]+acados_out.u0[1]-acados_out.u0[3]),3)))-40;
            thrust3.data=80/(1+exp(-4*pow((acados_out.u0[0]-acados_out.u0[1]+acados_out.u0[3]),3)))-40;
            thrust4.data=80/(1+exp(-4*pow((-acados_out.u0[2]),3)))-40;
            thrust5.data=80/(1+exp(-4*pow((-acados_out.u0[2]),3)))-40;
            */
            // publish thrusts
            /*
            thrust0.data=(acados_out.u0[0]+acados_out.u0[1]+acados_out.u0[3]);
            thrust1.data=(acados_out.u0[0]-acados_out.u0[1]-acados_out.u0[3]);
            thrust2.data=(-acados_out.u0[0]+acados_out.u0[1]-acados_out.u0[3]);
            thrust3.data=(-acados_out.u0[0]-acados_out.u0[1]+acados_out.u0[3]);
            thrust4.data=(acados_out.u0[2]-acados_out.u0[4]-acados_out.u0[5]);
            thrust5.data=(acados_out.u0[2]+acados_out.u0[4]-acados_out.u0[5]);
            thrust6.data=(acados_out.u0[2]-acados_out.u0[4]+acados_out.u0[5]);
            thrust7.data=(acados_out.u0[2]+acados_out.u0[4]+acados_out.u0[5]);
            */
            thrust0.data = acados_out.u0[0];
            thrust1.data = acados_out.u0[1];
            thrust2.data = acados_out.u0[2];
            thrust3.data = acados_out.u0[3];
            thrust4.data = acados_out.u0[4];
            thrust5.data = acados_out.u0[5];
            thrust6.data = acados_out.u0[6];
            thrust7.data = acados_out.u0[7];

            thrust0.data = thrust0.data ==0?0:100*sqrt(abs(thrust0.data))*(thrust0.data/abs(thrust0.data));
            thrust1.data = thrust1.data ==0?0:100*sqrt(abs(thrust1.data))*(thrust1.data/abs(thrust1.data));
            thrust2.data = thrust2.data ==0?0:100*sqrt(abs(thrust2.data))*(thrust2.data/abs(thrust2.data));
            thrust3.data = thrust3.data ==0?0:100*sqrt(abs(thrust3.data))*(thrust3.data/abs(thrust3.data));
            thrust4.data = thrust4.data ==0?0:100*sqrt(abs(thrust4.data))*(thrust4.data/abs(thrust4.data));
            thrust5.data = thrust5.data ==0?0:100*sqrt(abs(thrust5.data))*(thrust5.data/abs(thrust5.data));
            thrust6.data = thrust6.data ==0?0:100*sqrt(abs(thrust6.data))*(thrust6.data/abs(thrust6.data));
            thrust7.data = thrust7.data ==0?0:100*sqrt(abs(thrust7.data))*(thrust7.data/abs(thrust7.data));
            
           
           // test
            /*
            thrust0.data=0;
            thrust1.data=0;
            thrust2.data=0;
            thrust3.data=0;
            thrust4.data = 1000;
            thrust5.data = 1000;
            thrust6.data = 0;
            thrust7.data = 0;
            */
            
            
            thrust0_pub.publish(thrust0);
            thrust1_pub.publish(thrust1);
            thrust2_pub.publish(thrust2);
            thrust3_pub.publish(thrust3);
            thrust4_pub.publish(thrust4);
            thrust5_pub.publish(thrust5);
            thrust6_pub.publish(thrust6);
            thrust7_pub.publish(thrust7);

            /*
            // publish trajectory marker on rviz
            marker.header.frame_id = "map";
            marker.ns = "trajectory";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            geometry_msgs::Point current_point;
            current_point.x = acados_in.yref[0][0];
            current_point.y = acados_in.yref[0][1];
            current_point.z = acados_in.yref[0][2];
            marker.points.push_back(current_point);
            
            marker_pub.publish(marker);
            */
            // publish reference pose
            tf2::Quaternion quat;
            quat.setRPY(0, 0, yaw_ref);
            geometry_msgs::Quaternion quat_msg;
            tf2::convert(quat, quat_msg);
            ref_pose.pose.pose.position.x = acados_in.yref[0][0];
            ref_pose.pose.pose.position.y = acados_in.yref[0][1];
            ref_pose.pose.pose.position.z = acados_in.yref[0][2];
            ref_pose.pose.pose.orientation.x = quat_msg.x;
            ref_pose.pose.pose.orientation.y = quat_msg.y;
            ref_pose.pose.pose.orientation.z = quat_msg.z;
            ref_pose.pose.pose.orientation.w = quat_msg.w;
            ref_pose.header.stamp = ros::Time::now();
            ref_pose.header.frame_id = "odom_frame";
            ref_pose.child_frame_id = "base_link";

            ref_pose_pub.publish(ref_pose);

            // publish error pose
            tf2::Quaternion quat_error;
            yaw_error = yaw_sum - acados_in.yref[0][5];
            quat_error.setRPY(0, 0, yaw_error);
            geometry_msgs::Quaternion quat_error_msg;
            tf2::convert(quat_error, quat_error_msg);
            error_pose.pose.pose.position.x = acados_in.x0[0] - acados_in.yref[0][0];
            error_pose.pose.pose.position.y = acados_in.x0[1] - acados_in.yref[0][1];
            error_pose.pose.pose.position.z = acados_in.x0[2] - acados_in.yref[0][2];
            error_pose.pose.pose.orientation.x = quat_error_msg.x;
            error_pose.pose.pose.orientation.y = quat_error_msg.y;
            error_pose.pose.pose.orientation.z = quat_error_msg.z;
            error_pose.pose.pose.orientation.w = quat_error_msg.w;
            error_pose.header.stamp = ros::Time::now();
            error_pose.header.frame_id = "odom_frame";
            error_pose.child_frame_id = "base_link";

            error_pose_pub.publish(error_pose);

            // publish conrtrol input
            control_input0.data = acados_out.u0[0];
            control_input1.data = acados_out.u0[1];
            control_input2.data = acados_out.u0[2];
            control_input3.data = acados_out.u0[3];
            control_input4.data = acados_out.u0[4];
            control_input5.data = acados_out.u0[5];

            control_input0_pub.publish(control_input0);
            control_input1_pub.publish(control_input1);
            control_input2_pub.publish(control_input2);
            control_input3_pub.publish(control_input3);
            control_input4_pub.publish(control_input4);
            control_input5_pub.publish(control_input5);
            /*Mission information cout**********************************************/        
            
            if(cout_counter > 2){ //reduce cout rate
                
                std::cout << "gazebo_sim loop solver information" << std::endl;
                std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << "x_ref:    " << acados_in.yref[0][0] << "\ty_ref:   " << acados_in.yref[0][1] << "\tz_ref:    " << acados_in.yref[0][2] << "\tyaw_ref:    " << yaw_ref << std::endl;
                std::cout << "roll_ref:    "<<acados_in.yref[0][3]<< std::endl;
                std::cout << "x_gt:     " << acados_in.x0[0] << "\ty_gt:     " << acados_in.x0[1] << "\tz_gt:     " << acados_in.x0[2] << "\tyaw_gt:     " << local_euler.psi << std::endl;
                std::cout << "roll_gt:        " << acados_in.x0[3] << "\t\tpitch_gt:        " << acados_in.x0[4] << std::endl;
                std::cout << "u1    : " << acados_out.u0[0] << "\tu2:    " << acados_out.u0[1] << "\tu3:    " << acados_out.u0[2] << "\tu4:    " << acados_out.u0[3]<< "\tu5:    " << acados_out.u0[4]<< "\tu6:    " << acados_out.u0[5] << std::endl;
                std::cout << "t0:  " << thrust0.data << "\tt1:  " << thrust1.data << "\tt2:  " << thrust2.data << "\tt3:  " << thrust3.data << "\tt4:  " << thrust4.data << "\tt5:  " << thrust5.data << "\tt6:  " << thrust6.data << "\tt7:  " << thrust7.data << std::endl;
                std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
                std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
                std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
                
                cout_counter = 0;
            }
            else{
                cout_counter++;
            }
        }        
    };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2heavy_mpc");
    ros::NodeHandle nh;
    std::string ref_traj;
    std::string ref_mode;
    
    // Get parameters
    nh.getParam("/bluerov2heavy_mpc_node/ref_traj", ref_traj);
    nh.param<std::string>("/bluerov2heavy_mpc_node/ref_mode", ref_mode, "readtxt");  // default: readtxt
    
    // Initialize NMPC with selected mode
    NMPC nmpc(nh, ref_traj, ref_mode);
    ros::Rate loop_rate(20);
    start_sub = false;
    while(ros::ok()){
        if(start_sub == true){
            nmpc.run();
        }
        //nmpc.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}