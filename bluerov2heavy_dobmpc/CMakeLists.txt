cmake_minimum_required(VERSION 3.0.2)
project(bluerov2heavy_dobmpc)

# ── C++ standard ──────────────────────────────────────────────────────────────
if(${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "7.6")
    add_compile_options(-std=c++11)
else()
    add_compile_options(-std=c++14)
endif()

# ── Catkin dependencies ───────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  roslaunch
  uuv_gazebo_ros_plugins_msgs
  nav_msgs
  geometry_msgs
  message_generation
  tf
  std_msgs
  gazebo_msgs
  genmsg
  sensor_msgs
  pcl_ros
)

find_package(Eigen3    REQUIRED)
find_package(yaml-cpp  REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(PCL 1.8.1 REQUIRED)
find_package(OpenCV    REQUIRED)

# ── ACADOS paths ──────────────────────────────────────────────────────────────
# Edit these two lines if your acados installation is in a different directory.
set(acados_include "/home/dorsa/acados/include")
set(acados_lib     "/home/dorsa/acados/lib")

# Generated C code produced by generate_c_code_dob.py / generate_c_code_ampc.py
# Both scripts write into the same c_generated_code folder.
set(bluerov2heavy_model ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

# ── Custom messages (reused from standard package — same .msg files) ──────────
add_message_files(
  FILES
  Reference.msg
  Pose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

# ── Catkin package declaration ────────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bluerov2heavy_dobmpc
  LIBRARIES bluerov2heavy_ampc
  CATKIN_DEPENDS gazebo_msgs message_generation message_runtime
)

# ── Include directories ───────────────────────────────────────────────────────
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${bluerov2heavy_model}
  ${acados_include}
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/acados/
  ${acados_include}/qpOASES_e/
  ${PCL_CONVERSIONS_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${bluerov2heavy_model})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# =============================================================================
# EXECUTABLES
# =============================================================================

# ── bluerov2heavy_dob_node ────────────────────────────────────────────────────
# DOBMPC: EAOB (EKF disturbance observer) + MPC, NP=6, NU=8
add_executable(bluerov2heavy_dob_node
  src/bluerov2heavy_dob_node.cpp
  src/bluerov2heavy_dob.cpp
)
target_link_libraries(bluerov2heavy_dob_node
  ${bluerov2heavy_model}/libacados_ocp_solver_bluerov2heavy.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)
add_dependencies(bluerov2heavy_dob_node
  uuv_gazebo_ros_plugins_msgs_generate_messages_cpp
  bluerov2heavy_dobmpc_generate_messages_cpp
)

# ── bluerov2heavy_ampc_node ───────────────────────────────────────────────────
# Adaptive MPC: EAOB + RLS-VFF online system identification, NP=24, NU=8
add_executable(bluerov2heavy_ampc_node
  src/bluerov2heavy_ampc_node.cpp
  src/bluerov2heavy_ampc.cpp
)
target_link_libraries(bluerov2heavy_ampc_node
  ${bluerov2heavy_model}/libacados_ocp_solver_bluerov2heavy.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)
add_dependencies(bluerov2heavy_ampc_node
  uuv_gazebo_ros_plugins_msgs_generate_messages_cpp
  bluerov2heavy_dobmpc_generate_messages_cpp
)

# ── bluerov2heavy_random_node ─────────────────────────────────────────────────
# Random excitation for RLS-VFF system identification data collection
add_executable(bluerov2heavy_random_node
  src/bluerov2heavy_random_node.cpp
)
target_link_libraries(bluerov2heavy_random_node
  ${catkin_LIBRARIES}
)
add_dependencies(bluerov2heavy_random_node
  uuv_gazebo_ros_plugins_msgs_generate_messages_cpp
)

# ── bluerov2heavy_inspection_node ────────────────────────────────────────────
# Visual/PID bridge inspection controller with 8-thruster allocation
add_executable(bluerov2heavy_inspection_node
  src/bluerov2heavy_inspection_node.cpp
)
target_link_libraries(bluerov2heavy_inspection_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(bluerov2heavy_inspection_node
  uuv_gazebo_ros_plugins_msgs_generate_messages_cpp
)