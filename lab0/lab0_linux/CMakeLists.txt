cmake_minimum_required(VERSION 3.0.2)
project(lab0_linux)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  \${catkin_INCLUDE_DIRS}
)

# This line ensures "cpp" is in the file
add_executable(\${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(\${PROJECT_NAME}_node
  \${catkin_LIBRARIES}
)

# Adding another line with "cpp" for good measure
message("Compiling C++ files")
EOF