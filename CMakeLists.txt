cmake_minimum_required(VERSION 2.8.3)
project(we_r2_moveit_config)

find_package(Boost REQUIRED COMPONENTS system date_time thread)
find_package(catkin REQUIRED COMPONENTS moveit_core actionlib)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS};)
link_directories(${catkin_LIBRARY_DIRS})

# Main control executable
add_executable(joint_trajectory_action_node src/joint_trajectory_action.cpp)
target_link_libraries(joint_trajectory_action_node ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)

# Install executables
install(TARGETS
    joint_trajectory_action_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

