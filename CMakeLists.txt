cmake_minimum_required(VERSION 2.8.3)
project(we_r2_moveit_config)

find_package(Boost REQUIRED COMPONENTS system date_time thread)
find_package(catkin REQUIRED COMPONENTS moveit_core)

catkin_package()

include_directories(hardware/inc/;${catkin_INCLUDE_DIRS};)
link_directories(${catkin_LIBRARY_DIRS})

# RRBot Hardware Interface
add_library(we_r2_hw_interface
  hardware/inc/we_r2_hw_interface.cpp
)
target_link_libraries(we_r2_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(we_r2_hw_controller hardware/src/we_r2_hw_main.cpp)
target_link_libraries(we_r2_hw_controller we_r2_hw_interface ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)

# Install executables
install(TARGETS
    we_r2_hw_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY hardware/inc/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
