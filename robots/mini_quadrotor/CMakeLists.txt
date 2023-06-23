cmake_minimum_required(VERSION 3.0.2)
project(mini_quadrotor)

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros_control
  roscpp
  sensor_msgs
  )

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(process_pointcloud src/process_pointcloud.cpp)
add_dependencies(process_pointcloud ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(process_pointcloud
  ${catkin_LIBRARIES}
  )




install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
