cmake_minimum_required(VERSION 2.8.3)
project(atlas_object_sorting)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tough_controller_interface
  tough_perception_common
  tough_footstep
)

find_package(PCL)

catkin_package(CATKIN_DEPENDS roscpp rospy tough_controller_interface tough_perception_common)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_talker src/talker.cpp)
add_executable(${PROJECT_NAME}_perception src/atlas_object_sorting_perception.cpp)
add_executable(${PROJECT_NAME}_navigation src/atlas_object_sorting_navigation.cpp)

set_target_properties(${PROJECT_NAME}_perception PROPERTIES OUTPUT_NAME perception PREFIX "")
set_target_properties(${PROJECT_NAME}_navigation PROPERTIES OUTPUT_NAME navigation PREFIX "")

target_link_libraries(${PROJECT_NAME}_talker ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_perception ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_navigation ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_talker ${PROJECT_NAME}_perception ${PROJECT_NAME}_navigation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
