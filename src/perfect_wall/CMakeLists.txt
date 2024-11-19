cmake_minimum_required(VERSION 3.0.2)
project(perfect_wall)

find_package(catkin REQUIRED COMPONENTS  
  std_msgs  
  rospy  
  roscpp  
  sensor_msgs  
  pcl_conversions  
)  
catkin_package()  

include_directories(  
  ${catkin_INCLUDE_DIRS}  
)  

add_executable(perfect_wall src/main.cpp)  
target_link_libraries(perfect_wall ${catkin_LIBRARIES} ${PCL_LIBRARIES})