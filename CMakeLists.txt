cmake_minimum_required(VERSION 3.0.2)
project(cloud_match_ros)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs std_msgs pcl_ros pcl_conversions eigen_conversions tf2)

find_package(PCL REQUIRED )
find_package(Eigen3 REQUIRED )

catkin_package( 
	INCLUDE_DIRS include
	LIBRARIES omco_localization
	DEPENDS
	PCL
	)


include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIR} ${Eigen3_INCLUDE_DIR})

add_executable(pc_localizer_node src/pc_localizer_node.cpp src/pc_localizer.cpp)

target_link_libraries(pc_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})
