cmake_minimum_required(VERSION 2.8.3)
project(color_3d_tracker)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	sensor_msgs
	image_transport
	image_geometry
	cv_bridge
	tf
	std_srvs)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals)

catkin_package()

include_directories(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

add_executable(color_3d_tracker_node src/node_tracker.cpp src/tracker.cpp)
target_link_libraries(color_3d_tracker_node ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(color_3d_trainer_node src/node_trainer.cpp src/trainer.cpp)
target_link_libraries(color_3d_trainer_node ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})