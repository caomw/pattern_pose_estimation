cmake_minimum_required(VERSION 2.8.3)
project(pattern_pose_estimation)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  image_transport
  sensor_msgs
  std_srvs
  tf
  cv_bridge
  camera_calibration_parsers
  libartoolkit
  message_generation
  nodelet)

find_package(OpenCV REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES DetectMarker.srv)

generate_messages(
  DEPENDENCIES sensor_msgs ar_pose std_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${libartoolkit_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(marker_detection src/marker_detector.cpp 
                             src/marker_detector_node.cpp
                             src/marker_detector_server.cpp
                             src/marker_detector_nodelet.cpp)

add_executable(checkerboard_detector_node src/checkerboard_detector_node.cpp)
add_executable(marker_detector_node src/marker_detector_node_main.cpp)
add_executable(marker_detector_server src/marker_detector_server_main.cpp)
add_executable(marker_filter_node src/marker_filter_node.cpp)

add_dependencies(marker_detection ar_pose_gencpp)
add_dependencies(marker_filter_node ar_pose_gencpp)
add_dependencies(marker_detector_server ar_pose_gencpp)

target_link_libraries(marker_detection ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(checkerboard_detector_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(marker_filter_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(marker_detector_node marker_detection AR ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(marker_detector_server marker_detection AR ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
