cmake_minimum_required(VERSION 3.10.0)
project(extrinsic_calibrator)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O3")


find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package( Boost )

find_package(catkin REQUIRED COMPONENTS
      cv_bridge
      image_transport
      roscpp
      sensor_msgs
      std_msgs
      pcl_conversions
      pcl_ros
      tf
      tf_conversions
      tf2_eigen
      tf2
      rospy
      message_generation
      geometry_msgs
      rosbag
      nav_msgs
      dynamic_reconfigure
      )
      




catkin_package() # dont forget this for launch file or rosrun

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

add_executable(extrinsic_calibrator_exe
        src/main.cpp
        src/extrinsic_calibrator.cpp
        src/point_cloud_handler.cpp
        src/data_write_read.cpp
        src/ros_handler.cpp
        src/feature_extractor.cpp
        src/rotation_solver.cpp
        src/test.cpp
        src/translation_solver.cpp
)

add_executable(keyboard_publisher_exe
  src/keyboard_publisher.cpp)


target_link_libraries(extrinsic_calibrator_exe
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  Ceres::ceres
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(keyboard_publisher_exe
  ${catkin_LIBRARIES}
  )

add_dependencies(extrinsic_calibrator_exe
        ${catkin_EXPORTED_TARGETS}
    
        )

add_dependencies(keyboard_publisher_exe
  ${catkin_EXPORTED_TARGETS}
  )


add_definitions(${PCL_DEFINITIONS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )

catkin_package()



