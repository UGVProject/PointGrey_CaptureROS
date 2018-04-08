cmake_minimum_required(VERSION 2.8.3)
project(flea3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SRC_LIST src/main.cpp src/flea3Driver.cpp src/stereoCam.cpp)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        std_msgs
        )

set(OpenCV_DIR $ENV{HOME}/softwares/opencv_3.4/opencv/release)
FIND_PACKAGE( OpenCV REQUIRED )

catkin_package()

include_directories(
        include
        /usr/include/flycapture
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES( include ${catkin_INCLUDE_DIRS})

add_executable( stereo_record_node ${SRC_LIST})

add_library(flea3 ${SRC_LIST} )

TARGET_LINK_LIBRARIES( stereo_record_node ${OpenCV_LIBS} ${catkin_LIBRARIES} flycapture )
