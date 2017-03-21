cmake_minimum_required(VERSION 2.8.3)
project(flea3)

#unset(OpenCV_CONFIG_PATH CACHE)
#unset(OpenCV_DIR CACHE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SRC_LIST src/main.cpp src/flea3Driver.cpp src/stereoCam.cpp)

find_package(catkin REQUIRED COMPONENTS
        camera_info_manager
        cv_bridge
        image_transport
        roscpp
        rospy
        std_msgs
        )

FIND_PACKAGE( OpenCV REQUIRED )
#find_package(OpenCV REQUIRED HINTS /home/zh/softwares/opencv-3.1.0/install)

catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES ${PROJECT_NAME} ${catkin_INCLUDE_DIRS}
#        CATKIN_DEPENDS roscpp
#        DEPENDS opencv
)

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

#unset(OpenCV_CONFIG_PATH CACHE)
#unset(OpenCV_DIR CACHE)
