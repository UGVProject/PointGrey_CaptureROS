// stereoCam.h
// Created by Zhang Handuo n 08/04/16.
// Modified by Zhang Handuo on 22/04/16.
#pragma once

#ifndef FLEA3_STEREOCAM_H
#define FLEA3_STEREOCAM_H

#include "../include/flea3Driver.h"
//ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <fstream>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <string>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

namespace flea3 {

class stereoCam {
public:
  stereoCam();

  virtual ~stereoCam();

  unsigned int scaleFactor;
  double scale;

  void loadParam(ros::NodeHandle &nh);

//  void setGuid(const uint64_t guid_left, const uint64_t guid_right);

//  void setPublishFrequency(const double frequency);

//  void setTopicName(const std::string topic_name);

//  void setFrameId(const std::string frame_id);

//  void setShuttle(const double Shuttlespeed);

  void setTXTName(char txtfile[]);

  void set_TriggerMode_value(unsigned int triggerMode);

  void run();

  void stop();

private:
  flea3Driver m_cam_left;
  flea3Driver m_cam_right;
  // publishers
  image_transport::Publisher pub_2,pub_left,pub_right;
  ros::Publisher pub_left_cam_info, pub_right_cam_info;
  sensor_msgs::CameraInfoPtr left_cam_info_msg, right_cam_info_msg;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> left_info_mgr, right_info_mgr;
  cv::Mat frame_left;
  cv::Mat frame_right;
  cv::Mat frame_2;
//  cv::Mat frame_cut;
  bool visualization;
  bool flag_pause;
  cv::Mat res;
  unsigned int frameTimeStamp_left;
  unsigned int frameTimeStamp_right;
  double frameSecond_left;
  double frameSecond_right;
  int rzWidth;
  int rzHeight;
  int frame_show_ratio;
  unsigned int num_camera;
  char timestampfile1[100];
  double frameTimeStamp_left_old;
  double frameTimeStamp_right_old;
  double left_show_time;
  double right_show_time;
  double showtime;
  double difference;
  unsigned int count_outof_sync;
  // parameters
  std::string uid_left;
  std::string uid_right;
  int imgWidth_;
  int imgHeight_;
  int imgHeight_cut;
  unsigned int imgSize_;
  int upbound;
  int downbound;
  double shuttle_speed;
  unsigned int trigger_mode_value;
  bool cameraFault_;
  int64_t framecount;
  std::string topic_name_;
  std::string frame_id_;
  std::string left_topic_name_;
  std::string left_frame_id_;
  std::string right_topic_name_;
  std::string right_frame_id_;
  std::string left_cam_info_topic_;
  std::string right_cam_info_topic_;
  std::string left_calib_url;
  std::string right_calib_url;
  std::string left_cam_name;
  std::string right_cam_name;
  int loopFrequency_;
  bool quitSignal_;

  inline double imageTimeStampToSeconds(unsigned int uiRawTimestamp);

  sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t);

  void publishImage(cv::Mat img, image_transport::Publisher &pub_img, std::string img_frame_id, ros::Time t);

  void fillCamInfo(sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg,
        std::string left_frame_id, std::string right_frame_id);

  void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t) ;
  // void GetImageData(flea3Driver &camera, unsigned int &timestamp);
};
}

#endif // FLEA3_STEREOCAM_H
