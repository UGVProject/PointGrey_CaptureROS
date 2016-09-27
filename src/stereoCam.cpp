// stereoCam.cpp
// Created by Zhang Handuo on 08/04/16.
// Modified by Zhang Handuo on 22/04/16.
#include "stereoCam.h"

namespace flea3 {

stereoCam::stereoCam()
    : loopFrequency_(50.0), shuttle_speed(0.3), uid_left(0), // 15231263
      uid_right(0),                                          // 15231302
      cameraFault_(false), imgWidth_(1280), imgHeight_(1024),
      imgSize_(1280 * 1024), frameTimeStamp_left(0), frameTimeStamp_right(0),
      trigger_mode_value(0) {count_outof_sync = 0;}

stereoCam::~stereoCam() {
  //
}

void stereoCam::loadParam(ros::NodeHandle &nh) {
  // init publisher
  image_transport::ImageTransport it(nh);
  pub_2 = it.advertise(topic_name_, 30);
  quitSignal_ = false;
  framecount = 0;
  // const int rzWidth = static_cast<int>(imgWidth_/2);
  // unsigned int rzHeight = static_cast<int>(imgHeight_/2);
  rzHeight = imgHeight_ - upbound - downbound/ 2;
  imgSize_ = (unsigned int) (imgHeight_ * imgWidth_);
}

inline double stereoCam::imageTimeStampToSeconds(unsigned int uiRawTimestamp) {

        int nSecond =
                (uiRawTimestamp >> 25) & 0x7F; // get rid of cycle_* - keep 7 bits
        int nCycleCount = (uiRawTimestamp >> 12) & 0x1FFF; // get rid of offset
        int nCycleOffset = (uiRawTimestamp >> 0) & 0xFFF; // get rid of *_count

        return (double)nSecond +
               (((double)nCycleCount + ((double)nCycleOffset / 3072.0)) / 8000.0);
}

//void stereoCam::setGuid(const uint64_t guid_left, const uint64_t guid_right) {
//  uid_left = guid_left;
//  uid_right = guid_right;
//}
//void stereoCam::setShuttle(const double Shuttlespeed) {
//  shuttle_speed = Shuttlespeed;
//}
//void stereoCam::setPublishFrequency(const double frequency) {
//  loopFrequency_ = frequency;
//}

//void stereoCam::setTopicName(const std::string topic_name) {
//  topic_name_ = topic_name;
//}

void stereoCam::setTXTName(char txtfile[]) {
  memcpy(timestampfile1, txtfile, sizeof(timestampfile1));
  //std::cout << "txtFilename is: " << timestampfile1 << std::endl;
}

void stereoCam::set_TriggerMode_value(unsigned int triggerMode) {
  trigger_mode_value = triggerMode;
}

//void stereoCam::setFrameId(const std::string frame_id) { frame_id_ = frame_id; }

void stereoCam::run() {
  ros::NodeHandle nh("~");

  if(nh.getParam("image/width", imgWidth_))
    ROS_INFO("image width: %d", imgWidth_);
  else
    ROS_WARN("Use default image width: %d", imgWidth_);
  if(nh.getParam("image/height", imgHeight_))
    ROS_INFO("image height: %d", imgHeight_);
  else
    ROS_WARN("Use default image height: %d", imgHeight_);
  if(nh.getParam("roi/upbound", upbound))
    ROS_INFO("image roi up boundary: %d", upbound);
  else
    ROS_WARN("Use default image roi upbound: %d", upbound);
  if(nh.getParam("roi/downbound", downbound))
    ROS_INFO("image roi down boundary: %d", downbound);
  else
    ROS_WARN("Use default image roi downbound: %d", downbound);
  if(nh.getParam("left/Guid", uid_left))
    ROS_INFO("left camera guid: %d", uid_left);
  else
    ROS_WARN("Use default left camera guid: %d", uid_left);
  if(nh.getParam("right/Guid", uid_right))
    ROS_INFO("right camera guid: %d", uid_right);
  else
    ROS_WARN("Use default right camera guid: %d", uid_right);
  if(nh.getParam("shutter_speed", shuttle_speed))
    ROS_INFO("shutter speed: %f", shuttle_speed);
  else
    ROS_WARN("Use default shutter speed: %f", shuttle_speed);
  if(nh.getParam("publish/frequency", loopFrequency_))
    ROS_INFO("publish loop frequency: %d", loopFrequency_);
  else
    ROS_WARN("Use default publish loop frequency: %d", loopFrequency_);
  if(nh.getParam("publish/frame_id", frame_id_ ))
    ROS_INFO("Get stereo frame ID: %s", frame_id_.c_str());
  else
    ROS_WARN("Use default stereo frame ID: %s", frame_id_.c_str());
  if(nh.getParam("publish/topic_name", topic_name_ ))
    ROS_INFO("Get stereo frame topic: %s", topic_name_.c_str());
  else
    ROS_WARN("Use default stereo frame topic: %s", topic_name_.c_str());

  // load parameters
  loadParam(nh);
  std::ofstream outputFile;
  outputFile.open(timestampfile1);
  // init camera driver
  imgHeight_cut = imgHeight_ - upbound - downbound;
  m_cam_left.setShuttleSpeed(shuttle_speed);
  m_cam_right.setShuttleSpeed(shuttle_speed);
  m_cam_left.set_TriggerMode(trigger_mode_value);
  m_cam_right.set_TriggerMode(trigger_mode_value);
  frame_2 = cv::Mat(imgHeight_cut, imgWidth_ * 2, CV_8UC1);
  cv::Size rzSize(imgWidth_, rzHeight);
  // cv::Mat frame_left = frame_2(cv::Range::all() , cv::Range(1 , imgWidth_));
  // cv::Mat frame_right = frame_2(cv::Range::all() , cv::Range(imgWidth_ +1 ,
  // imgWidth_ *2));

  if (m_cam_left.init((const uint64_t) uid_left) && m_cam_right.init((const uint64_t) uid_right)) {
    frame_left = cv::Mat(imgHeight_, imgWidth_, CV_8UC1);
    frame_right = cv::Mat(imgHeight_, imgWidth_, CV_8UC1);
  } else {
    std::cout << "Left or Right camera init failed!" << std::endl;
    cameraFault_ = true;
  }

  // wait
  sleep(1);
  // cv::namedWindow( topic_name_.c_str() );
  sensor_msgs::ImagePtr image_msg;
  // ros::Rate loop_rate(loopFrequency_);
  uint32_t counter = 0; // for display
  while (ros::ok && !cameraFault_ && !quitSignal_) {

      if (m_cam_left.capture(frameTimeStamp_left)) {
          if (m_cam_right.capture(frameTimeStamp_right)) {}
          else {
              std::cout << "image retrieve image data wrong!" << std::endl;
          }
      } else {
          std::cout << "image retrieve image data wrong!" << std::endl;
      }
    m_cam_right.getFrame(frame_right.data, imgSize_);
    m_cam_left.getFrame(frame_left.data, imgSize_);
      cv::Mat frame_left_cut, frame_right_cut;
    frame_left_cut = frame_left( cv::Range(upbound, imgHeight_ - downbound), cv::Range::all() );
    frame_right_cut = frame_left( cv::Range(upbound, imgHeight_ - downbound), cv::Range::all() );

    frame_left_cut.copyTo(frame_2(cv::Rect(0, 0, imgWidth_, imgHeight_cut)));
    frame_right_cut.copyTo(frame_2(cv::Rect(imgWidth_, 0, imgWidth_, imgHeight_cut)));
    // convert Matrix to image_msg

    image_msg =
        cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_2).toImageMsg();
    image_msg->header.stamp = ros::Time::now();
    image_msg->header.frame_id = frame_id_;
    pub_2.publish(image_msg);
    framecount++;
    if(framecount > 1)
    {
            frameSecond_left = imageTimeStampToSeconds(frameTimeStamp_left);
            frameSecond_right = imageTimeStampToSeconds(frameTimeStamp_right);

            left_show_time = frameSecond_left - frameTimeStamp_left_old;
            // left_show_time = frameTimeStamp_left;
            right_show_time = frameSecond_right - frameTimeStamp_right_old;
            // right_show_time = frameTimeStamp_right;
            difference = left_show_time - right_show_time;
            if((difference >= 0.01)||(difference <= -0.01))
            {
              count_outof_sync ++;
              std::cout << "Out of sync : " << count_outof_sync << " Difference is: " << difference << std::endl;
            }

            // difference = frameSecond_left - frameSecond_right;
            outputFile << framecount << "frame: Left image difference: " << left_show_time
                       << " | Right image difference: " << right_show_time << " |difference: " << difference << std::endl;
    }
    frameTimeStamp_left_old = frameSecond_left;
    frameTimeStamp_right_old = frameSecond_right;

    cv::resize(frame_2, res, rzSize);
    // display image once every 10 frames
    if (counter >= 10) {
      cv::imshow("press 's' to decrease shuttle, 'b' to increase shuttle, "
                 "'ESC' to quit",
                 res);
      int key = cv::waitKey(1);
      if (key == 27) {
        break;
      } else if (key == 115) // type "s"
      {
        std::cout << "You typed s " << std::endl;
        if (shuttle_speed > 0.2) {
          shuttle_speed = shuttle_speed - 0.3;
          if ((m_cam_left.changeShuttleSpeed(shuttle_speed)) &&
              (m_cam_right.changeShuttleSpeed(shuttle_speed))) {
            std::cout << "shuttle speed deceased " << std::endl;
          } else {
            std::cout << "shuttle speed change failed! " << std::endl;
          }
        } else {
          std::cout << "shuttle speed is already very low! " << std::endl;
        }
      } else if (key == 98) // type "b"
      {
        std::cout << "You typed b " << std::endl;
        if (shuttle_speed < 22) {
          shuttle_speed = shuttle_speed + 0.3;
          if ((m_cam_left.changeShuttleSpeed(shuttle_speed)) &&
              (m_cam_right.changeShuttleSpeed(shuttle_speed))) {
            std::cout << "shuttle speed increased " << std::endl;
          } else {
            std::cout << "shuttle speed change failed! " << std::endl;
          }
        } else {
          std::cout << "shuttle speed is already very high! " << std::endl;
        }
      }
      counter = 0;
    } else {
      counter++;
    }

    ros::spinOnce();
//    loop_rate.sleep();
  }
  std::cout << "Altogether:  " << framecount << " frames" << std::endl;
  std::cout << "Stopping camera! Out of sync number is : " << count_outof_sync << std::endl;
  m_cam_left.shutdown();
  m_cam_right.shutdown();
  outputFile.close();
  stop();
}

void stereoCam::stop() {
  quitSignal_ = true;
  ros::shutdown();
}
}
