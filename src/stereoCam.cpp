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
      trigger_mode_value(0),scale(0.5) {count_outof_sync = 0; visualization = false;}

stereoCam::~stereoCam() {
  //
}

sensor_msgs::ImagePtr stereoCam::imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t){
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *)&num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*)(&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*)(&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}

void stereoCam::publishImage(cv::Mat img, image_transport::Publisher &pub_img, std::string img_frame_id, ros::Time t) {
    pub_img.publish(imageToROSmsg(img
                                , sensor_msgs::image_encodings::MONO8
                                , img_frame_id
                                , t ));
}

void stereoCam::loadParam(ros::NodeHandle &nh) {
  // init publisher
  image_transport::ImageTransport it(nh);
  pub_2 = it.advertise(topic_name_, 30);
  ROS_INFO_STREAM("Advertized on topic " << topic_name_);
  pub_left = it.advertise(left_topic_name_, 1); //left
  ROS_INFO_STREAM("Advertized on topic " << left_topic_name_);
  pub_right = it.advertise(right_topic_name_, 1); //right
  ROS_INFO_STREAM("Advertized on topic " << right_topic_name_);

  quitSignal_ = false;
  framecount = 0;
  // const int rzWidth = static_cast<int>(imgWidth_/2);
  // unsigned int rzHeight = static_cast<int>(imgHeight_/2);
  rzHeight = (imgHeight_ - upbound - downbound)/ 2;
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

void stereoCam::setTXTName(char txtfile[]) {
  memcpy(timestampfile1, txtfile, sizeof(timestampfile1));
  //std::cout << "txtFilename is: " << timestampfile1 << std::endl;
}

void stereoCam::set_TriggerMode_value(unsigned int triggerMode) {
  trigger_mode_value = triggerMode;
}

//void stereoCam::setFrameId(const std::string frame_id) { frame_id_ = frame_id; }

void stereoCam::run() {
  ros::NodeHandle nh;
  ros::NodeHandle nh_s("~");

  if(nh_s.getParam("image/width", imgWidth_))
    ROS_INFO("image width: %d", imgWidth_);
  else
    ROS_WARN("Use default image width: %d", imgWidth_);
  if(nh_s.getParam("showfrequency_ratio", frame_show_ratio))
    ROS_INFO("show every %d frames", frame_show_ratio);
  else
    ROS_WARN("Use default show ratio: %d", frame_show_ratio);
  if(nh_s.getParam("image/height", imgHeight_))
    ROS_INFO("image height: %d", imgHeight_);
  else
    ROS_WARN("Use default image height: %d", imgHeight_);
  if(nh_s.getParam("roi/upbound", upbound))
    ROS_INFO("image roi up boundary: %d", upbound);
  else
    ROS_WARN("Use default image roi upbound: %d", upbound);
  if(nh_s.getParam("roi/downbound", downbound))
    ROS_INFO("image roi down boundary: %d", downbound);
  else
    ROS_WARN("Use default image roi downbound: %d", downbound);
  if(nh_s.getParam("left/Guid", uid_left))
    ROS_INFO("left camera guid: %d", uid_left);
  else
    ROS_WARN("Use default left camera guid: %d", uid_left);
  if(nh_s.getParam("right/Guid", uid_right))
    ROS_INFO("right camera guid: %d", uid_right);
  else
    ROS_WARN("Use default right camera guid: %d", uid_right);
  if(nh_s.getParam("shutter_speed", shuttle_speed))
    ROS_INFO("shutter speed: %f", shuttle_speed);
  else
    ROS_WARN("Use default shutter speed: %f", shuttle_speed);
  if(nh_s.getParam("publish/frequency", loopFrequency_))
    ROS_INFO("publish loop frequency: %d", loopFrequency_);
  else
    ROS_WARN("Use default publish loop frequency: %d", loopFrequency_);
  if(nh_s.getParam("publish/frame_id", frame_id_ ))
    ROS_INFO("Get stereo frame ID: %s", frame_id_.c_str());
  else
    ROS_WARN("Use default stereo frame ID: %s", frame_id_.c_str());
  if(nh_s.getParam("publish/topic_name", topic_name_ ))
    ROS_INFO("Get stereo frame topic: %s", topic_name_.c_str());
  else
    ROS_WARN("Use default stereo frame topic: %s", topic_name_.c_str());

  if(nh_s.getParam("publish/left_frame_id", left_frame_id_ ))
    ROS_INFO("Get left stereo frame ID: %s", left_frame_id_.c_str());
  else
    ROS_WARN("Use default left stereo frame ID: %s", left_frame_id_.c_str());
  if(nh_s.getParam("publish/left_topic", left_topic_name_ ))
    ROS_INFO("Get left stereo frame topic: %s", left_topic_name_.c_str());
  else
    ROS_WARN("Use default left stereo frame topic: %s", left_topic_name_.c_str());

  if(nh_s.getParam("publish/right_frame_id", right_frame_id_ ))
    ROS_INFO("Get right stereo frame ID: %s", right_frame_id_.c_str());
  else
    ROS_WARN("Use default right stereo frame ID: %s", right_frame_id_.c_str());
  if(nh_s.getParam("publish/right_topic", right_topic_name_ ))
    ROS_INFO("Get right stereo frame topic: %s", right_topic_name_.c_str());
  else
    ROS_WARN("Use default right stereo frame topic: %s", right_topic_name_.c_str());

  if(nh_s.getParam("cvmat_show", visualization))
    ROS_INFO("Get visualization flag: %s",visualization? "true":"false");
  else
    ROS_WARN("Using default visualization flag: false!");
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
  // sensor_msgs::ImagePtr image_msg,image_msg_l,image_msg_r;
  // ros::Rate loop_rate(loopFrequency_);
  uint32_t counter = 0; // for display
  while (ros::ok && !cameraFault_ && !quitSignal_) {

    boost::thread th1 (boost::bind(&flea3Driver::capture, &m_cam_left, frameTimeStamp_left));
    boost::thread th2 (boost::bind(&flea3Driver::capture, &m_cam_right, frameTimeStamp_right));
    th1.join();
    th2.join();

    ros::Time t = ros::Time::now();   // Get current time
    int combine_SubNumber = pub_2.getNumSubscribers();
    int left_SubNumber = pub_left.getNumSubscribers();
    int right_SubNumber = pub_right.getNumSubscribers();


    m_cam_right.getFrame(frame_right.data, imgSize_);
    m_cam_left.getFrame(frame_left.data, imgSize_);
      cv::Mat frame_left_cut, frame_right_cut,frame_left_save,frame_right_save;
    frame_left_cut = frame_left( cv::Range(upbound, imgHeight_ - downbound), cv::Range::all() );
    frame_right_cut = frame_right( cv::Range(upbound, imgHeight_ - downbound), cv::Range::all() );

    frame_left_cut.copyTo(frame_2(cv::Rect(0, 0, imgWidth_, imgHeight_cut)));
    frame_right_cut.copyTo(frame_2(cv::Rect(imgWidth_, 0, imgWidth_, imgHeight_cut)));

    cv::resize(frame_left_cut, frame_left_save, cv::Size(),scale,scale);
    cv::resize(frame_right_cut, frame_right_save, cv::Size(),scale,scale);

    if(combine_SubNumber > 0) {
      publishImage(frame_2, pub_2, frame_id_, t);
    }
    if(left_SubNumber > 0){
      publishImage(frame_left_save, pub_left, left_frame_id_, t);
    }
    if(right_SubNumber > 0){
      publishImage(frame_right_save, pub_right, right_frame_id_, t);
    }

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

    if(visualization){
      cv::resize(frame_2, res, rzSize);
      // display image once every 10 frames
      if (counter >= frame_show_ratio) {
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

// void stereoCam::GetImageData(flea3Driver &camera, unsigned int &timestamp)
// {
//   if (!camera.capture(timestamp))
//     std::cout << "image retrieve image data wrong!" << std::endl;
// }
