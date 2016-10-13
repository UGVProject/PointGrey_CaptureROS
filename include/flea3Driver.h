// flea3Driver.h
// Created by Zhang Handuo on 08/04/16.
//

#ifndef FLEA3_FLEA3DRIVER_H
#define FLEA3_FLEA3DRIVER_H

#include "ros/ros.h"
//camera driver
#include "FlyCapture2.h"
//system headers
#include <iostream>
#include <vector>
#include <pthread.h>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#ifdef LINUX
#include <unistd.h>
#endif

namespace flea3 {

class flea3Driver {
public:
    flea3Driver();
    virtual ~flea3Driver();
    bool init(const uint64_t cameraUid );

//    bool start();

//    void stop();

    void setShuttleSpeed(double shuttleSpeed);

    void set_ShutterMode(bool bAuto);

    bool changeShuttleSpeed(double shuttleSpeed);

    bool shutdown();

    void set_TriggerMode(unsigned int triggerMode);

    void capture( unsigned int &captureTimestamp);
    void getFrame(unsigned char *imageData, unsigned int imageSize);
    // bool getFrame(unsigned char *imageData,
    //               unsigned int &captureTimestamp,
    //               unsigned int imageSize);
    //const uint64_t getImageSize();
    //bool getImageSize();

private:
    FlyCapture2::BusManager busMgr;
    //FlyCapture2::Error error;
    FlyCapture2::PGRGuid guid;
    FlyCapture2::Camera cam;
    FlyCapture2::CameraInfo camInfo;
    FlyCapture2::Property camProperty;
    FlyCapture2::Image rawImage;
    // FlyCapture2::Image monoImage;
    FlyCapture2::TriggerMode triggerMode;
    FlyCapture2::ImageMetadata metadata;
    unsigned int trigger_mode;
    const unsigned int k_frameInfoReg = 0x12F8;
    unsigned int frameInfoRegVal;
    const unsigned int k_ShutterReg = 0x1098;
    unsigned int ShutterRegVal;
    double shutter_speed;
    bool isCameraReady;
    bool isAuto;
    // bool isFrameAvailable;

    unsigned int width;
    unsigned int height;

    unsigned int _imageTimestamp;
    unsigned int _imageDataSize;

//    //threading
//    pthread_t cameraThread;
//    pthread_mutex_t cameraMutex;
//    pthread_mutex_t quitMutex;

    //bool quitSignal;

    //static void* pcameraThreadFunc(void *arg);
    //void cameraThreadFunc();
    bool startCameraThread();

    bool init_camera(const uint64_t cameraUid );
    bool shutdown_camera();
    bool setup_camera();
    //void cleanup_and_exit(dc1394camera_t *camera);



};

}   //namespace flea3


#endif //FLEA3_FLEA3DRIVER_H
