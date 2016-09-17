// flea3Driver.cpp
// Created by Zhang Handuo on 08/04/16.
// Modified by Zhang Handuo on 22/04/16.

#include "flea3Driver.h"

namespace flea3 {

    flea3Driver::flea3Driver():
            shutter_speed(3.3),
            trigger_mode(0),
            isCameraReady(false),
//            isFrameAvailable(false),
            width(1280),
            height(1024)
    {
        isCameraReady = 0;
//        std::cout<<"init mutexes\n";
//        pthread_mutex_init(&cameraMutex, NULL);
//        pthread_mutex_init(&quitMutex, NULL);
    }

    flea3Driver::~flea3Driver() {
//        std::cout<<"destroy mutexes\n";
//        pthread_mutex_destroy(&cameraMutex);
//        pthread_mutex_destroy(&quitMutex);
    }

    bool flea3Driver::init(const uint64_t cameraUid)
    {
        bool res = 0;
        isCameraReady=0;
        if(init_camera(cameraUid)){
            if(setup_camera()){
                //pthread_mutex_lock(&cameraMutex);
                isCameraReady=true;
                //pthread_mutex_unlock(&cameraMutex);
                res = 1;
            }else{
                res = 0;
            }
        }else{
            res = 0;
        }
        return res;
    }

    void flea3Driver::set_TriggerMode(unsigned int triggerMode)
    {
      trigger_mode = triggerMode;
    }

    void flea3Driver::setShuttleSpeed(double shuttleSpeed)
    {
        shutter_speed = shuttleSpeed;
    }

    bool flea3Driver::changeShuttleSpeed(double shuttleSpeed)
    {
        bool res=0;
        FlyCapture2::Error error;
        shutter_speed = shuttleSpeed;
        // camProperty.type = FlyCapture2::SHUTTER;
        camProperty.type = FlyCapture2::SHUTTER;
        // camProperty.absControl = true;
        // camProperty.onePush = false;
        camProperty.onOff = true;
        camProperty.autoManualMode = false;
        camProperty.absValue = shutter_speed;
        error = cam.SetProperty(&camProperty);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            std::cout << "setProperty Wrong" << std::endl;
            res = 0;
        }else{
        error = cam.GetProperty(&camProperty);
        std::cout << "shuttle speed set to" << camProperty.absValue << std::endl;
        res = 1;
      }
      return res;
    }

    bool flea3Driver::shutdown()
    {
        bool res = shutdown_camera();

        return (res);
    }

    bool flea3Driver::capture( unsigned int &captureTimestamp)
    {
      FlyCapture2::Error error;
      metadata = rawImage.GetMetadata();
      captureTimestamp = metadata.embeddedTimeStamp;
      error = cam.RetrieveBuffer( &rawImage );
      if (error != FlyCapture2::PGRERROR_OK) {
          std::cout << "image retrieve wrong\n" << std::endl;
          return false;
      }
      else{
        return true;
      }
    }

    void flea3Driver::getFrame(unsigned char *imageData, unsigned int imageSize)
    {
        memcpy(imageData, rawImage.GetData(),	imageSize);
    }

    bool flea3Driver::init_camera(const uint64_t cameraUid)
    {
        bool res = 0;
        uint32_t errCount = 0;
        FlyCapture2::Error error;
        // Get the camera information
        error = busMgr.GetCameraFromSerialNumber(cameraUid, &guid);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            std::cout << "getSerialNumberWrong" << std::endl;
            errCount ++;
        }
        // Connect to a camera
        error = cam.Connect(&guid);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            std::cout << "connect camera wrong!" << std::endl;
            errCount ++;
        }
        frameInfoRegVal = 0;
        error = cam.ReadRegister(k_frameInfoReg, &frameInfoRegVal);
        if ( error != FlyCapture2::PGRERROR_OK )
        {
          std::cout << " Wrong reading register!" << std::endl;
        }

        frameInfoRegVal = frameInfoRegVal | 0x41;
        error = cam.WriteRegister( k_frameInfoReg, frameInfoRegVal );
        if ( error != FlyCapture2::PGRERROR_OK )
        {
          std::cout << " Wrong writing frame_info register!" << std::endl;
        }

        ShutterRegVal = 0x80001200;
        error = cam.WriteRegister( k_ShutterReg, ShutterRegVal );
        if ( error != FlyCapture2::PGRERROR_OK )
        {
          std::cout << " Wrong writing auto_shutter register!" << std::endl;
        }

        error = cam.GetCameraInfo(&camInfo);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            std::cout << "GetCameraInfo Wrong!" << std::endl;
            errCount ++;
        }
        camProperty.type = FlyCapture2::AUTO_EXPOSURE;
        // camProperty.absControl = true;
        camProperty.onePush = false;
        camProperty.onOff = true;
        camProperty.autoManualMode = true;
        // camProperty.absValue = 5.3;
        error = cam.SetProperty(&camProperty);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            //PrintError( error );
            std::cout << "setProperty Wrong" << std::endl;
            errCount ++;
        }

        // camProperty.type = FlyCapture2::SHUTTER;
        // camProperty.absControl = true;
        // camProperty.onePush = false;
        // camProperty.onOff = true;
        // camProperty.autoManualMode = false;
        // camProperty.absValue = shutter_speed;
        // error = cam.SetProperty(&camProperty);
        // if (error != FlyCapture2::PGRERROR_OK)
        // {
        //     std::cout << "setProperty Wrong" << std::endl;
        //     errCount ++;
        // }

        if(errCount == 0){ return 1; }
        else{ return 0;}
    }

    bool flea3Driver::shutdown_camera()
    {
        bool res = 1;
        FlyCapture2::Error error;
        error = cam.StopCapture();
        triggerMode.onOff = false;
        error = cam.SetTriggerMode( &triggerMode );
        if (error != FlyCapture2::PGRERROR_OK)
        {
            //PrintError( error );
            res = 0;
        }
        std::cout << "Disconnect \n" << std::endl;
        error = cam.Disconnect();
        if (error != FlyCapture2::PGRERROR_OK)
        {
            //PrintError( error );
            res = 0;
        }
        return res;
    }

    bool flea3Driver::setup_camera()
    {
        FlyCapture2::Error error;
        uint32_t errCount = 0;
        error = cam.GetProperty(&camProperty);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            std::cout << "GetProperty Wrong" << std::endl;
            errCount ++;
        }
         std::cout<<"\n*** PROPERTY INFORMATION ***\n"
                    "type - " << camProperty.type << std::endl <<
                    "abs control - " << camProperty.absControl << std::endl <<
                    "one push - " << camProperty.onePush << std::endl <<
                    "onoff - " << camProperty.onOff << std::endl <<
                    "auto manual mode - " << camProperty.autoManualMode << std::endl <<
                    "abs value - " << camProperty.absValue << std::endl;
        // Set camera to trigger mode 0
        triggerMode.onOff = true;
        triggerMode.mode = trigger_mode;     //0 is camera external trigger
        triggerMode.parameter = 0;
        triggerMode.source = 0;
        triggerMode.polarity = 0;            //falling edge
        error = cam.SetTriggerMode( &triggerMode );
        if (error != FlyCapture2::PGRERROR_OK)
        {
            std::cout << "set triggerMode Wrong" << std::endl;
            errCount ++;
        }

        // error = cam.GetTriggerMode( &triggerMode );
        // if (error != FlyCapture2::PGRERROR_OK)
        // {
        //     std::cout << "Get triggerMode Wrong" << std::endl;
        //     errCount ++;
        // }
        // std::cout<<"\n*** TRIGGER INFORMATION ***\n"
        //            "onoff - " << triggerMode.onOff << std::endl <<
        //            "mode - " << triggerMode.mode << std::endl <<
        //            "source - " << triggerMode.source << std::endl <<
        //            "polarity - " << triggerMode.polarity << std::endl;
        error = cam.StartCapture();
        if ( error != FlyCapture2::PGRERROR_OK )
        {
            std::cout << "Start Capture Wrong" << std::endl;
            errCount ++;
        }
        else{
        std::cout << "Start Capturing ..." << std::endl;
        }

        if(errCount>0){
            return 0;
        }else{
            return 1;
        }
    }



} /* namespace flea3 */
