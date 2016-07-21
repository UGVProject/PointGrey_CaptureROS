#include "stereoCam.h"
#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;
bool CreateDirectory(char *datestr, char *timestr);
char timestampfile[100] ;

int main(int32_t argc, char **argv) {
  ros::init(argc, argv, "stereo_record_node");
  double shutter_speed;
  int numberofoutsync;
  char datestr[10];
  char timestr[10];
  time_t currentdate;
  time_t now;
  //struct tm *timenow;

  shutter_speed = 2.5;
  // shuttle_speed = atoi(argv[1]);
  time(&now);
  time(&currentdate);
  strftime(datestr, 20, "%F", localtime(&currentdate));
  //timenow = localtime(&now);
  strftime(timestr, 15, "%R", localtime(&now));
  if (!CreateDirectory(datestr, timestr)) {
    std::cout << "Directory is not created!" << std::endl;
    return -1;
  }
  // std::cout << "txtFilename is: " << timestampfile << std::endl;

  flea3::stereoCam m_camera;
  m_camera.setGuid(15231263, 15231302); // left and right UID
  m_camera.setPublishFrequency(50);
  m_camera.setShuttle(shutter_speed);
  m_camera.setFrameId("wide_camera");
  m_camera.set_TriggerMode_value(0);
  m_camera.setTopicName("wide/image_raw");
  m_camera.setTXTName(timestampfile);
  m_camera.run();

  return 0;
}

bool CreateDirectory(char *datestr, char *timestr) {
  char folderstring[17];
  char finalarray[50];
  folderstring[0] = datestr[0];
  folderstring[1] = datestr[1];
  folderstring[2] = datestr[2];
  folderstring[3] = datestr[3];
  folderstring[4] = datestr[4];
  folderstring[5] = datestr[5];
  folderstring[6] = datestr[6];
  folderstring[7] = datestr[7];
  folderstring[8] = datestr[8];
  folderstring[9] = datestr[9];
  folderstring[10] = '-';
  folderstring[11] = timestr[0];
  folderstring[12] = timestr[1];
  folderstring[13] = timestr[2];
  folderstring[14] = timestr[3];
  folderstring[15] = timestr[4];
  folderstring[16] = '\0';

  std::cout << "folderstring is: " << folderstring << std::endl;
  char *home = getenv("HOME");
  char pathex[30];
  sprintf(pathex, "%s/experiments", home);
  if (NULL == opendir(pathex))
    mkdir(pathex, 0777);
  sprintf(finalarray, "%s/%s", pathex, folderstring);
  mkdir(finalarray, 0777);
  //std::cout << "Folder created! " << std::endl;
  // makeing files
  // actually here we should check whether the directory exists using stat()
  struct stat statbuf;
  if (stat(finalarray, &statbuf) != -1) {
    if (S_ISDIR(statbuf.st_mode)) {
      sprintf(timestampfile, "%s/timestamp.txt", finalarray);
      return true;
    }
   }
   else {
   return false;
   }
  //return true;
}
