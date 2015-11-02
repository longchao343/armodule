#ifndef __PROCESS_H__
#define __PROCESS_H__

#include <glib.h>
#include "pthread.h"
#include <opencv/cv.h>

#include "ArMarkerdetector.hpp"
#include "Marker.h"
#include "ArThing.hpp"
#include "ArKvpString.hpp"
#include "ArKvpFloat.hpp"
#include <memory>

#if 0
#define SMART_TIMESTAMP(msg, time); {std::cout << std::endl << "***SMART " << msg << "\t" << time << std::endl;}
#else
#define SMART_TIMESTAMP(msg, time); /**/
#endif

#include "irrlicht.h"
#include <sys/time.h>

G_BEGIN_DECLS

using namespace kurento;
using namespace module;
using namespace armarkerdetector;
using namespace irr;

#include <time.h>

class StopWatch{
 private:
#if 0
  unsigned long startTime;
  unsigned long stopTime;
  unsigned long subTime;
#endif

  double startTime;
  double stopTime;
  double subTime;
  double filterTime;

  double getTime(){
    timespec tick;
    clock_gettime(CLOCK_MONOTONIC, &tick);
    return (tick.tv_sec * 1000) + (tick.tv_nsec)/1000000.0;
  }

#if 0
  unsigned long getTime(){
    struct timeval tv;
    if(gettimeofday(&tv, NULL) != 0){
      return 0;
    }
    return (unsigned long)((tv.tv_sec * 1000ul) + (tv.tv_usec / 1000ul));
  }
#endif

 public:
  StopWatch(){
    start();
  }

  void start(){
    startTime = getTime();
    subTime = startTime;
  }

  /*
  void stop(){
    stopTime = getTime();
  }   
  */

#if 0
  unsigned long getElapsedTime(){
    return getTime() - startTime;
  }

  unsigned long getSubTime(){
    unsigned long tmp = getTime();
    unsigned long subElapsed = tmp - subTime;
    subTime = tmp;
    return subElapsed;
  }
#endif

  double getElapsedTime(){
    return getTime() - startTime;
  }

  double resetFilterTime(){
    filterTime = getTime();
  }
  double getFilterTime(){
    return getTime() - filterTime;
  }

  double getSubTime(){
    double tmp = getTime();
    double subElapsed = tmp - subTime;
    subTime = tmp;
    return subElapsed;
  }

};

class ArProcess {
 private:
  int markerPoseFrameFrequency;
  int markerPoseFrame;
  float markerPoseFrequencyMs;
  float markerPoseFrequency;
  int isArInitialized;
  int width;
  int height;
  bool forTheVeryFirstTime;
  long veryFirstTime;
  bool reboot;
  unsigned long previousTime;

  int ardummy;
  int sizeFlag;
  std::map<int, irr::scene::IAnimatedMeshSceneNode*> nodes;
  std::map<int, irr::scene::IAnimatedMeshSceneNode*> fixedNodes;
  irr::IrrlichtDevice *device;
  irr::video::IVideoDriver* driver;
  irr::scene::ISceneManager* smgr;
  irr::scene::ILightSceneNode* light;
  irr::video::ITexture* rt;
  irr::scene::ICameraSceneNode* fixedCam;
  int countCheck;
  int count;
  long oldTimer;
  static int counter;

  std::map<int, core::vector3df> positions;
  std::map<int, core::vector3df> rotations;
  
  std::map<int, std::shared_ptr<ArThing>> arThings;
  std::map<int, cv::Mat> flats;
  std::map<int, int> flatsScale;
  bool dummyLoad;
  int augment2DModel(IplImage* img);
  int augment3DModel(cv::Mat& mat);
  void populate();
  void erasePopulation();
  irr::scene::IAnimatedMeshSceneNode* loadNode(std::string modelUrl, std::string& modelSuffix);
  void modifyNode(ArThing *arThing, std::map<std::string, std::string>& strings, std::map<std::string, float>& floats, irr::scene::IAnimatedMeshSceneNode* node, std::string modelSuffix);

  bool isNthFrame();  
  bool poseEventTimeout(unsigned long timeStamp);
  bool readImage(std::string url, cv::Mat& bg);
  int mShowDebugLevel;
  pthread_mutex_t mMutex;
  void *owndata;
  StopWatch stopWatch;

  void setPoseScale(irr::scene::IAnimatedMeshSceneNode* node, float value);
  void setPosePosition(int id, float value, int type);
  void setPoseRotation(int id, float value, int type);
  void generatePoseEvents(sigc::signal<void, MarkerPose> events,  std::shared_ptr<MediaObject> mo, unsigned long timeStamp, cv::Mat& mat);
  void generateCountEvents(sigc::signal<void, MarkerCount> events, std::shared_ptr<MediaObject> mo, unsigned long timeStamp);

  std::map<int, int> fixedAugmentables; 
  std::map<int, int> detectedMarkers; 
  std::map<int, int> detectedMarkersPrev;

#ifdef USE_MARKERLESS
  std::string conf_planar;
  std::map<std::string, int> planar_image_ids;
#endif

  std::vector<alvar::MarkerData> detectedMarkerData;
  double pmatrix[16];    
  alvar::Camera cam;

  void initAr(cv::Mat& mat);
  void parseAugmentable(ArThing *arThing, std::map<std::string, std::string>& strings, std::map<std::string, float>& floats);
  void createPlanarConfiguration();
  void solveProjectionMatrix(cv::Mat& mat);

  bool splitModel(std::string modelUrl, std::string &modelLocal, std::string &modelSuffix);
  bool fetchModel(std::string modelUrl, std::string &modelName);
  std::string tmpdir;
  void animate(irr::scene::IAnimatedMeshSceneNode* node, std::map<std::string, std::string> strings, std::string modelSuffix);
  void show(int isrowmajor, irr::core::CMatrix4<f32> m, std::string label);

  void addFgWithAlpha(cv::Mat &bg, cv::Mat &fg);
  bool load_from_url(const char *file_name, const char *url);
  double getMillisecondsTime2();
  unsigned long getMillisecondsTime();

public:
  ArProcess();
  ~ArProcess();


  void start(){
    stopWatch.start();
  }

  /*
  void stop(){
    stopWatch.stop();
  }   
  */
  unsigned long getElapsedTime(){    
    return stopWatch.getElapsedTime();
  }

  unsigned long getSubTime(){
    return stopWatch.getSubTime();
  }


  void resetFilterTime(){
    stopWatch.resetFilterTime();
  }

  unsigned long  getFilterTime(){
    return stopWatch.getFilterTime();
  }

  void detect(cv::Mat& mat);
  void generateEvents(std::shared_ptr<MediaObject> mo, sigc::signal<void, MarkerPose> signalMarkerPose, sigc::signal<void, MarkerCount> signalMarkerCount, cv::Mat& mat);

  cv::Mat set_overlay(std::string overlay_image, 
		      std::string overlay_text, 
		      float scale);
  void setShowDebugLevel(int level) { mShowDebugLevel = level; }

  bool augmentationEnabled;
  bool markerCountEnabled;
  bool markerPoseFrequencyEnabled;
  bool markerPoseFrameFrequencyEnabled;
  int sequenceNum;


  void writeImage(std::string name, cv::Mat& mat);
  void proactivate(cv::Mat& mat);
  void augment(cv::Mat& mat);

  void getTimeStamp(unsigned long *timestamp);

  bool isPoseEventTime(unsigned long timestamp){
    if(markerPoseFrameFrequencyEnabled && isNthFrame() ||
       markerPoseFrequencyEnabled && poseEventTimeout(timestamp)){
      return true;
    }
    return false;
  }

  void setPose (int id, int type, float value);

  void enableAugmentation (bool enableAugmentation){
    augmentationEnabled = enableAugmentation;
  }
  void enableMarkerCountEvents (bool enable){
    markerCountEnabled = enable;
  }

  void enableAugmentationSet (const std::vector<int> &enableAugmentation){
  }

  void disableAugmentationSet (const std::vector<int> &disableAugmentation){
  }

  void setMarkerPoseFrequency (bool enable, float frequency){
    markerPoseFrequencyEnabled = enable;
    markerPoseFrequency = frequency;
  }
  void setMarkerPoseFrameFrequency (bool enable, int frequency){
    markerPoseFrameFrequencyEnabled = enable;
    markerPoseFrameFrequency = frequency;
  }

  void setArThing (const std::vector<std::shared_ptr<ArThing>> setArThing);
};

G_END_DECLS

#endif /* __PROCESS_H__ */
