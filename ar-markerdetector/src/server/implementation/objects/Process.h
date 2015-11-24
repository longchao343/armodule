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

#include "Misc.h"

#include "irrlicht.h"
#include <sys/time.h>

G_BEGIN_DECLS

using namespace kurento;
using namespace module;
using namespace armarkerdetector;
using namespace irr;


#include "PlanarTracker.h"

#include "Stopwatch.h"

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
  Stopwatch stopwatch;

  void setPoseScale(irr::scene::IAnimatedMeshSceneNode* node, float value);
  void setPosePosition(int id, float value, int type);
  void setPoseRotation(int id, float value, int type);
  void generatePoseEvents(sigc::signal<void, MarkerPose> events,  std::shared_ptr<MediaObject> mo, unsigned long timeStamp, cv::Mat& mat);
  void generateCountEvents(sigc::signal<void, MarkerCount> events, std::shared_ptr<MediaObject> mo, unsigned long timeStamp);

  std::map<int, int> fixedAugmentables; 
  std::map<int, int> detectedMarkers; 
  std::map<int, int> detectedMarkersPrev;

  PlanarTracker *planarTracker;

  std::vector<alvar::MarkerData> detectedMarkerData;
  double pmatrix[16];    
  alvar::Camera cam;

  void initAr(cv::Mat& mat);
  void parseAugmentable(ArThing *arThing, std::map<std::string, std::string>& strings, std::map<std::string, float>& floats);
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
    stopwatch.start();
  }

  /*
  void stop(){
    stopWatch.stop();
  }   
  */
  unsigned long getElapsedTime(){    
    return stopwatch.getElapsedTime();
  }

  unsigned long getSubTime(){
    return stopwatch.getSubTime();
  }


  void resetFilterTime(){
    stopwatch.resetFilterTime();
  }

  unsigned long  getFilterTime(){
    return stopwatch.getFilterTime();
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

  float getTimeStamp(){
    return 667;
  }
};

G_END_DECLS

#endif /* __PROCESS_H__ */
