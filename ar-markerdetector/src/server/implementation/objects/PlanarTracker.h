#ifndef __PLANAR_TRACKER_H__
#define __PLANAR_TRACKER_H__

#include <opencv/cv.h>
#include "Marker.h"
#include <sys/time.h>

#include "Stopwatch.h"

class PlanarTracker{
 private:

  unsigned long getMillisecondsTime();

 public:
 PlanarTracker();


 void trackPlanars(cv::Mat& mat, std::vector<alvar::MarkerData>& detectedMarkerData);
#if 0
 void trackPlanars(cv::Mat& mat, std::vector<alvar::MarkerData>& detectedMarkerData, Stopwatch stopwatch, std::map<int, std::string> ticks);
#endif
  void solveProjectionMatrix(cv::Mat& mat);
  
  
  void createPlanarConfiguration(std::string tmpdir, int width, int height);
  
  void clear();
  
  std::string conf_planar;
  std::map<std::string, int> planar_image_ids;  
};

#endif
