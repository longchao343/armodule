#include "PlanarTracker.h"

PlanarTracker::PlanarTracker(){
}

void PlanarTracker::clear(){
  planar_image_ids.clear();
}

unsigned long PlanarTracker::getMillisecondsTime()
{
  struct timeval tv;
  if(gettimeofday(&tv, NULL) != 0){
    return 0;
  }
  return (unsigned long)((tv.tv_sec * 1000ul) + (tv.tv_usec / 1000ul));
}

void PlanarTracker::solveProjectionMatrix(cv::Mat& mat){
}

void PlanarTracker::trackPlanars(cv::Mat& mat, std::vector<alvar::MarkerData>& detectedMarkerData){
  }
#if 0
void PlanarTracker::trackPlanars(cv::Mat& mat, std::vector<alvar::MarkerData>& detectedMarkerData, Stopwatch stopwatch, std::map<int, std::string> ticks){
  }
#endif
  void PlanarTracker::createPlanarConfiguration(std::string tmpdir, int width, int height){

}

