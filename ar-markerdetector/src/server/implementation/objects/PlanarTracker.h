/**
Licensing and distribution

ArModule is licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

ALVAR 2.0.0 - A Library for Virtual and Augmented Reality Copyright 2007-2012 VTT Technical Research Centre of Finland Licensed under the GNU Lesser General Public License

Irrlicht Engine, the zlib and libpng. The Irrlicht Engine is based in part on the work of the Independent JPEG Group The module utilizes IJG code when the Irrlicht engine is compiled with support for JPEG images.
*/

/** @author Markus Ylikerälä */

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
