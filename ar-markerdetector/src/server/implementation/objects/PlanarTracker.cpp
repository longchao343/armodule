/**
Licensing and distribution

ArModule is licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

ALVAR 2.0.0 - A Library for Virtual and Augmented Reality Copyright 2007-2012 VTT Technical Research Centre of Finland Licensed under the GNU Lesser General Public License

Irrlicht Engine, the zlib and libpng. The Irrlicht Engine is based in part on the work of the Independent JPEG Group The module utilizes IJG code when the Irrlicht engine is compiled with support for JPEG images.
*/

/** @author Markus Ylikerälä */

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

