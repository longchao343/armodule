/**
Licensing and distribution

ArModule is licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

ALVAR 2.0.0 - A Library for Virtual and Augmented Reality Copyright 2007-2012 VTT Technical Research Centre of Finland Licensed under the GNU Lesser General Public License

Irrlicht Engine, the zlib and libpng. The Irrlicht Engine is based in part on the work of the Independent JPEG Group The module utilizes IJG code when the Irrlicht engine is compiled with support for JPEG images.
*/

/** @author Markus Ylikerälä */

#ifndef __STOPWATCH_H__
#define __STOPWATCH_H__

#include <time.h>

class Stopwatch{
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
  Stopwatch(){
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

#endif
