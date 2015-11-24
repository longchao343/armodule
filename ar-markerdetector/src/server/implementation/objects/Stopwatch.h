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
