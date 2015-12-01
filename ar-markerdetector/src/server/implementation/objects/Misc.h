#ifndef __MISC_H__
#define __MISC_H__

#if 0
#if 1
//#define SMART_TIMESTAMP(msg, time); {std::cout << std::endl << "***SMART " << msg << "\t" << time << std::endl;}
#define SMART_TIMESTAMPOLD(msg, time); {ticks[time] = "***SMART " << msg << "\t";}
#else
#define SMART_TIMESTAMPOLD(msg, time); /**/
#endif
#endif

#endif
