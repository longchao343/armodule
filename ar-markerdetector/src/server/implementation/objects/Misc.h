#ifndef __MISC_H__
#define __MISC_H__

#if 1
#define SMART_TIMESTAMP(msg, time); {std::cout << std::endl << "***SMART " << msg << "\t" << time << std::endl;}
#else
#define SMART_TIMESTAMP(msg, time); /**/
#endif


#endif
