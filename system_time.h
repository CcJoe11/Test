
#ifndef _PLATFORM_H_
#define _PLATFORM_H_

typedef double systime;

void getsystime(systime &t);
double getTimeIntervalms(const systime &now, const systime &last);

#if defined(Linux) || defined(Darwin)
    #include <sys/time.h>
#elif defined(Windows)
    #include <Windows.h>
#else
//    #error "nonsupport platform."
#endif

#endif /* _PLATFORM_H_ */
