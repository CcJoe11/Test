//
// Created by xinyang on 19-7-31.
//
#include<system_time.h>
#include<stdlib.h>
#include<sys/time.h>
#include<unistd.h>
//#if defined(Linux) || defined(Darwin)
static systime getsystime(){
    timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec / 1000.0 + tv.tv_sec * 1000.0;
}

void getsystime(systime &t) {
    static systime time_base = getsystime();
    timeval tv;
    gettimeofday(&tv, NULL);
    t = tv.tv_usec / 1000.0 + tv.tv_sec * 1000.0 - time_base;
}

double getTimeIntervalms(const systime &now, const systime &last) {
    return now - last;
}
