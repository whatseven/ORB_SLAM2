#ifndef COMMON_H
#define COMMON_H

#ifdef _WIN32
#include <time.h>
#else
#include <time.h>
#endif

inline void override_sleep(float micro_seconds)
{
#ifdef _WIN32
    _sleep(micro_seconds);
#else
    sleep(micro_seconds);
#endif
}

#endif