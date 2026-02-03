#ifndef TIMER_H
#define TIMER_H

#include "Macro.h"

#if RUN_ON_WINDOWS

#include <Windows.h>

class Timer
{
private:
    LARGE_INTEGER nFreq, nBeginTime, nEndTime;
    double nFreq_inv;
public:
    void begin(int delay_us = 0);
    void out(double* t, double* dt);
    double operator()();
    double clock();

    static void usleep(double uDelay);
};
#else

#include <sys/time.h>

void usleep(double uDelay);

class Timer
{
private:
    timeval start;
    timeval end;
public:
    void Begin(int delay_us = 0);
    void out(double* t, double* dt);
    double operator()();
};

void Timer::begin(int delay_us)
{
    gettimeofday(&start, NULL);
    end = start;
    usleep(delay_us);
}

void Timer::out(double* t, double* dt)
{
    timeval end_bf = end;
    gettimeofday(&end, NULL);
    *t = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) * 0.000001;
    *dt = (end.tv_sec - end_bf.tv_sec) + (end.tv_usec - end_bf.tv_usec) * 0.000001;
}

double Timer::operator()()
{
    gettimeofday(&end, NULL);
    return (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) * 0.000001;
}

#endif

#endif //TIMER_H