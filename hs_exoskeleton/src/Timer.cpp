#include "Timer.h"

#if  RUN_ON_WINDOWS
/*windows 的微秒级 usleep 封装*/
void Timer::usleep(double uDelay)
{
    LARGE_INTEGER litmp;
    LONGLONG QPart1, QPart2;
    double dfMinus, dfFreq, dfTim;
    QueryPerformanceFrequency(&litmp);
    dfFreq = (double)litmp.QuadPart;
    QueryPerformanceCounter(&litmp);
    QPart1 = litmp.QuadPart;
    do
    {
        QueryPerformanceCounter(&litmp);
        QPart2 = litmp.QuadPart;
        dfMinus = (double)(QPart2 - QPart1);
        if (dfMinus < 0)
        {
            break;
        }
        dfTim = dfMinus / dfFreq * 1000000;
    } while (dfTim < uDelay);
}

/*计时器*/
void Timer::begin(int delay_us)
{
    QueryPerformanceFrequency(&nFreq); nFreq_inv = 1.0 / nFreq.QuadPart;
    QueryPerformanceCounter(&nBeginTime);
    usleep(delay_us);
}
void Timer::out(double* t, double* dt)
{
    *dt = *t;
    QueryPerformanceCounter(&nEndTime);
    *t = (nEndTime.QuadPart - nBeginTime.QuadPart) * nFreq_inv;
    *dt = *t - *dt;
}
double Timer::operator()()
{
    QueryPerformanceCounter(&nEndTime);
    return (nEndTime.QuadPart - nBeginTime.QuadPart) * nFreq_inv;
}

double Timer::clock() {
    return this->operator()();
}
#else
void usleep(int uDelay)
{
    int time_use;

    timeval start;
    timeval end;

    gettimeofday(&start, NULL);

    do
    {
        gettimeofday(&end, NULL);
        time_use = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
    } while (time_use < uDelay);
}

#endif //  RUN_ON_WINDOWS