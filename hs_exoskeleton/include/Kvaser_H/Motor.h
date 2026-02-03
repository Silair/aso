#ifndef MOTOR_H
#define MOTOR_H
#include "Constants.h"

//struct MOTOR
//{
//    long id;
//    struct Encoder
//    {
//        int count;
//        int AbsZeroPos;
//    } encoder;
//    double InitPos;
//    double In;
//    double Wn;
//    double Kt_inv;
//    double direction;
//    uint8_t config;
//    struct Limiter
//    {
//        double p_lb;
//        double p_ub;
//        double v_ub;
//    } limiter;
//};//config bit0: is Limiter used? bit1: is tpdo used? bit2: is virtual node?


struct Motor
{
    long id = 0;
    
    struct Encoder
    {
        int count = 0;
        int AbsZeroPos = 0;
    } encoder;
    
    float In = 0;
    
    double Kt_inv = 0;

    struct Limiter
    {
        double p_lb = -1e+10;
        double p_ub = 1e+10;
        double v_ub = 1e+10;
    } limiter;

    bool IsReversed = false;

    bool IsVirtualNode = false;

    bool IsTPDOused = false;

    Motor()
    {
    }

    Motor(long id, Encoder encoder, float In, double Kt_inv, Limiter limiter, bool IsReversed = false, bool IsVirtualNode = false):
        id(id),
        encoder(encoder),
        In(In),
        Kt_inv(Kt_inv),
        limiter(limiter),
        IsReversed(IsReversed),
        IsVirtualNode(IsVirtualNode)
    {
    }

   /* const Motor& operator=(const Motor& m)
    {
        id = m.id;
        encoder = m.encoder;
        In = m.In;
        Kt_inv = m.Kt_inv;
        limiter = m.limiter;
        IsReversed = m.IsReversed;
        IsVirtualNode = m.IsVirtualNode;
        IsTPDOused = m.IsTPDOused;
        return *this;
    }*/
    

    const long& ID() const
    {
        return id;
    }

    void virtualize()
    {
        IsVirtualNode = true;
    }

    void realize()
    {
        IsVirtualNode = false;
    }

    void SetTPDO()
    {
        IsTPDOused = true;
    }

    double cnt2pos(int cnt) const
    {
        if (IsReversed)
            return (double)(-cnt + (encoder.AbsZeroPos)) * DBPI / encoder.count;
        else
            return (double)(cnt - (encoder.AbsZeroPos)) * DBPI / encoder.count;
    }

    double cnt2vel(int cnt) const
    {
        if (IsReversed)
            return (double)(-cnt) * DBPI / encoder.count;
        else
            return (double)(cnt)*DBPI / encoder.count;
    }

    int32_t pos2cnt(double pos) const
    {
        if (pos > limiter.p_ub)pos = limiter.p_ub;
        if (pos < limiter.p_lb)pos = limiter.p_lb;

        int32_t cnt = pos * encoder.count / DBPI;

        if (IsReversed)
            return encoder.AbsZeroPos - cnt;
        else
            return encoder.AbsZeroPos + cnt;
    }

    int32_t vel2cnt(double vel) const
    {
        if (vel > limiter.v_ub)vel = limiter.v_ub;
        if (vel < -limiter.v_ub)vel = -limiter.v_ub;

        int32_t cnt = vel * encoder.count / DBPI;

        if (IsReversed)
            return  -cnt;
        else
            return cnt;
    }

    int32_t sp2cnt(double sp) const
    {
        if (sp > limiter.v_ub)
            return limiter.v_ub * encoder.count / DBPI;
        else
            return sp * encoder.count / DBPI;
    }

    float torque2current(double torque) const
    {
        float current = Kt_inv * torque;
        if (IsReversed)current = -current;

        if (current > In)return In;
        if (current < -In)return -In;

        return current;
    }

    int16_t torque2CurrentPermillage(double torque) const
    {
        //return torque2current(torque) * 1000.0 / (In * 1.4142135623731);
        return torque2current(torque) * 1000.0 / In;
    }

};

#endif // !MOTOR_H

