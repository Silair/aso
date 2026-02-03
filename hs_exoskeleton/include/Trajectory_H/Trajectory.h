#ifndef TRAJECTORY_H
#define TRAJECTORY_H
//C system headers

//C++ standard library headers

//other libraries' headers
#include <Eigen/Core> 

//project's headers
#include "TrajectorySequence.h"

using namespace Eigen;

template<int dim>
class Trajectory
{
protected:
    Matrix<double, dim, 1> x;
    Matrix<double, dim, 1> dx;
    Matrix<double, dim, 1> ddx;
    double CurrentTime = -1.0;
    TrajectorySequence<dim> TrajSeq;
public:
    Trajectory()
    {
        x.setZero();
        dx.setZero();
        ddx.setZero();
    }
    ~Trajectory()
    {
    }
    const Matrix<double, dim, 1>& pos(double t)
    {
        if (t != CurrentTime)
        {
            TrajectoryGenerator(t);
            CurrentTime = t;
        }
        return x;
    }
    const Matrix<double, dim, 1>& vel(double t)
    {
        if (t != CurrentTime)
        {
            TrajectoryGenerator(t);
            CurrentTime = t;
        }
        return dx;
    }
    const Matrix<double, dim, 1>& acc(double t)
    {
        if (t != CurrentTime)
        {
            TrajectoryGenerator(t);
            CurrentTime = t;
        }
        return ddx;
    }

    const TrajectorySequence<dim>& sampling(double dt, double t1, double t0 = 0.0)
    {
        int cnt_max = (int)((t1 - t0) / dt);
        TrajSeq.reset(cnt_max, dt);

        for (int i = 0; i <= cnt_max; ++i)
        {
            TrajSeq.setPosSeq(i, pos(i * dt + t0));
            TrajSeq.setVelSeq(i, vel(i * dt + t0));
            TrajSeq.setAccSeq(i, acc(i * dt + t0));
        }

        return TrajSeq;
    }

    const TrajectorySequence<dim>& sequence()
    {
        return TrajSeq;
    }

    void writeTraj2Text(const std::string& dir, double dt, double t1, double t0 = 0.0)
    {
        int cnt_max = (int)((t1 - t0) / dt);
        TrajectorySequence<dim> ts(cnt_max, dt);
        for (int i = 0; i <= cnt_max; ++i)
        {
            ts.setPosSeq(i, pos(i * dt + t0));
            ts.setVelSeq(i, vel(i * dt + t0));
            ts.setAccSeq(i, acc(i * dt + t0));
        }
        ts.writeTrajSeq2Text(dir);
    }

    virtual void TrajectoryGenerator(double t) = 0;
};

#endif //TRAJECTORY_H
