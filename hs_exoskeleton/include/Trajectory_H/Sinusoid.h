#ifndef SINUSOID_H
#define SINUSOID_H

//C system headers

//C++ standard library headers

//other libraries' headers

//project's headers
#include "Trajectory.h"

using namespace Eigen;

template<int dim>
class Sinusoid final : public Trajectory<dim>
{
private:
    Matrix<double, dim, 1> amplitude;
    Matrix<double, dim, 1> omega;
    Matrix<double, dim, 1> offset;
    Matrix<double, dim, 1> phase;
public:
    Sinusoid(const Eigen::Matrix<double, dim, 1>& AmpVec,//A
        const Eigen::Matrix<double, dim, 1>& PerVec,//T
        const Eigen::Matrix<double, dim, 1>& OffsetVec = MatrixXd::Zero(dim, 1),//����ƫ��
        const Eigen::Matrix<double, dim, 1>& PhaseVec = MatrixXd::Zero(dim, 1)) ://����
        Trajectory<dim>()
    {
        const double pi = 3.141592653589793;
        amplitude = AmpVec;
        omega = PerVec;
        omega = 2.0 * pi * omega.cwiseInverse();
        offset = OffsetVec;
        phase = PhaseVec;
    }

    Sinusoid() :Trajectory<dim>()
    {
        offset.setZero();
        phase.setZero();
    }

    Sinusoid& SetAmplitude(const Matrix<double, dim, 1>& AmpVec)
    {
        amplitude = AmpVec;
        return *this;
    }
    Sinusoid& SetPeriod(const Matrix<double, dim, 1>& PerVec)
    {
        omega = PerVec;
        omega = 2.0 * 3.141592653589793 * omega.cwiseInverse();
        return *this;
    }
    Sinusoid& SetOffset(const Matrix<double, dim, 1>& OffsetVec)
    {
        offset = OffsetVec;
        return *this;
    }

    Sinusoid& SetAmplitude(double Amp)
    {
        amplitude.fill(Amp);
        return *this;
    }
    Sinusoid& SetPeriod(double Per)
    {
        omega.fill(Per);
        omega = 2.0 * 3.141592653589793 * omega.cwiseInverse();
        return *this;
    }
    Sinusoid& SetOffset(double Offset)
    {
        offset.fill(Offset);
        return *this;
    }
    Sinusoid& SetPhase(double Phase)
    {
        phase.fill(Phase);
        return *this;
    }
    void TrajectoryGenerator(double t)//�켣������
    {
        //amplitude * sin(omega * t);
        this->x = amplitude.array() * ((omega * t).array() + phase.array()).sin() + offset.array();

        //amplitude * omega * cos(omega * t);
        this->dx = amplitude.array() * omega.array() * ((omega * t).array() + phase.array()).cos();

        //amplitude * omega * omega * (-sin(omega * t));
        this->ddx = amplitude.array() * omega.array().square() * (-((omega * t).array() + phase.array()).sin());
    }
};

#endif // !SINUSOID_H

