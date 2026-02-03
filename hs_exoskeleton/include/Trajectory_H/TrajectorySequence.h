#ifndef TRAJECTORY_SEQUENCE_H
#define TRAJECTORY_SEQUENCE_H
//C system headers

//C++ standard library headers
#include <fstream>
#include <string>

//other libraries' headers
#include <Eigen/Core>

//project's headers

using namespace Eigen;

template<int dim>
class TrajectorySequence
{
protected:
    int N = 0;
    double T = 0.005;
    Matrix<double, dim, Dynamic> x;
    Matrix<double, dim, Dynamic> dx;
    Matrix<double, dim, Dynamic> ddx;
public:
    TrajectorySequence()
    {
    }

    TrajectorySequence(int length, double samplingPeriod = 0.005) :N(length), T(samplingPeriod)
    {
        x.resize(dim, length + 1);
        dx.resize(dim, length + 1);
        ddx.resize(dim, length + 1);

    }

    TrajectorySequence(const TrajectorySequence<dim>& TrajSeq) :TrajectorySequence(TrajSeq.seqLen(), TrajSeq.samplingPeriod())
    {
        x = TrajSeq.posSeq();
        dx = TrajSeq.velSeq();
        ddx = TrajSeq.accSeq();
    }

    ~TrajectorySequence()
    {
    }

    TrajectorySequence& operator=(const TrajectorySequence<dim>& TrajSeq)
    {
        N = TrajSeq.seqLen();
        T = TrajSeq.samplingPeriod();
        x = TrajSeq.posSeq();
        dx = TrajSeq.velSeq();
        ddx = TrajSeq.accSeq();
        return *this;
    }

    TrajectorySequence& operator+=(const TrajectorySequence<dim>& TrajSeq)
    {
        Matrix<double, dim, Dynamic> xbf;
        Matrix<double, dim, Dynamic> dxbf;
        Matrix<double, dim, Dynamic> ddxbf;

        xbf = x.leftCols(N);
        dxbf = dx.leftCols(N);
        ddxbf = ddx.leftCols(N);

        N += TrajSeq.seqLen();

        x.resize(dim, N + 1);
        dx.resize(dim, N + 1);
        ddx.resize(dim, N + 1);

        x << xbf, TrajSeq.posSeq();
        dx << dxbf, TrajSeq.velSeq();
        ddx << ddxbf, TrajSeq.accSeq();

        return *this;
    }

    virtual void writeTrajSeq2Text(const std::string& dir) const
    {
        std::string path;

        {
            path = dir + "\\position.txt";
            std::ofstream file(path.c_str());
            file << x.transpose();
            file.close();
        };

        {
            path = dir + "\\velocity.txt";
            std::ofstream file(path.c_str());
            file << dx.transpose();
            file.close();
        };

        {
            path = dir + "\\acceleration.txt";
            std::ofstream file(path.c_str());
            file << ddx.transpose();
            file.close();
        };
    }

    const int& seqLen() const
    {
        return N;
    }

    const double& samplingPeriod() const
    {
        return T;
    }

    const Matrix<double, dim, 1>& pos(int i) const
    {
        return x.col(i);
    }
    const Matrix<double, dim, 1>& vel(int i) const
    {
        return dx.col(i);
    }
    const Matrix<double, dim, 1>& acc(int i) const
    {
        return ddx.col(i);
    }

    const Matrix<double, dim, 1>& posStart() const
    {
        return x.col(0);
    }

    const Matrix<double, dim, 1>& posEnd() const
    {
        return x.col(N);
    }

    template<typename T>
    void setPosSeq(int i, const T& vec) {
        x.col(i) = vec;
    }
    template<typename T>
    void setVelSeq(int i, const T& vec) {
        dx.col(i) = vec;
    }
    template<typename T>
    void setAccSeq(int i, const T& vec) {
        ddx.col(i) = vec;
    }


    const Matrix<double, dim, Dynamic>& posSeq() const {
        return x;
    }
    const Matrix<double, dim, Dynamic>& velSeq() const {
        return dx;
    }
    const Matrix<double, dim, Dynamic>& accSeq() const {
        return ddx;
    }

    virtual void reset(int length, double samplingPeriod = 0.005) {
        N = length;
        T = samplingPeriod;
        x.resize(dim, length + 1);
        dx.resize(dim, length + 1);
        ddx.resize(dim, length + 1);
    }

    virtual void diff() {
        dx.col(0).setZero();
        for (int i = 1; i <= N; ++i)
            dx.col(i) = (x.col(i) - x.col(i - 1)) / T;

        ddx.col(0).setZero();
        for (int i = 1; i <= N; ++i)
            ddx.col(i) = (dx.col(i) - dx.col(i - 1)) / T;
    }
};

#endif // ! TRAJECTORY_SEQUENCE_H

