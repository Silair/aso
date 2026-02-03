#ifndef CONTROLLER_H
#define CONTROLLER_H

//C system headers

//C++ standard library headers
#include <string>

//other libraries' headers
#include <Eigen/Core>

//project's headers
#include "TEXTIO.h"

template<int n, int m>
class Controller
{
protected:
	double ControlCycle;

	Eigen::Matrix<double, n, 1> u;

	std::string dir;

public:
	Controller(double _ControlCycle) :ControlCycle(_ControlCycle), u(Eigen::MatrixXd::Zero(n, 1))
	{
	}

	~Controller()
	{
	}

	virtual void setDataRecordingPath(const std::string& DataRecordingPath)
	{
		dir = DataRecordingPath;
	}

};

#endif // !CONTROLLER_H