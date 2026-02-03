#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

//C system headers

//C++ standard library headers

//other libraries' headers

//project's headers
#include "Controller.h"

using std::string;
using namespace Eigen;

template<int n>
class PIDController :public Controller<n, n>
{
private:
	Matrix<double, n, 1> Kp, Ki, Kd;
	Matrix<double, n, 1> up, ui, ud;
	Matrix<double, n, 1> Intergral_e;

	WriteDataIntoText RecorderUp;
	WriteDataIntoText RecorderUi;
	WriteDataIntoText RecorderUd;
	WriteDataIntoText RecorderU;

public:
	PIDController(double ControlCycle) :
		Controller<n, n>(ControlCycle),
		Kp(Eigen::MatrixXd::Zero(n, 1)),
		Ki(Eigen::MatrixXd::Zero(n, 1)),
		Kd(Eigen::MatrixXd::Zero(n, 1)),
		Intergral_e(Eigen::MatrixXd::Zero(n, 1)),
		up(Eigen::MatrixXd::Zero(n, 1)),
		ui(Eigen::MatrixXd::Zero(n, 1)),
		ud(Eigen::MatrixXd::Zero(n, 1))
	{
	}

	~PIDController()
	{
	}

	void setPIDparameters(const Matrix<double, n, 1>& Kp, const Matrix<double, n, 1>& Ki, const Matrix<double, n, 1>& Kd)
	{
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}
	void setPIDparameters(const std::string& path)
	{	
		ReadDataFromText(n, (double*)&Kp, (path + "\\Kp.txt").c_str());
		ReadDataFromText(n, (double*)&Ki, (path + "\\Ki.txt").c_str());
		ReadDataFromText(n, (double*)&Kd, (path + "\\Kd.txt").c_str());

	}
	void setDataRecordingPath(const std::string& DataRecordingPath)
	{
		Controller<n, n>::setDataRecordingPath(DataRecordingPath);
		RecorderUp.setPath(this->dir + "\\Up.txt");
		RecorderUi.setPath(this->dir + "\\Ui.txt");
		RecorderUd.setPath(this->dir + "\\Ud.txt");
		RecorderU.setPath(this->dir + "\\U.txt");
	}
	void dataRecording()
	{
		RecorderUp(up);
		RecorderUi(ui);
		RecorderUd(ud);
		RecorderU(this->u);
	}
	void stopRecording()
	{
		RecorderUp.ending();
		RecorderUi.ending();
		RecorderUd.ending();
		RecorderU.ending();
	}
	const Matrix<double, n, 1>& operator()(const Matrix<double, n, 1>& e, const Matrix<double, n, 1>& de)
	{
		up = Kp.cwiseProduct(e);
		
		Intergral_e += e * this->ControlCycle;
		ui = Ki.cwiseProduct(Intergral_e);
		
		ud = Kd.cwiseProduct(de);
		
		this->u = up + ui + ud;

		return this->u;
	}
	const Matrix<double, n, 1>& operator()()
	{
		return this->u;
	}

	const Matrix<double, n, 1>& getUp() const { return up; }
	const Matrix<double, n, 1>& getUi() const { return ui; }
	const Matrix<double, n, 1>& getUd() const { return ud; }
	const Matrix<double, n, 1>& getU() const { return this->u; }
};

#endif // !PID_CONTROLLER_H