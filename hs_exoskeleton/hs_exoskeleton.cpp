//C system headers

//C++ standard library headers
#include <iostream>
#include <chrono>
#include <thread>


//other libraries' headers

//project's headers
#include "kvaser.h"
#include "motors.h"
#include "Timer.h"
#include "selfDevManuipulator.h"
#include "PIDController.h"
#include "Sinusoid.h"
#include "DataRecorder.h"

using namespace std;

void MotionTest() 
{
	double gait_period = 0.75;//步态周期s
	int num_gait = 20;//步态次数
	double duration = gait_period * num_gait;//总运动时间
	double step_size = 0.005;//控制周期s
	int cnt = 0;
	int cnt_max = 3022;//最大循环次数

	//Vector2d amp(DEG2RAD(25), DEG2RAD(25));//最大关节角

	MotorParameters;//设置电机参数
	selfDevManuipulator<2> hip;//创建髋关节对象
	hip.setMotor(motor);
	hip.CANBus().canInit(0, canBITRATE_1M);//总线初始化

	long id_get_L = 0x0001;
	long id_get_R = 0x0002;
	uint8_t CtlDat = 0x01;
	uint8_t CtlDat_dis = 0x00;
	hip.CANBus().MotorOnOff(id_get_L, CtlDat);
	hip.CANBus().MotorOnOff(id_get_R, CtlDat);

	PIDController<2> hip_controller(step_size);//创建PID控制对象，控制周期5ms
	hip_controller.setPIDparameters(R"(C:\Users\admin\Desktop\hs_exoskeleton\Data\PIDController\PlD_Parameters)");//从文件中读取PID参数

	hip.statusUpdate();//状态更新

	/*Sinusoid<2> traj_des(amp * 0.5, Vector2d::Constant(gait_period), hip.Ang() + Vector2d(0, 0), Vector2d(PI / 2, -PI / 2));*/
	string path_traj_des(R"(C:\Users\admin\Desktop\hs_exoskeleton\Data\Desired Trajectory)");//轨迹路径
	/*traj_des.writeTraj2Text(path_traj_des, step_size, duration);*/

	//设置轨迹阅读器
	Vector2d q_des, dq_des;//位置误差，速度误差
	//ReadDataFromText q_des_reader(path_traj_des+R"(\position.txt)"), dq_des_reader(path_traj_des + R"(\velocity.txt)");
	ReadDataFromText q_des_reader(path_traj_des + R"(\q_test_rad.txt)"), dq_des_reader(path_traj_des + R"(\dq_test_rad.txt)");
	q_des_reader(q_des);
	dq_des_reader(dq_des);


	//数据记录
	DataRecorder recorder;
	recorder.addRecorder(hip, R"(C:\Users\admin\Desktop\hs_exoskeleton\Data\Robot Status)")
		    .addRecorder(hip_controller, R"(C:\Users\admin\Desktop\hs_exoskeleton\Data\PIDController\Output)");
	recorder();

	Vector2d error_vec, d_error_vec;//位置误差，速度误差

	Vector2d trq_tmp(1, 1);

	while (cnt < cnt_max) 
	{
		cnt++;

		hip.statusUpdate();

		q_des_reader(q_des);
		dq_des_reader(dq_des);

		error_vec = q_des - hip.Ang();
		d_error_vec = dq_des - hip.AngVel();

		hip.trqCtrl(hip_controller(error_vec, d_error_vec));

		//cout << hip_controller(error_vec, d_error_vec) << endl;
		//std::this_thread::sleep_for(std::chrono::milliseconds(50));
			
		recorder();
	}

	hip.brake();
	hip.CANBus().MotorOnOff(id_get_L, CtlDat_dis);
	hip.CANBus().MotorOnOff(id_get_R, CtlDat_dis);
}

int main() {

	MotionTest();

	return 0;
}
 