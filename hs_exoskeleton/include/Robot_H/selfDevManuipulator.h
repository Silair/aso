#ifndef SELFDEVMANUIPULATOR_H
#define SELFDEVMANUIPULATOR_H


#include <iostream>
#include <functional>
#include <cmath>
#include <string>


#include <Eigen/Core>


#include "kvaser.h"
#include "TEXTIO.h"
#include "Constants.h"
#include "Motor.h"
#include "UnitConv.h"
#include "Timer.h"
#include <stdlib.h>


#include <iostream>
#include <cstring>


#include "canlib.h"


#include "CANMessage.h"

using namespace std;
using namespace Eigen;


template<int DOF>
class selfDevManuipulator
{
    private:
        Kvaser kcan;
        Motor motor[DOF];

        Matrix<double, DOF, 1>q, dq, sinq, cosq;

        WriteDataIntoText RecAng;
        WriteDataIntoText RecAngVel;

        ReadDataFromText ReadAng;
        ReadDataFromText ReadAngVel;

        bool isExceptionOccurred = false;

    public:
        selfDevManuipulator()
        {
            kcan = Kvaser();
            q.setZero();
        }
        ~selfDevManuipulator()
        {
            brake();
        }

        Kvaser& CANBus()
        {
            return kcan;
        }


        void setMotor(int i, const Motor& m)
        {
            motor[i] = m;
        }

        void setMotor(Motor* m)
        {
            for (int i = 0; i < DOF; ++i) motor[i] = m[i];
        }

        void setDataRecordingPath(const string& path)
        {
            RecAng.setPath(path + "\\q.txt");
            RecAngVel.setPath(path + "\\dq.txt");
        }


        void setDataReadingPath(const string& path)
        {
            ReadAng.setPath(path + "\\q.txt");
            ReadAngVel.setPath(path + "\\dq.txt");
        }


        void dataRecording()
        {
            RecAng(q);
            RecAngVel(dq);
        }

        void stopRecording()
        {
            RecAng.ending();
            RecAngVel.ending();
        }


        void statusUpdate(const Matrix<double, DOF, 1>& theta, const Matrix<double, DOF, 1>& dtheta = Matrix<double, DOF, 1>::Zero())
        {
            q = theta;
            dq = dtheta;
            sinq = q.array().sin();
            cosq = q.array().cos();
        }

        void statusUpdate()
        {
            float cur_position = 0;
            float cur_speed = 0;

            for (int i = DOF - 1; i >= 0; --i)
            {
                if (kcan.MotorGetReg(motor[i].ID(), MOTOR_REG_POSITION_MEAS_DEGREE, (uint32_t *)&cur_position) == 0 &&
                    kcan.MotorGetReg(motor[i].ID(), MOTOR_REG_SPEED_MEAS_RPM, (uint32_t *)&cur_speed) == 0){
                   
                    q[i]=DEG2RAD(cur_position);
                    dq[i]=RPM2RAD(cur_speed);
                } else {
                    brake();  
                }
                sinq = q.array().sin();
                cosq = q.array().cos();
            }
        }


        bool motorStatusException(int index)
        {
            if (q[index] > motor[index].limiter.p_ub)
            {
                cout << "Motor " << index << " breaks position upper bound! " << q[index] << '\n';
                return true;
            }
            else if (q[index] < motor[index].limiter.p_lb)
            {
                cout << "Motor " << index << " breaks position lower bound! " << q[index] << '\n';
                return true;
            }
            else if (dq[index] > motor[index].limiter.v_ub || (-dq[index]) > motor[index].limiter.v_ub)
            {
                cout << "Motor " << index << " breaks velocity bound! " << dq[index] << '\n';
                return true;
            }
            else
                return false;
        }

        void readStatus()
        {
            ReadAng((double*)&q, DOF);
            ReadAngVel((double*)&dq, DOF);
            statusUpdate(q, dq);
        }


        const double& Ang(const int& i)
        {
            return q[i];
        }
        const double& AngVel(const int& i)
        {
            return dq[i];
        }
        const Eigen::Matrix<double, DOF, 1>& Ang()
        {
            return q;
        }
        const Eigen::Matrix<double, DOF, 1>& SinAng()
        {
            return sinq;
        }
        const Eigen::Matrix<double, DOF, 1>& CosAng()
        {
            return cosq;
        }
        const Eigen::Matrix<double, DOF, 1>& AngVel()
        {
            return dq;
        }


        template<typename T>
        void trqCtrl(const T& trq)
        {
            for (int i = DOF - 1; i >= 0; --i){

                kcan.torqueMode(0,motor[i].id,SELFMOTORCUR2CNT(trq[i])) ;
            }
        }

        void trqCtrl(int i, double trq) {
            kcan.torqueMode(0,motor[i].id,SELFMOTORCUR2CNT(trq));
        }

        template<typename T>
        void velCtrl(const T& vel, const T& cur)
        {
            for (int i = DOF - 1; i >= 0; --i)
                kcan.speedMode(0,motor[i].ID(),RAD2RPM(vel[i]),SELFMOTORCUR2CNT(cur[i]));  
        }

        void velCtrl(int i, double vel,double cur)
        {

            kcan.speedMode(0,motor[i].ID(),RAD2RPM(vel),SELFMOTORCUR2CNT(cur)); 
        }


        template<typename T1, typename T2, typename T3>
        void posVelCtrl(const T1& pos, const T2& vel, const T3& cur)
        {
            for (int i = DOF - 1; i >= 0; --i)
                kcan.positionMode(0, motor[i].ID(), RAD2DEG(pos[i]), RAD2RPM(vel[i])*10, SELFMOTORPOSMODECURLIMIT(cur[i]));

        }

        void posVelCtrl(int i, double pos, double vel, double cur)
        {
            kcan.positionMode(0, motor[i].ID(), RAD2DEG(pos), RAD2RPM(vel)*10, SELFMOTORPOSMODECURLIMIT(cur));
        }

        void brake()
        {
            for (int i = 0; i < DOF; ++i)
                kcan.selfDevMotorStop(motor[i].id);
        }

        bool exception()
        {
            return isExceptionOccurred;
        }
};
#endif 