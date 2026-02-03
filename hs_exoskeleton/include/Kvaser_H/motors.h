#ifndef MOTORS_H
#define MOTORS_H
#include "UnitConv.h"
//电机参数
#define MotorParameters \
Motor motor[2];\
/*          编号 编码器{count,  零位}                 In电流限制       1 / 力矩常数      限位                                                  反向*/\
motor[0] = { 1,  {81 * 65536,   0},  6.1,     1 / (0.118 * 81), {-DEG2RAD(90), DEG2RAD(90), 10 * SecondHandVelocity} };\
motor[1] = { 2,  {81 * 65536,   0},              6.1,     1 / (0.118 * 81), {-DEG2RAD(90), DEG2RAD(90), 10 * SecondHandVelocity} };

#endif // !MOTORS_H
