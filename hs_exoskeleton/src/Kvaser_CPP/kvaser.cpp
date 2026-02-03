#include <iostream>
#include <math.h>
#include "kvaser.h"
#include <cmath>
using namespace std;


/********************刘洋修改于2022年4月*************************/


void usleep(int time)
{

}
/*
 * 初始化 can 通道和 can 总线
 */
int Kvaser::canInit(int channel_number)//ok
{
    // 初始化 can 驱动（重复调用只会调用一次）
    canInitializeLibrary();
    // 打开 can 通道
    handle = canOpenChannel(channel_number, canOPEN_ACCEPT_VIRTUAL);
    if (handle < 0)
    {
        cout << "can Open Channel error!" << endl;
        return EXIT_FAILURE;
    }
    // 若发送消息时出现 TimeOut 错误，调整波特率（一般是 canBITRATE_500K）
    status = canSetBusParams(handle, canBITRATE_1M, 0, 0, 0, 0, 0);
    if (checkStatus("canSetBusParams"))
    {
        cout << "can Set Bus Params error!" << endl;
        return EXIT_FAILURE;
    }
    // 打开 can 总线
    status = canBusOn(handle);
    if (checkStatus("canBusOn"))
    {
        cout << "can Bus On error!" << endl;
        return EXIT_FAILURE;
    }
    cout << "can initialized!" << endl;
    return EXIT_SUCCESS;
}

int Kvaser::canInit(int channel_number, int canbitRate){
    // 初始化 can 驱动（重复调用只会调用一次）
    canInitializeLibrary();
    // 打开 can 通道
    handle = canOpenChannel(channel_number, canOPEN_ACCEPT_VIRTUAL);
    if (handle < 0)
    {
        cout << "can Open Channel error!" << endl;
        return EXIT_FAILURE;
    }
    // 若发送消息时出现 TimeOut 错误，调整波特率（一般是 canBITRATE_500K）
    status = canSetBusParams(handle, canbitRate, 0, 0, 0, 0, 0);
    if (checkStatus("canSetBusParams"))
    {
        cout << "can Set Bus Params error!" << endl;
        return EXIT_FAILURE;
    }
    // 打开 can 总线
    status = canBusOn(handle);
    if (checkStatus("canBusOn"))
    {
        cout << "can Bus On error!" << endl;
        return EXIT_FAILURE;
    }
    cout << "can initialized!" << endl;
    return EXIT_SUCCESS;
}

/*
 * 关闭 can 通道
 */
int Kvaser::canRelease()//ok
{
    status = canBusOff(handle);
    if (checkStatus("canBusOff"))
    {
        return EXIT_FAILURE;
    }
    status = canClose(handle);
    if (checkStatus("canClose"))
    {
        return EXIT_FAILURE;
    }
    cout << "can released!" << endl;
    return EXIT_SUCCESS;
}

/*
 * 发送 can 消息
 */
int Kvaser::canSend(uint8_t *msg, long can_id, unsigned int dlc)//ok
{
    status = canWrite(handle, can_id, msg, dlc, 0);
    if (checkStatus("canWrite"))
    {
        return EXIT_FAILURE;
    }
//    status = canWriteSync(handle, 100);
//    if (checkStatus("canWriteSync"))
//    {
//        return EXIT_FAILURE;
//    }
    return EXIT_SUCCESS;
}

/*
 * 连接电机//废除//
 */
int Kvaser::connectMotor(long can_id)//ok
{
//    uint8_t data1[8] = {0};
//    //MOTOR SETTING
        return 0;
}

/*
 * 电机使能//废除//
 */
int Kvaser::motorEnable(long can_id)//不需要操作电机使能
{

    cout << "Motor " << can_id << " has been enabled!" << endl;
    return EXIT_SUCCESS;
}

/*
 * 关闭电机//废除//
 */
int Kvaser::motorDisable(long can_id)//不需要操作电机使能
{

    return EXIT_SUCCESS;
}

/*
 * 选择电机运动模式//废除//
 */
int Kvaser::modeChoose(long can_id, Mode mode)//
{
    return EXIT_SUCCESS;
}

int Kvaser::speedMode(int use, long can_id, float speed ,float current){
    uint8_t Data[8];
    int16_t cur_tor=(int16_t) (protect(current));

    rv_type_convert.to_float=speed;
    Data[0]=0x00|CMD_VELOCITY_CONTROL;
    Data[1]=rv_type_convert.buf[0];
    Data[2]=rv_type_convert.buf[1];
    Data[3]=rv_type_convert.buf[2];
    Data[4]=rv_type_convert.buf[3];
    Data[5]=cur_tor&0xff;  //取低八位
    Data[6]=cur_tor>>8; //取高八位

    int flag=canSend(Data, can_id, 7);
    if (flag == EXIT_FAILURE)
    {
        cout << "Motor " << can_id << " start Speed mode failed!" << endl;
        return EXIT_FAILURE;
    }
    //    cout << "Motor " << can_id << " start speed mode!" << endl;
    return EXIT_SUCCESS;
}

/*
 * 速度模式
 */
/*
motor_id:1~0x7FE 20250314与自研电机通信一致
spd:   RPM
cur:0~65535 0.1A
ack_status:0~3
*/
int Kvaser::speedMode(long can_id, float speed ,float current)
{
    uint8_t Data[8];
    int16_t cur_tor=(int16_t) (protect(current)*100);

    rv_type_convert.to_float=speed;
    Data[0]=0x00|CMD_VELOCITY_CONTROL;
    Data[1]=rv_type_convert.buf[0];
    Data[2]=rv_type_convert.buf[1];
    Data[3]=rv_type_convert.buf[2];
    Data[4]=rv_type_convert.buf[3];
    Data[5]=cur_tor&0xff;  //取低八位
    Data[6]=cur_tor>>8; //取高八位

    int flag=canSend(Data, can_id, 7);
    if (flag == EXIT_FAILURE)
    {
        cout << "Motor " << can_id << " start Speed mode failed!" << endl;
        return EXIT_FAILURE;
    }
//    cout << "Motor " << can_id << " start speed mode!" << endl;
    return EXIT_SUCCESS;
}
/*
 * 速度模式重载函数，该函数实现电机状态等参数自动返回。
 * can_id           待控制电机ID
 * speed            电机指令目标速度
 * current          当前指令电流限制
 * torque_real      当前电机实际电流
 * speed_real       当前电机实际速度
 * position_real    当前电机实际位置
 * motorEn          电机使能状态
 * errflag          电机错误状态
*/
int Kvaser::speedMode(long can_id, float speed ,float current,float* torque_real,float* speed_real,float* position_real,int* motorEn,int* errflag)
{
    uint8_t Data[8];
    int16_t cur_tor=(int16_t) (protect(current)*100);

    rv_type_convert.to_float=speed;
    Data[0]=0x00|CMD_VELOCITY_CONTROL|0x20;
    Data[1]=rv_type_convert.buf[0];
    Data[2]=rv_type_convert.buf[1];
    Data[3]=rv_type_convert.buf[2];
    Data[4]=rv_type_convert.buf[3];
    Data[5]=cur_tor&0xff;
    Data[6]=cur_tor>>8;

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(Data, Get_data, can_id, 7, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        if(temp_rev_dlc==8 && (Get_data[0]&0x1F)==CMD_VELOCITY_CONTROL && (Get_data[0]&0x40)==0x40)
        {
            *errflag=(Get_data[0]&0x80)>0?1:0;
            *motorEn=(Get_data[0]&0x20)>0?1:0;
            *torque_real=0.01*(int16_t)(Get_data[1]|Get_data[2]<<8);
            *speed_real=0.1*(int16_t)(Get_data[3]|Get_data[4]<<8);
            *position_real=(float)((int32_t)(Get_data[5]<<8|Get_data[6]<<16|Get_data[7]<<24))/256/17.453;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
     *hpb20250320添加，与positionMode其他不同的是，速度、电流限制提取写在了操作杆类
     */
int Kvaser::positionMode(int use, long can_id, float position, float speed, float current){
    uint8_t Data[8];
    uint16_t cur_tor=(uint16_t) (protect(current));
    uint16_t spd=(uint16_t)(fabs(speed));
    rv_type_convert.to_float=position;

    Data[0]=0x00|CMD_POSITION_CONTROL;
    Data[1]=rv_type_convert.buf[0];
    Data[2]=rv_type_convert.buf[1];
    Data[3]=rv_type_convert.buf[2];
    Data[4]=rv_type_convert.buf[3];
    Data[5]=spd&0xFF;
    Data[6]=(spd&0xFF00)>>8;
    Data[7]=cur_tor&0xFF;

    int flag=canSend(Data, can_id, 8);
    if (flag == EXIT_FAILURE)
    {
        cout << "Motor " << can_id << " startSensor position mode failed!" << endl;
        return EXIT_FAILURE;
    }
    //    cout << "Motor " << can_id << " startSensor position mode!" << endl;
    return EXIT_SUCCESS;
}

/*
 * 位置模式 20250314与自研电机通信一致
 */
/*
motor_id:1~0x7FE
pos:float     DEGREE
spd:0~32767   0.1RPM
torque:0~4095 0.1A
ack_status:0~3
*/
int Kvaser::positionMode(long can_id, float position, float speed, float current)
{
    uint8_t Data[8];
    uint16_t cur_tor=(uint16_t) (protect(current)*2.5);
    uint16_t spd=(uint16_t)(fabs(speed)*10);
    rv_type_convert.to_float=position;

    Data[0]=0x00|CMD_POSITION_CONTROL;
    Data[1]=rv_type_convert.buf[0];
    Data[2]=rv_type_convert.buf[1];
    Data[3]=rv_type_convert.buf[2];
    Data[4]=rv_type_convert.buf[3];
    Data[5]=spd&0xFF;
    Data[6]=(spd&0xFF00)>>8;
    Data[7]=cur_tor&0xFF;

    int flag=canSend(Data, can_id, 8);
    if (flag == EXIT_FAILURE)
    {
        cout << "Motor " << can_id << " startSensor position mode failed!" << endl;
        return EXIT_FAILURE;
    }
//    cout << "Motor " << can_id << " startSensor position mode!" << endl;
    return EXIT_SUCCESS;
}

/*
 * 位置模式重载函数，该函数实现电机状态等参数自动返回。
 * can_id           待控制电机ID
 * position         电机指令目标位置
 * speed            当前指令速度限制
 * current          当前指令电流限制
 * torque_real      当前电机实际电流
 * speed_real       当前电机实际速度
 * position_real    当前电机实际位置
 * motorEn          电机使能状态
 * errflag          电机错误状态
*/
int Kvaser::positionMode(long can_id, float position, float speed, float current,float* torque_real,float* speed_real,float* position_real,int* motorEn,int* errflag)
{
    uint8_t Data[8];
    uint16_t cur_tor=(uint16_t) (protect(current)*2.5);
    uint16_t spd=(uint16_t)(fabs(speed)*10);
    rv_type_convert.to_float=position;

    Data[0]=0x00|CMD_POSITION_CONTROL|0x20;
    Data[1]=rv_type_convert.buf[0];
    Data[2]=rv_type_convert.buf[1];
    Data[3]=rv_type_convert.buf[2];
    Data[4]=rv_type_convert.buf[3];
    Data[5]=spd&0xFF;
    Data[6]=(spd&0xFF00)>>8;
    Data[7]=cur_tor&0xFF;

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(Data, Get_data, can_id, 8, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        if(temp_rev_dlc==8 && (Get_data[0]&0x1F)==CMD_POSITION_CONTROL && (Get_data[0]&0x40)==0x40)
        {
            *errflag=(Get_data[0]&0x80)>0?1:0;
            *motorEn=(Get_data[0]&0x20)>0?1:0;
            *torque_real=0.01*(int16_t)(Get_data[1]|Get_data[2]<<8);
            *speed_real=0.1*(int16_t)(Get_data[3]|Get_data[4]<<8);
            *position_real=(float)((int32_t)(Get_data[5]<<8|Get_data[6]<<16|Get_data[7]<<24))/256/17.453;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
 * 力位混合模式
 */
/*
motor_id:1~0x7FE
position:     DEGREE
speed:          RPM
fowardCurrent: A

*/
int Kvaser::FPMixMode(long can_id, float kp,float kd,float position, float speed, float fowardCurrent)
{
    uint8_t Data[8];
    float fCurrent=fowardCurrent>90?90:fowardCurrent<-90?-90:fowardCurrent;
    uint16_t fcur=(uint16_t) 1024*(fCurrent+90.0)/180;
    float posReg=(PI*position/180+12.5)<0?0:(PI*position/180+12.5)>25?25:(PI*position/180+12.5);
    float spdReg=(speed*2*PI/60+18)<0?0:(speed*2*PI/60+18)>25?25:(speed*2*PI/60+18);
    uint16_t spd=(uint16_t)(spdReg*4096/36);
    uint16_t pos=(uint16_t)(65536*posReg/25.0);
    float fkp=kp<0?0:kp>500?500:kp;
    float fkd=kd<0?0:kd>5.0?5.0:kd;
    uint16_t kpReg=(uint16_t)(1024*fkp/500.0);
    uint16_t kdReg=(uint16_t)(256*fkd/5.0);


    Data[0]=0x00|CMD_FPMIX_CONTROL;
    Data[1]=(kpReg>>2)&0xFF;
    Data[2]=((kdReg>>2)&0x3F)|((kpReg<<6)&0xC0);
    Data[3]=((pos>>10)&0x3F)|((kdReg<<6)&0xC0);
    Data[4]=((pos>>2)&0xFF);
    Data[5]=((spd>>6)&0x3F)|((pos<<6)&0xC0);
    Data[6]=(fcur>>8&0x03)|((spd<<2)&0xFC);
    Data[7]=fcur&0xFF;

    int flag=canSend(Data, can_id, 8);
    if (flag == EXIT_FAILURE)
    {
        cout << "Motor " << can_id << " startSensor position mode failed!" << endl;
        return EXIT_FAILURE;
    }
    //    cout << "Motor " << can_id << " startSensor position mode!" << endl;
    return EXIT_SUCCESS;
}

/*
 * 力位混合模式重载函数，该函数实现电机状态等参数自动返回。
 * can_id           待控制电机ID
 * kp/kd            力位混合参数
 * position         电机指令目标位置
 * speed            当前指令速度
 * fowardCurrent    当前指令前馈
 * torque_real      当前电机实际电流
 * speed_real       当前电机实际速度
 * position_real    当前电机实际位置
 * motorEn          电机使能状态
 * errflag          电机错误状态
*/
int Kvaser::FPMixMode(long can_id, float kp,float kd,float position, float speed, float fowardCurrent,float* torque_real,float* speed_real,float* position_real,int* motorEn,int* errflag)
{
    uint8_t Data[8];
    float fCurrent=fowardCurrent>90?90:fowardCurrent<-90?-90:fowardCurrent;
    uint16_t fcur=(uint16_t) 1024*(fCurrent+90.0)/180;
    float posReg=(PI*position/180+12.5)<0?0:(PI*position/180+12.5)>25?25:(PI*position/180+12.5);
    float spdReg=(speed*2*PI/60+18)<0?0:(speed*2*PI/60+18)>25?25:(speed*2*PI/60+18);
    uint16_t spd=(uint16_t)(spdReg*4096/36);
    uint16_t pos=(uint16_t)(65536*posReg/25.0);
    float fkp=kp<0?0:kp>500?500:kp;
    float fkd=kd<0?0:kd>5.0?5.0:kd;
    uint16_t kpReg=(uint16_t)(1024*fkp/500.0);
    uint16_t kdReg=(uint16_t)(256*fkd/5.0);


    Data[0]=0x00|CMD_FPMIX_CONTROL|0x20;
    Data[1]=(kpReg>>2)&0xFF;
    Data[2]=((kdReg>>2)&0x3F)|((kpReg<<6)&0xC0);
    Data[3]=((pos>>10)&0x3F)|((kdReg<<6)&0xC0);
    Data[4]=((pos>>2)&0xFF);
    Data[5]=((spd>>6)&0x3F)|((pos<<6)&0xC0);
    Data[6]=(fcur>>8&0x03)|((spd<<2)&0xFC);
    Data[7]=fcur&0xFF;

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(Data, Get_data, can_id, 8, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        if(temp_rev_dlc==8 && (Get_data[0]&0x1F)==CMD_FPMIX_CONTROL && (Get_data[0]&0x40)==0x40)
        {
            *errflag=(Get_data[0]&0x80)>0?1:0;
            *motorEn=(Get_data[0]&0x20)>0?1:0;
            *torque_real=0.01*(int16_t)(Get_data[1]|Get_data[2]<<8);
            *speed_real=0.1*(int16_t)(Get_data[3]|Get_data[4]<<8);
            *position_real=(float)((int32_t)(Get_data[5]<<8|Get_data[6]<<16|Get_data[7]<<24))/256/17.453;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
 * 力矩（电流）模式，20250314与自研电机通信一致
 */
int Kvaser::torqueMode(int use, long can_id, float torque)//ok
{
    uint8_t data[8] = {0}; // TC;

    int16_t cur_tor=(int16_t) (protect(torque));

    data[0]=0x00|CMD_CURRENT_CONTROL;
    data[1]=cur_tor&0xFF;
    data[2]=(cur_tor>>8)&0xFF;

    int flag = canSend(data, can_id, 3);
    if (flag == EXIT_FAILURE)
    {
        cout << "Motor " << can_id << " startSensor torque failed!" << endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/*
 * 力矩（电流）模式，20250314与自研电机通信一致
 */
int Kvaser::torqueMode(long can_id, float torque)//ok
{
    uint8_t data[8] = {0}; // TC;

    int16_t cur_tor=(int16_t) (protect(torque)*100);

    data[0]=0x00|CMD_CURRENT_CONTROL;
    data[1]=cur_tor&0xFF;
    data[2]=(cur_tor>>8)&0xFF;

    int flag = canSend(data, can_id, 3);
    if (flag == EXIT_FAILURE)
    {
        cout << "Motor " << can_id << " startSensor torque failed!" << endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/*
 * 电流模式重载函数，该函数实现电机状态等参数自动返回。
 * can_id           待控制电机ID
 * torque          当前指令电流
 * torque_real      当前电机实际电流
 * speed_real       当前电机实际速度
 * position_real    当前电机实际位置
 * motorEn          电机使能状态
 * errflag          电机错误状态
*/

//自动返回的数据解析
int Kvaser::torqueMode(long can_id, float torque,float* torque_real,float* speed_real,float* position_real,int* motorEn,int* errflag)
{
    uint8_t data[8] = {0}; // TC;

    int16_t cur_tor=(int16_t) (protect(torque)*100);

    data[0]=0x00|CMD_CURRENT_CONTROL|0x20;
    data[1]=cur_tor&0xFF;
    data[2]=(cur_tor>>8)&0xFF;

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(data, Get_data, can_id, 3, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        if(temp_rev_dlc==8 && (Get_data[0]&0x1F)==CMD_CURRENT_CONTROL && (Get_data[0]&0x40)==0x40)
        {
            *errflag=(Get_data[0]&0x80)>0?1:0;
            *motorEn=(Get_data[0]&0x20)>0?1:0;
            *torque_real=0.01*(int16_t)(Get_data[1]|Get_data[2]<<8);
            *speed_real=0.1*(int16_t)(Get_data[3]|Get_data[4]<<8);
            *position_real=(float)((int32_t)(Get_data[5]<<8|Get_data[6]<<16|Get_data[7]<<24))/256/17.453;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
 * 电机复位//废除//
 */
int Kvaser::motorReset(long can_id)
{
//    if (motorDisable(can_id) == EXIT_FAILURE)
//    {
//        return EXIT_FAILURE;
//    }
//    else if (modeChoose(can_id, SINGLE_FEEDBACK_POSITION_MODE) == EXIT_FAILURE)
//    {
//        return EXIT_FAILURE;
//    }
//    else if (motorEnable(can_id) == EXIT_FAILURE)
//    {
//        return EXIT_FAILURE;
//    }
//    positionMode(can_id, 0, 60000);
//    usleep(1000 * 1000 * 5); // 需要睡 5s，否则无法复位
    return EXIT_SUCCESS;
}

/*
 * 电流保护
 */
float Kvaser::protect(float torque)
{
    float temp = torque;
    if (torque > HIGH_LIMIT)
    {
        temp = HIGH_LIMIT;
    }
    else if (torque < LOW_LIMIT)
    {
        temp = LOW_LIMIT;
    }
    return temp;
}

/*
 * 开始转动//废除//
 */
int Kvaser::beginMovement(long can_id)//无需运行开始指令
{
    return EXIT_SUCCESS;
}

/*
 * 获取真实位置//废除//
 */
float Kvaser::getPosition(long can_id,uint8_t *motor_state)
{
    uint8_t data[8] = {0};
    uint8_t mes_type=0;
    uint8_t mes_code=0;

    data[0]=0xe0;

    /*0：保留，目前无效
    1：查询当前位置
    2：查询当前速度
    3：查询当前电流
    4：查询当前功率
    5：查询当前加速度
    6：查询当前磁链观测增益
    7：查询当前扰动补偿系数
    8：查询反馈补偿系数
    9：查询阻尼系数*/
    data[1]=0x01;

//    rv_type_convert.to_float=position;
    int reg=0;
    uint8_t Get_data[6];
    uint32_t temp_rev_dlc;
     RV_TypeConvert data_conv;
    canFlushReceiveQueue(handle);
    do{
    reg=getInfAndRead(data, Get_data, can_id, 2, &temp_rev_dlc);
    mes_type=(Get_data[0]&0xE0)>>5;
    *motor_state=Get_data[0]&0x1F;
    mes_code=Get_data[1];
    }while((mes_type!=5 || mes_code!=1) && reg==EXIT_SUCCESS);
    if(reg==EXIT_FAILURE)
    {
        *motor_state |=0x80;
    }
    else
    {
        *motor_state &=0x1F;
    }
    if(mes_type==5)
    {
        if(mes_code==1)
        {
            data_conv.buf[0]=Get_data[5];
            data_conv.buf[1]=Get_data[4];
            data_conv.buf[2]=Get_data[3];
            data_conv.buf[3]=Get_data[2];
        }
    }

    return data_conv.to_float;
}

/*
 * 获取真实速度//废除//
 */
float Kvaser::getVelocity(long can_id,uint8_t *motor_state)
{
    uint8_t data[8] = {0};
    uint8_t mes_type=0;
    uint8_t mes_code=0;

    data[0]=0xe0;

    /*0：保留，目前无效
    1：查询当前位置
    2：查询当前速度
    3：查询当前电流
    4：查询当前功率
    5：查询当前加速度
    6：查询当前磁链观测增益
    7：查询当前扰动补偿系数
    8：查询反馈补偿系数
    9：查询阻尼系数*/
    data[1]=0x02;

//    rv_type_convert.to_float=position;
    int reg=0;
    uint8_t Get_data[6];
    uint32_t temp_rev_dlc;
     RV_TypeConvert data_conv;
    canFlushReceiveQueue(handle);
    do{
    reg=getInfAndRead(data, Get_data, can_id, 2, &temp_rev_dlc);
    mes_type=(Get_data[0]&0xE0)>>5;
    *motor_state=Get_data[0]&0x1F;
    mes_code=Get_data[1];
    }while((mes_type!=5 || mes_code!=2) && reg==EXIT_SUCCESS);
    if(reg==EXIT_FAILURE)
    {
        *motor_state |=0x80;
    }
    else
    {
        *motor_state &=0x1F;
    }
    if(mes_type==5)
    {
        if(mes_code==2)
        {
            data_conv.buf[0]=Get_data[5];
            data_conv.buf[1]=Get_data[4];
            data_conv.buf[2]=Get_data[3];
            data_conv.buf[3]=Get_data[2];
        }
    }

    return data_conv.to_float;
}

/*
 * 获取真实电流//废除//
 */
float Kvaser::getCurrent(long can_id,uint8_t *motor_state)
{
    uint8_t data[8] = {0};
    uint8_t mes_type=0;
    uint8_t mes_code=0;

    data[0]=0xe0;

    /*0：保留，目前无效
    1：查询当前位置
    2：查询当前速度
    3：查询当前电流
    4：查询当前功率
    5：查询当前加速度
    6：查询当前磁链观测增益
    7：查询当前扰动补偿系数
    8：查询反馈补偿系数
    9：查询阻尼系数*/
    data[1]=0x03;

//    rv_type_convert.to_float=position;
    int reg=0;
    uint8_t Get_data[6];
    uint32_t temp_rev_dlc;
    RV_TypeConvert data_conv;
    canFlushReceiveQueue(handle);
    do{
    reg=getInfAndRead(data, Get_data, can_id, 2, &temp_rev_dlc);
    mes_type=(Get_data[0]&0xE0)>>5;
    *motor_state=Get_data[0]&0x1F;
    mes_code=Get_data[1];
    }while((mes_type!=5 || mes_code!=3) && reg==EXIT_SUCCESS);
    if(reg==EXIT_FAILURE)
    {
        *motor_state |=0x80;
    }
    else
    {
        *motor_state &=0x1F;
    }
    if(mes_type==5)
    {
        if(mes_code==3)
        {
            data_conv.buf[0]=Get_data[5];
            data_conv.buf[1]=Get_data[4];
            data_conv.buf[2]=Get_data[3];
            data_conv.buf[3]=Get_data[2];
        }
    }

    return data_conv.to_float;
}

/*
 * 获取真实电机功率//废除//
 */
float Kvaser::getPower(long can_id,uint8_t *motor_state)
{
    uint8_t data[8] = {0};
    uint8_t mes_type=0;
    uint8_t mes_code=0;

    data[0]=0xe0;

    /*0：保留，目前无效
    1：查询当前位置
    2：查询当前速度
    3：查询当前电流
    4：查询当前功率
    5：查询当前加速度
    6：查询当前磁链观测增益
    7：查询当前扰动补偿系数
    8：查询反馈补偿系数
    9：查询阻尼系数*/
    data[1]=0x04;

//    rv_type_convert.to_float=position;
    int reg=0;

    uint8_t Get_data[6];
    uint32_t temp_rev_dlc;
    RV_TypeConvert data_conv;
    canFlushReceiveQueue(handle);
    do{
    reg=getInfAndRead(data, Get_data, can_id, 2, &temp_rev_dlc);
    mes_type=(Get_data[0]&0xE0)>>5;
    *motor_state=Get_data[0]&0x1F;
    mes_code=Get_data[1];
    }while((mes_type!=5 || mes_code!=4) && reg==EXIT_SUCCESS);

    if(reg==EXIT_FAILURE)
    {
        *motor_state |=0x80;
    }
    else
    {
        *motor_state &=0x1F;
    }

    if(mes_type==5)
    {
        if(mes_code==4)
        {
            data_conv.buf[0]=Get_data[5];
            data_conv.buf[1]=Get_data[4];
            data_conv.buf[2]=Get_data[3];
            data_conv.buf[3]=Get_data[2];
        }
    }

    return data_conv.to_float;
}


//MOTOR SETTING 电机电机通信模式设置指令//废除//
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.//默认选2
0x03:set the current position to zero.
*/
int Kvaser::MotorSetting(long can_id,uint8_t cmd)
{
    if(cmd==0) return EXIT_FAILURE;

    uint8_t Get_data[8]={0};
    uint8_t Data[8]={0};
    uint32_t temp_rev_dlc;

    Data[0]=can_id>>8;
    Data[1]=can_id&0xff;
    Data[2]=0x00;
    Data[3]=cmd;

    canFlushReceiveQueue(handle);//清除CAN缓存
    int reg=getInfAndRead(Data, Get_data, 0x7FF, 4, &temp_rev_dlc);
    if(reg==EXIT_SUCCESS)
    {
        if(temp_rev_dlc==4)
        {
            if(Get_data[3]==0x00)
            {
                return EXIT_FAILURE;
            }
            else
            {
                return EXIT_SUCCESS;
            }
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    else
    {
        return EXIT_FAILURE;
    }
}


//电机零点设置，设置当前位置为电机零点//废除//
int Kvaser::MotorZeroSet(long can_id)
{
    uint8_t Get_data[8]={0};
    uint8_t Data[8]={0};
    uint32_t temp_rev_dlc;

    Data[0]=can_id>>8;
    Data[1]=can_id&0xff;
    Data[2]=0x00;
    Data[3]=0x03;

    canFlushReceiveQueue(handle);//清除CAN缓存
    int reg=getInfAndRead(Data, Get_data, 0x7FF, 4, &temp_rev_dlc);
    if(reg==EXIT_SUCCESS)
    {
        if(temp_rev_dlc==4)
        {
            if(Get_data[3]==0x03 && Get_data[2]==0x01)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    else
    {
        return EXIT_FAILURE;
    }
}

//MOTOR电机CAN 通信ID 设置指令

int Kvaser::MotorIDSet(long can_id,long new_id)
{

    uint8_t Get_data[8]={0};
    uint8_t Data[8]={0};
    uint32_t temp_rev_dlc;

    Data[0]=can_id>>8;
    Data[1]=can_id&0xff;
    Data[2]=0x00;
    Data[3]=0x04;
    Data[4]=new_id>>8;
    Data[5]=new_id&0xff;

    uint8_t send_dlc=6;

    canFlushReceiveQueue(handle);//清除CAN缓存
    int reg=getInfAndRead(Data, Get_data, 0x7FF,send_dlc, &temp_rev_dlc);
    if(reg==EXIT_SUCCESS)
    {
        if(temp_rev_dlc==4)
        {
            if(Get_data[3]==0x04 && Get_data[2]==0x01)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    else
    {
        return EXIT_FAILURE;
    }
}

//MOTOR电机CAN 通信ID 重置指令，重置后默认为1

int Kvaser::MotorIDReset()
{

    uint8_t Get_data[8]={0};
    uint8_t Data[8]={0};
    uint32_t temp_rev_dlc;

    Data[0]=0x7F;
    Data[1]=0x7F;
    Data[2]=0x00;
    Data[3]=0x05;
    Data[4]=0x7F;
    Data[5]=0x7F;

    uint8_t send_dlc=6;

    canFlushReceiveQueue(handle);//清除CAN缓存
    int reg=getInfAndRead(Data, Get_data, 0x7FF,send_dlc, &temp_rev_dlc);
    if(reg==EXIT_SUCCESS)
    {
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}

//查询电机通信模式//废除//
//uint8_t *ComMode 为返回的模式值 0x01：自动报文模式   0x02：问答模式   0x80：查询失败
int Kvaser::MotorCommModeReading(long can_id,uint8_t *ComMode)
{

    uint8_t Get_data[8]={0};
    uint8_t Data[8]={0};
    uint32_t temp_rev_dlc;

    Data[0]=can_id>>8;
    Data[1]=can_id&0xff;
    Data[2]=0x00;
    Data[3]=0x81;
    uint8_t send_dlc=4;

    canFlushReceiveQueue(handle);//清除CAN缓存
    int reg=getInfAndRead(Data, Get_data, 0x7FF,send_dlc, &temp_rev_dlc);
    if(reg==EXIT_SUCCESS)
    {
        if(temp_rev_dlc==4)
        {
            if(Get_data[3]!=0x80 && Get_data[2]==0x01)
            {
                *ComMode=Get_data[3];
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    else
    {
        return EXIT_FAILURE;
    }
}

//查询CAN 通信ID
int Kvaser::MotorIDReading(long *id_get)//can 协议
{
    uint8_t Get_data[8]={0};
    uint8_t Data[8]={0};
    uint32_t temp_rev_dlc;
    uint32_t send_id=*id_get;
    Data[0]=0x64;

    uint8_t send_dlc=1;

    canFlushReceiveQueue(handle);//清除CAN缓存
    int reg=getInfAndRead(Data, Get_data, send_id,send_dlc, &temp_rev_dlc);
    if(reg==EXIT_SUCCESS)
    {
        if(temp_rev_dlc==5)
        {
            if(Get_data[0]==0x64)
            {
                return EXIT_SUCCESS;
            }else
            { return EXIT_FAILURE;
            }
        } else
        {return EXIT_FAILURE;
        }
    }else
    {return EXIT_FAILURE;
    }
}

// 查询钛虎电机的CSP
//int Kvaser::MotorCSPReading(long id_get, int16_t *C_CSP, int16_t *S_CSP, int32_t *P_CSP) // can 协议
//{
//    uint8_t Get_data[8] = {0};
//    uint8_t Data[8] = {0};
//    uint32_t temp_rev_dlc;
//    uint32_t send_id = id_get;
//    Data[0] = 0x41; // 补充分号
//
//    uint8_t send_dlc = 1;
//
//    canFlushReceiveQueue(handle); // 清除CAN缓存
//    int reg = getInfAndRead(Data, Get_data, send_id, send_dlc, &temp_rev_dlc);
//    if (reg == EXIT_SUCCESS)
//    {
//        if (temp_rev_dlc == 8)
//        {
//            *C_CSP = Get_data[0] + (Get_data[1] << 8);
//            *S_CSP = Get_data[2] + (Get_data[3] << 8);
//            uint32_t unsigned_P_CSP = Get_data[4] + (Get_data[5] << 8) + (Get_data[6] << 16) + (Get_data[7] << 24);
//            *P_CSP = static_cast<int32_t>(unsigned_P_CSP);
//            return EXIT_SUCCESS; // 返回成功标识
//        }
//        else
//        {
//            return EXIT_FAILURE;
//        }
//    }
//    else
//    {
//        return EXIT_FAILURE;
//    }
//}

/*
 * 读取 can 通道返回数据
 */
int Kvaser::getInfAndRead(uint8_t *send_msg, uint8_t *read_msg, long can_id, unsigned int send_dlc,unsigned int *read_dlc)
{
    unsigned long tempTime;
    unsigned int flag;
    canWrite(handle, can_id, send_msg, send_dlc, 0);//

    int reg=canReadWait(handle, &can_id, read_msg, read_dlc, &flag, &tempTime,300);//等待250ms数据
    if(reg==canOK)
    {
       return EXIT_SUCCESS;
    }
    else
    {
       return EXIT_FAILURE;
    }

}

/*
 * 检查 can 通道状态
 */
int Kvaser::checkStatus(const string &id)
{
    if (status != canOK)
    {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(status, buf, sizeof(buf));
        cout << id << ": failed, stat = " << (int) status << " info: " << buf << endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/*
 * 数据格式转换
 */
void Kvaser::dataIntegrationInt(uint8_t *temp, int32_t temp_value)
{
    temp[7] = (temp_value >> 24);
    temp[6] = ((temp_value << 8) >> 24);
    temp[5] = ((temp_value << 16) >> 24);
    temp[4] = ((temp_value << 24) >> 24);
}

/*
 * 数据格式转换
 */
void Kvaser::dataIntegrationFloat(uint8_t *temp, float temp_value)
{
    int data = *(int *) &temp_value;
    temp[7] = (data >> 24);
    temp[6] = ((data << 8) >> 24);
    temp[5] = ((data << 16) >> 24);
    temp[4] = ((data << 24) >> 24);
    temp[3] |= 1 << 7;
}


/*
 * 数据格式转换
 */
void Kvaser::dataFloatIntegration(uint8_t *temp, float *temp_value)
{
    uint32_t data;
    data = ((temp[7] & 0xff) << 24) | ((temp[6] & 0xff) << 16) | ((temp[5] & 0xff) << 8) | (temp[4] & 0xff);
    *temp_value = *(float *) &data;
}

/*
 * 获取当前电机固件版本号
 */
int Kvaser::MotorGetVersion(long can_id,uint32_t *Version)
{
    uint8_t data[8] = {0};

    data[0]=0x00|CMD_GET_VERSION;
    data[1]=CMD_DATA_OK;

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(data, Get_data, can_id, 2, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        if(temp_rev_dlc==6 && Get_data[1]==CMD_DATA_OK && Get_data[0]==(0x40|CMD_GET_VERSION))
        {
            *Version=(uint32_t)Get_data[2]|Get_data[3]<<8|Get_data[4]<<16|Get_data[5]<<24;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
 * 获取对应寄存器值
 * RegNumb  寄存器地址
 * DataGet  获取返回值
 */
int Kvaser::MotorGetReg(long can_id,uint16_t RegNumb,uint32_t *DataGet)
{
    uint8_t data[8] = {0};

    data[0]=0x00|CMD_GET_REG;
    data[1]=RegNumb&0xFF;
    data[2]=(RegNumb>>8)&0xFF;

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(data, Get_data, can_id, 3, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        uint16_t cmd_temp=Get_data[1]|(Get_data[2]<<8);
        uint8_t cmd_type=(cmd_temp>>13)&0x07;
        if(temp_rev_dlc>=4 && Get_data[0]==(0x40|CMD_GET_REG) && cmd_temp==RegNumb)
        {
            switch(cmd_type)
            {
            case DATA_8BIT:
                *DataGet=(uint32_t)(0xFFFFFFFF&Get_data[3]);
                break;
            case DATA_16BIT:
                *DataGet=(uint32_t)Get_data[3]|Get_data[4]<<8;
                break;
            case DATA_32BIT:
                *DataGet=(uint32_t)Get_data[3]|Get_data[4]<<8|Get_data[5]<<16|Get_data[6]<<24;
                break;
            default:
                break;
            }
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
 * 寄存器设置
 * 设置后仅改变当前寄存器数值，如果需要写入flash还需要MotorSetReg(canid,MOTOR_REG_WRITE_TO_FLASH,(uint32_t *)&RegData)操作
 */
int Kvaser::MotorSetReg(long can_id,uint16_t RegNumb,uint32_t *DataSet)
{
    uint8_t data[8] = {0};
    uint32_t dlc_send=0;

    data[0]=0x00|CMD_SET_REG;
    data[1]=RegNumb&0xFF;
    data[2]=(RegNumb>>8)&0xFF;
    data[3]=(*DataSet)&0xFF;
    data[4]=((*DataSet)>>8)&0xFF;
    data[5]=((*DataSet)>>16)&0xFF;
    data[6]=((*DataSet)>>24)&0xFF;

    switch((RegNumb>>13)&0x07)
    {
    case DATA_8BIT:
        dlc_send=4;
        break;
    case DATA_16BIT:
        dlc_send=5;
        break;
    case DATA_32BIT:
        dlc_send=7;
        break;
    default:
        break;
    }

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(data, Get_data, can_id, dlc_send, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        uint16_t cmd_temp=Get_data[1]|(Get_data[2]<<8);
        if(temp_rev_dlc==3 && Get_data[0]==(0x40|CMD_SET_REG) && cmd_temp==RegNumb)
        {

        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
 * 电机使能/去使能
 * CtlData  CMD_MOTOR_OFF/CMD_MOTOR_ON
 */
int Kvaser::MotorOnOff(long can_id,uint8_t CtlData)
{
    uint8_t data[8] = {0};
    uint32_t dlc_send=2;

    data[0]=0x00|CMD_MOTOR_ON_OFF;
    data[1]=CtlData;

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(data, Get_data, can_id, dlc_send, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        if(temp_rev_dlc==2 && Get_data[0]==(0x40|CMD_MOTOR_ON_OFF) && Get_data[1]==CtlData)
        {

        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
 * 获取电机错误码
 */
int Kvaser::MotorGetErrorCode(long can_id,uint16_t *FaultOccurred, uint16_t *FaultNow)
{
    uint8_t data[8] = {0};
    uint32_t dlc_send=2;
    uint32_t fault_temp;

    data[0]=0x00|CMD_GET_ERROR_CODE;
    data[1]=CMD_DATA_OK;

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(data, Get_data, can_id, dlc_send, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        if(temp_rev_dlc==5 && Get_data[0]==(0x40|CMD_GET_ERROR_CODE))
        {
            fault_temp=Get_data[1]|(Get_data[2]<<8)|(Get_data[3]<<16)|(Get_data[4]<<24);
            *FaultNow= (uint16_t) fault_temp;
            *FaultOccurred=(uint16_t) (fault_temp>>16);
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
 * 清除电机错误
 */
int Kvaser::MotorErrorClear(long can_id)
{
    uint8_t data[8] = {0};
    uint32_t dlc_send=2;

    data[0]=0x00|CMD_ERROR_CLEAR;
    data[1]=CMD_DATA_OK;

    int reg=0;
    uint8_t Get_data[8];
    uint32_t temp_rev_dlc;
    canFlushReceiveQueue(handle);
    reg=getInfAndRead(data, Get_data, can_id, dlc_send, &temp_rev_dlc);
    if(reg==EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    else
    {
        if(temp_rev_dlc==2 && Get_data[0]==(0x40|CMD_ERROR_CLEAR) && Get_data[1]==CMD_DATA_OK)
        {

        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

/*
 * 速度转动_并设置速度,钛虎电机
 */
int Kvaser::MotorSpeedRun(long can_id, int16_t SpeedRun)
{
    uint8_t canData[5] = {0};
    uint32_t dlc_send = 5;

    canFlushReceiveQueue(handle);
    canData[0] = 0x1D; // 首字节设置为0x1D

    // 不需要再区分正负，直接赋值并处理
    int16_t val16 = SpeedRun;
    canData[1] = val16 & 0xFF;       // 最低有效字节
    canData[2] = (val16 >> 8) & 0xFF; // 最高有效字节

    // 根据SpeedRun是否为负数来设置剩余字节
    if (SpeedRun < 0) {
        canData[3] = 0xFF;
        canData[4] = 0xFF;
    } else {
        // 如果SpeedRun是非负数，且协议要求这些字节为0x00，则保持它们为0x00
        // 这里已经是默认值了，因为canData数组已经初始化为0
    }

    canWrite(handle, can_id, canData, dlc_send, 0);
    return 0;
}

/*
 * 位置转动_并设置位置
 */
int Kvaser::MotorLocationRun(long can_id, int32_t LocationRun)
{
    uint8_t canData[5] = {0};
    uint32_t dlc_send = 5;

    canFlushReceiveQueue(handle);
    canData[0] = 0x1E; // 首字节设置为0x1E
    // 不需要再区分正负，直接赋值并处理
    int32_t val16 = LocationRun;
    canData[1] = val16 & 0xFF;       // 最低有效字节
    canData[2] = (val16 >> 8) & 0xFF; // 最高有效字节
    canData[3] = (val16 >> 16) & 0xFF; // 最高有效字节
    // 根据SpeedRun是否为负数来设置剩余字节
    if (LocationRun < 0) {
        canData[4] = 0xFF;
    } else {
        // 如果SpeedRun是非负数，且协议要求这些字节为0x00，则保持它们为0x00
        // 这里已经是默认值了，因为canData数组已经初始化为0
    }

    canWrite(handle, can_id, canData, dlc_send, 0);
    return 0;
}

//todo: 确定是否是零速停止
int Kvaser::selfDevMotorStop(long id){
    float speed = 0;
    float current= 0;
    int reg = speedMode(id, speed ,current);
    return reg;
}
/*
 * 电机停止
 * 钛虎单个电机停止代码
 * can_id  要输入id的电机
 */
int Kvaser::MotorStopID(long can_id)
{
    uint8_t data[8] = {0};
    uint32_t dlc_send=1;
    data[0] = 0x02; // 第一个字节固定为0x1D

    canFlushReceiveQueue(handle);

    canWrite(handle, can_id, data, dlc_send, 0);
    return 0;
}

/*
 * 位置加速度控制
 */
int Kvaser::Motorspeed_location(long can_id,int32_t LocationRun,int16_t SpeedRun)
{
    uint8_t canData[8] = {0};
    uint32_t dlc_send=7;
    canData[0] = 0x58; // 第一个字节固定为0x1D

    canFlushReceiveQueue(handle);

    int32_t val16 = LocationRun;
    canData[1] = val16 & 0xFF;       // 最低有效字节
    canData[2] = (val16 >> 8) & 0xFF; // 最高有效字节
    canData[3] = (val16 >> 16) & 0xFF; // 最高有效字节
    // 根据SpeedRun是否为负数来设置剩余字节
    if (LocationRun < 0) {
        canData[4] = 0xFF;
    } else {
        // 如果SpeedRun是非负数，且协议要求这些字节为0x00，则保持它们为0x00
        // 这里已经是默认值了，因为canData数组已经初始化为0
    }
    int32_t speed16 = SpeedRun;

    canData[5] = speed16 & 0xFF;       // 最低有效字节
    canData[6] = (speed16 >> 8) & 0xFF; // 最高有效字节

    canWrite(handle, can_id, canData, dlc_send, 0);

    return 0;
}


/*
 * 封装读取CSP
 */
// 封装读取电机 CSP 并显示的函数



