#ifndef ROS_KVASER_CAN_DRIVER_KVASER_CAN_H
#define ROS_KVASER_CAN_DRIVER_KVASER_CAN_H

#include "canlib.h"
#include "stdint.h"
//#include "motor.h"
#include <string>
using namespace std;


//主控制指令
#define CMD_GET_VERSION			0x00
#define CMD_GET_REG 			0x01
#define CMD_SET_REG 			0x02
#define CMD_MOTOR_ON_OFF        0x03		//电机使能开关
#define CMD_GET_ERROR_CODE		0x04
#define CMD_ERROR_CLEAR    		0x05
#define CMD_CURRENT_CONTROL		0x06
#define CMD_VELOCITY_CONTROL	0x07
#define CMD_POSITION_CONTROL	0x08
#define CMD_FPMIX_CONTROL		0x09		//力位混合

//电机使能、去使能
#define CMD_MOTOR_OFF			0x00
#define CMD_MOTOR_ON			0x01

//握手数据
#define CMD_DATA_OK				0x5A

//电机寄存器宏定义
#define DATA_8BIT    		1
#define DATA_16BIT   		2
#define DATA_32BIT   		3
#define DATA_BIT_POSTION   13

//8BIT
#define  MOTOR_REG_STATUS				(1|(DATA_8BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_CONTROL_MODE  	    (2|(DATA_8BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_WRITE_TO_FLASH       (3|(DATA_8BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_ZERO_SET				(4|(DATA_8BIT<<DATA_BIT_POSTION))
//16BIT
#define  MOTOR_REG_SPEED_KP  			(2|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KI  			(3|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KD  			(4|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_Q_KP  				(6|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_Q_KI  				(7|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_Q_KD  				(8|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_D_KP  				(10|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_D_KI  				(11|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_D_KD  				(12|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_C1  			(13|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_C2  			(14|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_C1  		(15|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_C2  		(16|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_KI  			(17|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_KP  			(18|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FLUXWK_KP  			(19|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FLUXWK_KI  			(20|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FLUXWK_BUS  			(21|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_BUS_VOLTAGE  		(22|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_HEATS_TEMP  			(23|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_MOTOR_POWER  		(24|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_DAC_OUT1  			(25|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_DAC_OUT2  			(26|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_DAC_OUT3  			(27|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FLUXWK_BUS_MEAS      (30|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_A  						(31|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_B  						(32|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_ALPHA_MEAS  				(33|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_BETA_MEAS  				(34|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_Q_MEAS  					(35|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_D_MEAS  					(36|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_Q_REF  					(37|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_D_REF  					(38|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_V_Q  						(39|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_V_D  						(40|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_V_ALPHA  					(41|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_V_BETA  						(42|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_ENCODER_EL_ANGLE  			(43|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_ENCODER_SPEED  				(44|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_EL_ANGLE  			(45|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_ROT_SPEED  			(46|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_I_ALPHA  				(47|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_I_BETA  				(48|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_BEMF_ALPHA  			(49|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_BEMF_BETA  			(50|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_EL_ANGLE  			(51|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_ROT_SPEED  		(52|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_I_ALPHA  			(53|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_I_BETA  			(54|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_BEMF_ALPHA  		(55|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_BEMF_BETA  		(56|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_DAC_USER1  					(57|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_DAC_USER2  					(58|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_HALL_EL_ANGLE 				(59|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_HALL_SPEED  					(60|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FF_VQ  						(62|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FF_VD  						(63|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FF_VQ_PIOUT  				(64|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FF_VD_PIOUT  				(65|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_DCBUS_REF  				(66|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_DCBUS_MEAS  				(67|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_ACBUS_FREQ  				(68|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_ACBUS_RMS  				(69|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_I_KP  					(70|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_I_KI    					(71|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_I_KD    					(72|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_V_KP   					(73|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_V_KI    					(74|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_V_KD    					(75|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_STARTUP_DURATION			(76|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_PWM_FREQUENCY  			(77|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KP  				(78|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KI  				(79|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KD  				(80|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KP_DIV  				(81|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KI_DIV  				(82|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KD_DIV  				(83|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_D_KP_DIV  				(84|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_D_KI_DIV  				(85|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_D_KD_DIV  				(86|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_Q_KP_DIV  				(87|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_Q_KI_DIV  				(88|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_I_Q_KD_DIV  				(89|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KP_DIV  		(90|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KI_DIV 			(91|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KD_DIV  		(92|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_I_KP_DIV  			(93|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_I_KI_DIV  			(94|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_I_KD_DIV  			(95|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_V_KP_DIV  			(96|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_V_KI_DIV  			(97|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_V_KD_DIV  			(98|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_KI_DIV  			(99|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_KP_DIV  			(100|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FLUXWK_KP_DIV  			(101|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FLUXWK_KI_DIV  			(102|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STARTUP_CURRENT_REF  	(105|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PULSE_VALUE  			(106|(DATA_16BIT<<DATA_BIT_POSTION))

#define  MOTOR_REG_POLEPAIR  				(300|(DATA_16BIT<<DATA_BIT_POSTION))

#define  MOTOR_REG_ALIGN_DURATION  	        (301|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_MAX_SPDRPM  				(302|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_GEAR_RATIO  			    (303|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_ROTATION_DIR  		    (304|(DATA_16BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_CMD_FORM  			    (305|(DATA_16BIT<<DATA_BIT_POSTION))

//32BIT
#define  MOTOR_REG_FAULTS_FLAGS  			(0|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_MEAS  				(1|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_REF  				(2|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_EST_BEMF  		(3|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOPLL_OBS_BEMF  		(4|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_EST_BEMF  		(5|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_STOCORDIC_OBS_BEMF  		(6|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FF_1Q 	 				(7|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FF_1D  					(8|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_FF_2  					(9|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_PFC_FAULTS  				(40|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_CURRENT_POSITION  		(41|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_RS  					(91|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_LS  					(92|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_KE  					(93|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_VBUS  				(94|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_MEAS_NOMINALSPEED  	(95|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_CURRENT  				(96|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_SPDBANDWIDTH  		(97|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_LDLQRATIO  			(98|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_NOMINAL_SPEED  		(99|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_CURRBANDWIDTH  		(100|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_J  					(101|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_F  					(102|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_MAX_CURRENT  			(103|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_STARTUP_SPEED  		(104|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SC_STARTUP_ACC  			(105|(DATA_32BIT<<DATA_BIT_POSTION))

#define  MOTOR_REG_POSITION_MEAS_DEGREE  	(200|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_MEAS_RPM  			(201|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_CURRENT_MEAS_A  			(202|(DATA_32BIT<<DATA_BIT_POSTION))

#define  MOTOR_REG_SPEED_KP1		  		(220|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KI1		  		(221|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KD1		  		(222|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KP2		  		(223|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KI2		  		(224|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPEED_KD2		  		(225|(DATA_32BIT<<DATA_BIT_POSTION))

#define  MOTOR_SPD_PID_V1		  		    (230|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_SPD_PID_V2		  		    (231|(DATA_32BIT<<DATA_BIT_POSTION))

#define  MOTOR_REG_POSITION_KP1		  		(240|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KI1		  		(241|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KD1		  		(242|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KP2		  		(243|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KI2		  		(244|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSITION_KD2		  		(245|(DATA_32BIT<<DATA_BIT_POSTION))

#define  MOTOR_REG_POSCMDFILTER		  		(260|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_POSSPDFILTER		  		(261|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPDCMDFILTER		  		(262|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_SPDCURFILTER		  		(263|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_CURCMDFILTER		  		(264|(DATA_32BIT<<DATA_BIT_POSTION))

#define  MOTOR_REG_ENCODER_ALIGN			(300|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_ALIGN_CURRENT			(302|(DATA_32BIT<<DATA_BIT_POSTION))

#define  MOTOR_REG_RS				 		(310|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_LS				 		(311|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_NOMINAL_CURRENT			(312|(DATA_32BIT<<DATA_BIT_POSTION))
#define  MOTOR_REG_ENCODER_PPR				(313|(DATA_32BIT<<DATA_BIT_POSTION))


//

/*数据通信错误码 */
#define MOTOR_CMD_OK      0x00
#define MOTOR_CMD_NOK     0x01
#define MOTOR_CMD_UNKNOWN 0x02
#define MOTOR_DATAID_UNKNOWN  0x03
#define MOTOR_ERROR_RO_REG 0x04
#define MOTOR_ERROR_UNKNOWN_REG 0x05
#define MOTOR_ERROR_STRING_FORMAT 0x06
#define MOTOR_ERROR_BAD_DATA_TYPE 0x07

#define MOTOR_ERROR_BAD_RAW_FORMAT 0x0A
#define MOTOR_ERROR_WO_REG 0x0B
#define MOTOR_ERROR_REGISTER_ACCESS 0x0C
#define MOTOR_ERROR_CALLBACK_NOT_REGISTRED 0x0D




#define PI 3.14159

typedef struct
{
    uint16_t 	MotorPolePair ;			/* Number of motor pole pairs */
    float  		MotorRS;             	/* Stator resistance , ohm*/
    float 		MotorLS;          		/* Stator inductance, H For I-PMSM it is equal to Lq */
    float 	    MotorNominalCurrent;	/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the PID for speed regulation (i.e. reference torque).
                                         Transformation of real currents (A) into int16_t format must be done accordingly with
                                         formula:  Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *Amplifying network gain)/(MCU supply voltage/2)*/
    uint16_t 	MotorMaxSpdRPM ;		/* MAX speed of motor */
    /***************** MOTOR SENSORS PARAMETERS  ******************************/
    /* Motor sensors parameters are always generated but really meaningful only
       if the corresponding sensor is actually present in the motor         */

    /*** Quadrature encoder ***/
    uint32_t  MotorEncoderPPR;   		/*!< Number of pulses per revolution */



}Motor_par;//电机物理参数结构体

class Kvaser final
{
public:
    enum Mode
    {
        TORQUE_MODE = 1, // 力矩/电流模式
        SPEED_MODE = 2, // 速度模式
        MICRO_STEPPING_MODE = 3,
        DUAL_FEEDBACK_POSITION_MODE = 4,
        SINGLE_FEEDBACK_POSITION_MODE = 5, // 位置模式
    };

    enum ErrorType
    {
        INIT_ERROR = 1,
        CONNECT_ERROR = 2,
        DISCONNECT_ERROR = 3,
        MODE_CHOOSE_ERROR = 4,
        RELEASE_ERROR = 5,
        RUNNING_ERROR = 6,
    };

    union RV_TypeConvert
    {
        float to_float;
        int to_int;
        unsigned int to_uint;
        uint8_t buf[4];
    }rv_type_convert;





    int canInit(int channel_number = DEFAULT_CHANNEL_NUMBER);

    int canRelease();

    int canSend(uint8_t *msg, long can_id, unsigned int dlc);

    int connectMotor(long can_id);

    int motorEnable(long can_id);

    int motorDisable(long can_id);

    int modeChoose(long can_id, Mode mode);

    int FPMixMode(long can_id, float kp,float kd,float position, float speed, float fowardCurrent);
    int FPMixMode(long can_id, float kp,float kd,float position, float speed, float fowardCurrent,float* torque_real,float* speed_real,float* position_real,int* motorEn,int* errflag);

    /*
     *hpb20250320添加，与positionMode其他不同的是，速度、电流限制提取写在了操作杆类
     */
    int positionMode(int use, long can_id, float position, float speed, float current);
    int positionMode(long can_id, float position, float speed, float current);
    int positionMode(long can_id, float position, float speed, float current,float* torque_real,float* speed_real,float* position_real,int* motorEn,int* errflag);

    /*
     *hpb20250320添加，与speedMode其他不同的是，速度、电流限制提取写在了操作杆类
     */
    int speedMode(int use, long can_id, float speed ,float current);
    int speedMode(long can_id, float speed ,float current);
    int speedMode(long can_id, float speed ,float current,float* torque_real,float* speed_real,float* position_real,int* motorEn,int* errflag);

    /*
     *hpb20250320添加，与torqueMode其他不同的是，速度、电流限制提取写在了操作杆类
     */
    int torqueMode(int use,long can_id, float torque);
    int torqueMode(long can_id, float torque);
    int torqueMode(long can_id, float torque,float* torque_real,float* speed_real,float* position_real,int* motorEn,int* errflag);

    int motorReset(long can_id);

    float protect(float torque);

    int beginMovement(long can_id);

    float getPosition(long can_id,uint8_t *motor_state);

    float getVelocity(long can_id,uint8_t *motor_state);

    int getInfAndRead(uint8_t *send_msg, uint8_t *read_msg, long can_id, unsigned int send_dlc, unsigned int *read_dlc);

    void dataFloatIntegration(uint8_t *temp, float *temp_value);

    float getCurrent(long can_id,uint8_t *motor_state);

    float getPower(long can_id,uint8_t *motor_state);

    int MotorSetting(long can_id,uint8_t cmd);

    int MotorIDReading(long *id_get);

    int MotorCommModeReading(long can_id,uint8_t *ComMode);

    int MotorIDReset();

    int MotorIDSet(long can_id,long new_id);

    int MotorZeroSet(long can_id);

    int MotorGetVersion(long can_id,uint32_t *Version);
    int MotorGetReg(long can_id,uint16_t RegNumb,uint32_t *DataGet);
    int MotorSetReg(long can_id,uint16_t RegNumb,uint32_t *DataSet);
    int MotorOnOff(long can_id,uint8_t CtlData);
    int MotorGetErrorCode(long can_id,uint16_t *FaultOccurred, uint16_t *FaultNow);
    int MotorErrorClear(long can_id);
/**王工测试**/
    int MotorSpeedRun(long can_id,int16_t SpeedRun);
    int MotorStopID(long can_id);
    int MotorGetC(long can_id,long *motorC);
    int MotorLocationRun(long can_id, int32_t LocationRun);
    int Motorspeed_location(long can_id,int32_t LocationRun,int16_t SpeedRun);
    int MotorCSPReading(long id_get, int16_t *C_CSP, int16_t *S_CSP, int32_t *P_CSP);
    //hpb 20250318
    int selfDevMotorStop(long id);
    int canInit(int channel_number, int canbitRate = canBITRATE_1M);

private:
    canHandle handle;
    canStatus status;
    static const int DEFAULT_CHANNEL_NUMBER = 0;
    constexpr static const float LOW_LIMIT = -5500.00;
    constexpr static const float HIGH_LIMIT = 5500.00;

    int checkStatus(const string &id);

    void dataIntegrationInt(uint8_t *temp, int32_t temp_value);

    void dataIntegrationFloat(uint8_t *temp, float temp_value);
};

#endif //ROS_KVASER_CAN_DRIVER_KVASER_CAN_H
