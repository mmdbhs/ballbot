#ifndef  _PID_
#define _PID_
#include <Arduino.h>


typedef struct 
{
	float KP ;//pid三个参数
	float KI;
	float KD;
	int16_t I_Max;
	int16_t I_Min;
	int32_t Out_Max;
	int32_t Out_Min;
	
	float p_out;
	float i_out;
	float d_out;
	
	int16_t error;//当前误差
	int16_t last_error;//上一次误差
}User_PID_Typedef;

/*
	@brief	初始化结构体里面的变量
*/
void PID_Init(User_PID_Typedef* mypid, float KP, float KI,float KD, int16_t IMAX, int16_t OutMAX);

/*
	@brief	计算pid的输出
	@param	now：当前值
	@param	goal：目标值
	@retval output:pid计算的输出值
*/
int16_t Pid_Out(User_PID_Typedef *mypid,int16_t now,int16_t goal);

#endif




