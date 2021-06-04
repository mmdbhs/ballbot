#ifndef  _PID_
#define _PID_
#include <Arduino.h>


typedef struct 
{
	float KP ;//pid三个参数
	float KI;
	float KD;
	float I_Max;
	float I_Min;
	float Out_Max;
	float Out_Min;
	
	float p_out;
	float i_out;
	float d_out;
	
	float error;//当前误差
	float last_error;//上一次误差
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
float Pid_Out(User_PID_Typedef *mypid,float now,float goal);

#endif




