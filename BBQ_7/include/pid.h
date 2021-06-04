#ifndef  _PID_
#define _PID_
#include <Arduino.h>


typedef struct 
{
	float KP ;//pid��������
	float KI;
	float KD;
	float I_Max;
	float I_Min;
	float Out_Max;
	float Out_Min;
	
	float p_out;
	float i_out;
	float d_out;
	
	float error;//��ǰ���
	float last_error;//��һ�����
}User_PID_Typedef;

/*
	@brief	��ʼ���ṹ������ı���
*/
void PID_Init(User_PID_Typedef* mypid, float KP, float KI,float KD, int16_t IMAX, int16_t OutMAX);

/*
	@brief	����pid�����
	@param	now����ǰֵ
	@param	goal��Ŀ��ֵ
	@retval output:pid��������ֵ
*/
float Pid_Out(User_PID_Typedef *mypid,float now,float goal);

#endif




