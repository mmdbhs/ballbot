#ifndef  _PID_
#define _PID_
#include <Arduino.h>


typedef struct 
{
	float KP ;//pid��������
	float KI;
	float KD;
	int16_t I_Max;
	int16_t I_Min;
	int32_t Out_Max;
	int32_t Out_Min;
	
	float p_out;
	float i_out;
	float d_out;
	
	int16_t error;//��ǰ���
	int16_t last_error;//��һ�����
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
int16_t Pid_Out(User_PID_Typedef *mypid,int16_t now,int16_t goal);

#endif




