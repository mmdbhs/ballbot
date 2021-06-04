#include "pid.h"

/*
	@brief	����pid�����
	@param	now����ǰֵ
	@param	goal��Ŀ��ֵ
	@retval output:pid��������ֵ
*/
int16_t Pid_Out(User_PID_Typedef *mypid,int16_t now,int16_t goal)
{
	static int16_t output = 0;
	(mypid->error) = goal - now;
	mypid->p_out = (mypid->error)*(mypid->KP);
	mypid->i_out += (mypid->error)*(mypid->KI);
	mypid->d_out = ((mypid->error) - (mypid->last_error))*(mypid->KD);
	//��������
	if(mypid->i_out>(mypid->I_Max))
		mypid->i_out = mypid->I_Max;
	else if(mypid->i_out<(mypid->I_Min))
		mypid->i_out = mypid->I_Min;		
	
	output =mypid->p_out+mypid->i_out+mypid->d_out;
	//�������
	if(output >= (mypid->Out_Max))
		output = mypid->Out_Max ;
	else if(output <= (mypid->Out_Min))
		output = mypid->Out_Min;
	mypid->last_error = mypid->error;
	
	return output;
}
/*
	@brief	��ʼ���ṹ������ı���
*/
void PID_Init(User_PID_Typedef* mypid, float KP, float KI,float KD, int16_t IMAX, int16_t OutMAX)
{
	mypid->KP = KP;//pid��������
	mypid->KI = KI;
	mypid->KD = KD;
	mypid->I_Max = IMAX;
	mypid->I_Min = -IMAX;
	mypid->Out_Max = OutMAX;
	mypid->Out_Min = -OutMAX;

	mypid->error = 0;//��ǰ���
	mypid->last_error = 0;//��һ�����
}



