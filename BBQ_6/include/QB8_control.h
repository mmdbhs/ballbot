#ifndef _QB_CONTROL_H
#define _QB_CONTROL_H
#include <Arduino.h>
#include <SPI.h>
#include "pid.h"
/********************通信相关*****************************/
#define HSPI_MISO   26   //当前使用的spi引脚
#define HSPI_MOSI   27
#define HSPI_SCLK   25
#define HSPI_SS_one 33   //三个片选引脚
#define HSPI_SS_two 32
#define HSPI_SS_three  2



#define DEBUG_SERIAL Serial       
#define SERIAL_JY901 Serial2
/******************控制相关********************************/
#define CORRECT_ANGLE 0 //矫正陀螺仪摆放误差，角度制

#define BALANCE_KP 0   //平衡环pid
#define BALANCE_KI 0
#define BALANCE_KD 0
#define BALANCE_IMAX 2000
#define BALANCE_OUTMAX 5000
#define SPEED_KP 0     //速度环PID
#define SPEED_KI 0
#define SPEED_KD 0
#define SPEED_IMAX 1000
#define SPEED_OUTMAX 2000


#define MOTOR1_DIR 13   //三个电机方向引脚
#define MOTOR2_DIR 27
#define MOTOR3_DIR  32
#define MOTOR1_PUL 12   //三个电机脉冲引脚
#define MOTOR2_PUL 14
#define MOTOR3_PUL 33
#define EXCITATION 16  //电机驱动器细分
#define STEP_ANGLE 1.8 //步进电机步距角
#define MOTOR_LEFT 1 //步进电机向左转
#define MOTOR_RIGHT 0   //步进电机右转
/***********************************************************/
class QB8
{
public:
    static int16_t motor_speed[3];     //储存三个轮子的旋转速度
    float give_x_speed;         //底盘的目标运行状态
    float give_y_speed;
    int16_t angular_velocity;
    struct SAngle  //储存姿态数据
    {
      short Angle[3];
      short T;
    }stcAngle;
    User_PID_Typedef balance_x_pid;
    User_PID_Typedef balance_y_pid;
    User_PID_Typedef speed_x_pid;
    User_PID_Typedef speed_y_pid;
    hw_timer_t *timer[3] = {NULL,NULL,NULL};   //三个步进电机的定时器
private:
    SPIClass * hspi = NULL;
    static const int spiClk = 1000000; // 1 MHz ，通信波特率
    uint8_t cs_pin[3] = {HSPI_SS_one, HSPI_SS_two, HSPI_SS_three};
    uint8_t dir_pin[3] = {MOTOR1_DIR, MOTOR2_DIR, MOTOR3_DIR };
    char databuf[8];        //储存spi发送数据的数组
    int16_t real_motor_speed[3];    //储存三个电机的实际转速
    float real_x_speed;        //底盘的实际运行状态
    float real_y_speed;
    int16_t real_angular_velocity;
    uint8_t rxbuf[8];   //储存spi接收到的原始数据

        
public:
    
  //将车子的运动解算为三个轮子的旋转速度，运动学逆解
  void move_solution(float x_speed, float y_speed, int16_t angular_velocity);
  //读取JY901数据
  void serialEvent();
  //机器人初始化
  void BQ8Init();
  //发送电机旋转数据
  void send_data();
  //全向轮底盘运动学正解
  void obverse_calculate(void);
  //根据姿态传感器和电机速度，更新控制数据
  void control_update();
  //timer1的中断函数,给1号步进电机发送数据  
  static void timer1_interrupt();
  //timer2的中断函数,给2号步进电机发送数据  
  static void timer2_interrupt();
  //timer3的中断函数,给3号步进电机发送数据  
  static void timer3_interrupt();
private:
 //发送spi数据
  void hspiCommand(char *data, uint8_t len, uint8_t slave_num); 
  //将控制数据存入待发送数组
  void data_tansform(char *data, uint8_t mode, int16_t speed, int16_t pisition);
  //将接收到的原始数据转化为速度
  int16_t data_decode(uint8_t *data); 
  //spi通信初始化
  void SPI_Init();
  //控制步进电机旋转指定速度，单位为：rpm
  void step_speed_mode(uint8_t motor_num);

  
};


#endif