#include "QB8_control.h"
#include <SPI.h>
#include <Arduino.h>
#include "pid.h"

/**
 * @brief: spi初始化
 * @return： none
*/
void QB8::SPI_Init()
{
  hspi = new SPIClass(HSPI);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS_one); //SCLK, MISO, MOSI, SS
  pinMode(HSPI_SS_one, OUTPUT); //HSPI SS
  pinMode(HSPI_SS_two, OUTPUT); //HSPI SS
  pinMode(HSPI_SS_three, OUTPUT); //HSPI SS
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
}
/**
 * @brief: hspi发送数据
 * @param: data:待发送数据数组
 * @param：len：发送数据长度 
 * @param: slave_num:要发送信息的从机片选引脚
 * @return： none
*/
void QB8::hspiCommand(char *data, uint8_t len, uint8_t slave_num) {

  //use it as you would the regular arduino SPI API
  digitalWrite(slave_num, LOW); //pull SS slow to prep other end for transfer
  //delay(1);
  for(int i = 0;i < len;i++)
  {
    rxbuf[i] = hspi->transfer(data[i]);

    }
  digitalWrite(slave_num, HIGH); //pull ss high to signify end of data transfer

  hspi->endTransaction();
}
/**
 * @brief: 将数据存入发送数组
 * @param: data:待发送数据数组
 * @param：mode：控制模式；    0x00：速度模式  0x01：位置模式
 * @param：speed：速度模式的 速度  单位为rad/s
 * @param: pisition： 位置模式的 位置rad
 * @return： none
*/
void QB8::data_tansform(char *data, uint8_t mode, int16_t speed, int16_t pisition)
{
  data[0] = 'a';//帧头
  data[1] = mode;
  data[2] = (speed>>8) & 0xff;
  data[3] = speed & 0xff;
  data[4] = (pisition>>8) & 0xff;
  data[5] = pisition & 0xff;
  data[6] = char(data[0]+data[1]+data[2]+data[3]+data[4]+data[5]);//数据校验位

}

/**
 * @brief: 将接收到的原始数据转化为速度
 * @param: data:接收到的原始数据 
 * @return： 电机的速度，单位为rad/s/100
*/
int16_t QB8::data_decode(uint8_t  *data) 
{
  int16_t speed;
  if(data[4] == uint8_t(data[1] + data[2] + data[3]))
  {
    speed = data[2]<<8 | data[3]; 
  }
  else
  {
    DEBUG_SERIAL.println("decode error!");
     speed = 0;
  }
 

  return speed;
}
/**
 * @brief: 将车子的运动解算为三个轮子的旋转速度
 * @param：speed_car：车子的速度
 * @param：angle_car：车子的前进角度
 * @param: angular_velocity： 车子的旋转角速度
 * @return： none
*/
void QB8::move_solution(float x_speed, float y_speed, float angular_velocity)
{

  //速度解算
  motor_speed[0] = -sqrt(3)/2 * x_speed + 0.5* y_speed + angular_velocity;
  motor_speed[1] = -(sqrt(3)/2 * x_speed + 0.5* y_speed + angular_velocity);  //因为硬件原因，所以整体结果取负
  motor_speed[2] = -y_speed + angular_velocity;
  Serial.print("motor0_speed = ");
  Serial.println(motor_speed[0]);
   Serial.print("motor1_speed = ");
  Serial.println(motor_speed[1]);
   Serial.print("motor2_speed = ");
  Serial.println(motor_speed[2]);
}


/**
 * @brief: 全向轮底盘运动学正解，通过轮子速度解算运动运动
 * @return： none
*/
void QB8::obverse_calculate(void)
{

  real_motor_speed[1] = - real_motor_speed[1];
  real_x_speed = (real_motor_speed[0] - real_motor_speed[1])/sqrt(3);  //转化为直角坐标
  real_y_speed = (real_motor_speed[0] + real_motor_speed[1])/3 - 2 * real_motor_speed[2] / 3;
  
  real_angular_velocity = (real_motor_speed[0] + real_motor_speed[1] + real_motor_speed[2])/3;

  // DEBUG_SERIAL.print("real_car_speed =");
  // DEBUG_SERIAL.println(real_x_speed);
  // DEBUG_SERIAL.print("real_car_angle = ");
  // DEBUG_SERIAL.println(real_x_speed);
  // DEBUG_SERIAL.print("real_angular_velocity = ");
  // DEBUG_SERIAL.println(real_angular_velocity);
}

/**
 * @brief: 发送电机旋转速度数据
 * @return： none
*/
void QB8::send_data()
{
    for(int i = 0; i < sizeof(dir_pin);i++)
    {
      step_speed_mode(i);
    }
}
/**
 * @brief 读取JY901数据，并更新控制数据
 * @return none
 */
void QB8::serialEvent() 
{
  static unsigned char ucRxBuffer[250];
  static unsigned char ucRxCnt = 0; 
  //SERIAL_DEBUG.println("进入");
  ucRxBuffer[ucRxCnt++]=SERIAL_JY901.read();
  if (ucRxBuffer[0]!=0x55) 
  {
    ucRxCnt=0;
    return;
  }
  if (ucRxCnt<11) {return;}
  else
  {
    if(ucRxBuffer[1] == 0x53)
    {
      memcpy(&stcAngle,&ucRxBuffer[2],8);

      control_update(); //更新控制数据
      } 
      ucRxCnt=0;  
      //SERIAL_DEBUG.println("退出");
  }

}
//机器人初始化
/**
 * @brief 相关初始化操作
 * @return none
 */
void QB8::BQ8Init()
{
    //SPI_Init();
    timer[0] = timerBegin(1, 80, true);   //定时器初始化
    timer[1] = timerBegin(2, 80, true);
    timer[2] = timerBegin(3, 80, true);
    timerAttachInterrupt(timer[0], &timer1_interrupt, true);
    timerAttachInterrupt(timer[1], &timer2_interrupt, true);
    timerAttachInterrupt(timer[2], &timer3_interrupt, true);
    timerAlarmWrite(timer[0], 1000000/2000, true);
    timerAlarmEnable(timer[0]);

    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(MOTOR1_PUL, OUTPUT);
    pinMode(MOTOR2_PUL, OUTPUT);
    pinMode(MOTOR3_PUL, OUTPUT);
    //初始化PID
    PID_Init(&balance_x_pid, BALANCE_KP, BALANCE_KI, BALANCE_KD, BALANCE_IMAX, BALANCE_OUTMAX);
    PID_Init(&balance_y_pid, BALANCE_KP, BALANCE_KI, BALANCE_KD, BALANCE_IMAX, BALANCE_OUTMAX);
    PID_Init(&speed_x_pid, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_IMAX, SPEED_OUTMAX);
    PID_Init(&speed_y_pid, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_IMAX, SPEED_OUTMAX);

    
}

/**
 * @brief 根据姿态传感器和电机速度，更新控制数据
 * @return none
 */
void QB8::control_update()
{
  //直角坐标变换，矫正姿态传感器的放置误差
  float x_angle;
  float y_angle;

  x_angle = (float)stcAngle.Angle[0]*180/32768;
  y_angle = (float)stcAngle.Angle[1]*180/32768;

  DEBUG_SERIAL.print("x_angle = ");
  DEBUG_SERIAL.println(x_angle);
  DEBUG_SERIAL.print("y_angle = ");
  DEBUG_SERIAL.println(y_angle);
  x_angle = x_angle * cos(CORRECT_ANGLE* PI/ 180) - y_angle * sin(CORRECT_ANGLE* PI/ 180);
  y_angle = y_angle * cos(CORRECT_ANGLE* PI/ 180) + x_angle * sin(CORRECT_ANGLE* PI/ 180);
  //上一帧的脉冲频率作为速度
  if(abs(x_angle) > MAX_ERROR_ANGLE || abs( y_angle) > MAX_ERROR_ANGLE)  //完全失稳，停止平衡
  {
    give_x_speed = 0;
    give_y_speed = 0;
  }
  else
  {
    give_x_speed = Pid_Out(&balance_x_pid, x_angle, X_BALANCE_ANGLE) -  Pid_Out(&speed_x_pid, give_x_speed, 0); 
    give_y_speed = Pid_Out(&balance_y_pid, y_angle, Y_BALANCE_ANGLE) -  Pid_Out(&speed_y_pid, give_y_speed, 0);
  }
    angular_velocity = 0; //不需要角速度
    DEBUG_SERIAL.print("give_x_speed = ");
    DEBUG_SERIAL.println(give_x_speed);
    DEBUG_SERIAL.print("give_y_speed = ");
    DEBUG_SERIAL.println(give_y_speed);
}

/**
 * @brief timer1的中断函数,给1号步进电机发送数据
 * @return none
 */
void QB8::timer1_interrupt()
{
  //DEBUG_SERIAL.println("i am in timer1!");


  if((motor_speed[0]) != 0)  //是否有脉冲需要发送
  {
    digitalWrite(MOTOR1_PUL, HIGH);
    //delayMicroseconds(10);
    digitalWrite(MOTOR1_PUL, LOW);
  }
}
/**
 * @brief timer2的中断函数,给2号步进电机发送数据
 * @return none
 */
void QB8::timer2_interrupt()
{

  //DEBUG_SERIAL.println("i am in timer2 !");

  if((motor_speed[1]) != 0)  //是否有脉冲需要发送
  {
    
    digitalWrite(MOTOR2_PUL, HIGH);
    digitalWrite(MOTOR2_PUL, LOW);
  }
}
/**
 * @brief timer3的中断函数,给3号步进电机发送数据
 * @return none
 */
void QB8::timer3_interrupt()
{

  //DEBUG_SERIAL.println("i am in timer3 !");

  if((motor_speed[2]) != 0)  //是否有脉冲需要发送
  {
    digitalWrite(MOTOR3_PUL, HIGH);
    digitalWrite(MOTOR3_PUL, LOW);
  }
}

/**
 * @brief 控制步进电机旋转指定速度，单位为：rpm
 * @param motor_num:控制电机的序号
 * @return none
*/
void QB8::step_speed_mode(uint8_t motor_num)
{
  int pulse_frequency;
  if(motor_speed[motor_num] == 0)
  {
    timerAlarmDisable(timer[motor_num]) ;//关闭定时器中断
    return;
  }
  if(motor_speed[motor_num] < 0)        //判断旋转方向
  {
    digitalWrite(dir_pin[motor_num], MOTOR_LEFT);
  }
  else
  {
    digitalWrite(dir_pin[motor_num], MOTOR_RIGHT);
  }

  pulse_frequency = abs(motor_speed[motor_num] * 360* EXCITATION/ 60  / STEP_ANGLE );
  if( pulse_frequency < 1)
  {
    pulse_frequency = 1;
  }
  timerAlarmWrite(timer[motor_num], 1000000 / pulse_frequency, true);  //调节定时器触发频率
  timerAlarmEnable(timer[motor_num]);   //开启中断
}