/*
项目：改为步进电机控制
时间：2021.6.1
作者：默默的保护神
结果：
*/

#include <SPI.h>
#include <Arduino.h>
// Define ALTERNATE_PINS to use non-standard GPIO pins for SPI bus
#include "QB8_control.h"

/******************************************/
void serialReceiveUserCommand(); //在线调参
/********************************************/
//uninitalised pointers to SPI objects
QB8 myQB8;  //建立我的 QB8 对象
int16_t QB8::motor_speed[3] = {-200,330,-220};

void setup() {

  myQB8.BQ8Init();  //初始化
  DEBUG_SERIAL.begin(115200);
  SERIAL_JY901.begin(115200);//打开串口

  myQB8.give_x_speed =0;  //速度初值
  myQB8.give_y_speed = 0;
  myQB8.angular_velocity = 0;


}

void loop() {
  //use the SPI buses

  while(SERIAL_JY901.available())  //接收JY901数据
    myQB8.serialEvent();
  DEBUG_SERIAL.print("Angle:");
  DEBUG_SERIAL.print((float)myQB8.stcAngle.Angle[0]/32768*180);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print((float)myQB8.stcAngle.Angle[1]/32768.0*180);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println((float)myQB8.stcAngle.Angle[2]/32768.0*180);

  myQB8.move_solution(myQB8.give_x_speed, myQB8.give_y_speed, myQB8.angular_velocity);  //解算速度
  
  myQB8.send_data();  //发送数据

  //delay(500);

  serialReceiveUserCommand();//串口调参

}
/**
 * @brief 串口调参
 * @return none
 */
void serialReceiveUserCommand() {
  
  static String received_chars;
  static uint8_t flag = 0;

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n' ) {
      if(flag == 0)       
      {
        myQB8.balance_x_pid.KP = received_chars.toFloat();
        myQB8.balance_y_pid.KP = received_chars.toFloat();
        Serial.print("myQB8.balance_x_pid.KP= ");
        Serial.println(myQB8.balance_x_pid.KP);
        received_chars = "";
        flag = 1;
      }
      else
      {
        myQB8.balance_x_pid.KD = received_chars.toFloat();
        myQB8.balance_y_pid.KD = received_chars.toFloat();
        Serial.print("myQB8.balance_x_pid.KD= ");
        Serial.println(myQB8.balance_x_pid.KD);
        received_chars = "";
        flag = 0;
      }
     
    }
    
  }
}