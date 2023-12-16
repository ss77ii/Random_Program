/*
 * @Author: LIUXY 2816184983@qq.com
 * @Date: 2023-10-13 11:49:21
 * @LastEditors: LIUXY 2816184983@qq.com
 * @LastEditTime: 2023-11-29 16:11:46
 * @FilePath: \esp32came:\PlatformIO\Projects\MovewithESPNOW\src\pid.hpp
 * @Description: 鏉╂瑦妲告妯款吇鐠佸墽鐤�,鐠囩柉顔曠純鐢ustomMade`, 閹垫挸绱慿oroFileHeader閺屻儳婀呴柊宥囩枂 鏉╂稖顢戠拋鍓х枂: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _PID_HPP
#define _PID_HPP

#include "Arduino.h" // 

class _PID
{
  private:
    /* data */

  public:
  float target_val; // 
  float actual_val; // 
  float err;        // 
  float err_last;   //
  float Kp, Ki, Kd; //
  float integral;   // 

  _PID();
  float PID_realize(float temp_val);
  void set_p_i_d(float p, float i, float d);
  float get_pid_target(void);
  void set_pid_target(float temp_val);
  void param_init(void);
};

_PID::_PID()
{
  this->target_val = 0.0;
  this->actual_val = 0.0;
  this->err = 0.0;
  this->err_last = 0.0;
  this->integral = 0.0;
  this->Kp = 0.0; // 24
  this->Ki = 0.0;
  this->Kd = 0.0;
}

void _PID::set_pid_target(float temp_val)
{
  this->target_val = temp_val;

}
float _PID::get_pid_target(void)
{
  return this->target_val;
}

void _PID::set_p_i_d(float p, float i, float d)
{
  this->Kp = p; // 
  this->Ki = i;
   //  I
  this->Kd = d;
   // D
}

float _PID::PID_realize(float temp_val)
{
  /**/
  this->err = this->target_val - temp_val;
  /**/
  this->integral += this->err;
  /**/
  this->actual_val = this->Kp * this->err + this->Ki * this->integral + this->Kd * (this->err - this->err_last);
  /**/
  this->err_last = this->err;
  /**/
  return this->actual_val;
}

#endif
