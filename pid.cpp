/*
   PID Controller

   author : Zhentao Xie

   Description:
   the PID class is a collection of routine necessary
   to perform a PID control based on setpoints (remote control),
   inputs (measured attitude).
   Can be used for any type of system and features :
   - Derivative on measurement
   - Windsup of integral errors
 
 * Modified from original PID Controller created by vincennt JAUNET (vincent.jaunet@hotmail.fr)
*/

#include "pid.h"


//PID yprSTAB[3];
//PID yprRATE[3];
PID pid_roll;
//PID pid_rollRate;

//default constructo
PID::PID()
{
  //PID constants
  m_Kd = 0;
  m_Ki = 0;
  m_Kp = 0;

  //PID variables
  m_err = 0;
  m_sum_err = 0;
  m_ddt_err = 0;
  m_lastInput= 0;
  //m_outmax =  350;
  //m_outmin = -350;
  m_outmax =  200;
  m_outmin = -200;
}


PID::PID(float kp_,float ki_,float kd_)
{
  //PID constants
  m_Kd = kp_;
  m_Ki = ki_;
  m_Kp = kd_;

  //PID variables
  m_err = 0;
  m_sum_err = 0;
  m_ddt_err = 0;
  m_lastInput= 0;
  //m_outmax =  400;
  //m_outmin = -400;
  m_outmax =  200;
  m_outmin = -200;
}



float PID::update_pid_std(float setpoint, float input, float dt)
{

  //Computes error
  m_err = setpoint-input;

  //Integrating errors
  m_sum_err += m_err * m_Ki * dt;
  
  rollIntg = m_sum_err;

  //calculating error derivative
  //Input derivative is used to avoid derivative kick
  //m_ddt_err = -m_Kd / dt * (input - m_lastInput);
  m_ddt_err = m_Kd / dt * (m_err - m_lastErr);

  //Calculation of the output
  //winds up boundaries
  m_output = m_Kp*m_err + m_sum_err + m_ddt_err;
  if (m_output > m_outmax) {
    //winds up boundaries
    m_sum_err  = 0.0;
    m_output   = m_outmax;
  }else if (m_output < m_outmin) {
    //winds up boundaries
    m_sum_err  = 0.0;
    m_output   = m_outmin;
  }

  //m_lastInput= input;
  m_lastErr = m_err;

  //printf("kp %f ki %f kd %f\n", m_Kp, m_Ki, m_Kd);
  //printf("setpt %7.2f input   %7.2f output   %f\n", setpoint, input, m_output);
  //printf("err   %7.2f ddt_err %7.2f sum_err  %7.2f\n", m_err, m_ddt_err, m_sum_err);
 

  return m_output;
}

float PID::update_pid_std(float error, float dt)
{

  //Computes error
  m_err = error;

  //Integrating errors
  m_sum_err += m_err * m_Ki * dt;

  //calculating error derivative
  //Input derivative is used to avoid derivative kick
  //m_ddt_err = -m_Kd / dt * (input - m_lastInput);
  m_ddt_err = m_Kd / dt * (m_err - m_lastErr);

  //Calculation of the output
  //winds up boundaries
  m_output = m_Kp*m_err + m_sum_err + m_ddt_err;
  if (m_output > m_outmax) {
    //winds up boundaries
    m_sum_err  = 0.0;
    m_output   = m_outmax;
  }else if (m_output < m_outmin) {
    //winds up boundaries
    m_sum_err  = 0.0;
    m_output   = m_outmin;
  }

  //m_lastInput= input;
  m_lastErr = m_err;

  //printf("kp %f ki %f kd %f\n", m_Kp, m_Ki, m_Kd);
  //printf("setpt %7.2f input   %7.2f output   %f\n", setpoint, input, m_output);
  //printf("err   %7.2f ddt_err %7.2f sum_err  %7.2f\n", m_err, m_ddt_err, m_sum_err);
 

  return m_output;
}



void PID::reset()
{
  m_sum_err   = 0;
  m_ddt_err   = 0;
  m_lastInput = 0;
  m_lastErr = 0;
}



void PID::set_Kpid(float Kp,float Ki, float Kd)
{
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;

}

void PID::set_windup_bounds(float Min,float Max)
{
  m_outmax = Max;
  m_outmin = Min;
}
