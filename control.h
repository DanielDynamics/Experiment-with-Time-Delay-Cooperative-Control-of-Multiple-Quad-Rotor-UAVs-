/*
 * File:   Control.h
 * Author: Zhentao Xie
 *
 * 
 * Modified from original PID control created by 01-2014, vincent jaunet (vincent.jaunet@hotmail.fr)
 */

#ifndef Included_ControlClass_H
#define Included_ControlClass_H

#include <iostream>
#include <stdint.h>
#include <time.h>

#include "net.h"
#include "servo.h"
#include "pid.h"
#include "dmp.h"

class ControlClass
{
 public:
  ControlClass();
  ~ControlClass();

  void start_sign();
  void stop_sign();
  void Run_Control();
  float dt;
  bool started;

  float thr, ypr_setpoint[3], ypr_recevpoint[3];
  
  //float Ixx;
  //float Beta;
  float MaxOutput;
  float consensus;
  float Rollset;
  float k1, k2, k3;
 

 private:
  
  timespec oldtime_;
  timespec time_;

  void calcdt_();

  //PID variables
  float kp_,ki_,kd_;
  float PIDout[3];
  float PIDout_roll;
  

};

extern ControlClass Control;
  
#endif      
