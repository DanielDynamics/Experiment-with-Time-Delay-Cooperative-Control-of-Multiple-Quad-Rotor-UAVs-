/*
 * File:   Control.cpp
 * Author: Zhentao Xie
 *
 *
 *
 * Implement consensus control.
 * 
 * The consensus control law for follower is U_i =  follower_PID + k1(x0_integral - x1_integral)
   + k2(x0 - x1) + k3(x0_dot - x1_dot)
 * 
 * Modified from a original PID controller created by vincent jaunet (vincent.jaunet@hotmail.fr)
 *
 */

#include "control.h"

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3


ControlClass Control;


ControlClass::ControlClass()
{
   started = false;
   //MaxOutput = 250.0;
}


 ControlClass::~ControlClass()
 {
 }

void ControlClass::start_sign()
{
  started = true;
}

void ControlClass::stop_sign()
{
  started = false;
}

inline void ControlClass::calcdt_()
{
  oldtime_ = time_;
  clock_gettime(CLOCK_MONOTONIC, &time_);
  Control.dt = ((static_cast <int64_t>(time_.tv_sec) * 1000000000 +
	       static_cast <int64_t>(time_.tv_nsec)) -
	      (static_cast <int64_t>(oldtime_.tv_sec) * 1000000000 +
	       static_cast <int64_t>(oldtime_.tv_nsec))) / 1000000000.0;
  // printf("dt:%f\n",dt);
}

void ControlClass::Run_Control()
{

  //1- get attitude of the drone
  imu.getAttitude();
    
  //printf("%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n",remote.recev_buf.yaw,remote.recev_buf.pitch,remote.recev_buf.roll,
		//remote.recev_buf.yaw_rate, remote.recev_buf.pitch_rate, remote.recev_buf.roll_rate);
		
  printf("roll_intg: %.2f   roll_rate: %.2f   roll: %.2f  neibour_roll: %.2f\n", remote.recev_buf.roll_intg, imu.gyro[ROLL], imu.ypr[ROLL], remote.recev_buf.roll);
  
  //2- Timer dt
  Control.calcdt_();
  
  //3- Calculate PID
  
  Control.PIDout_roll = pid_roll.update_pid_std(Control.Rollset, imu.ypr[ROLL], Control.dt);
    
  //4- Calculate Consensus Control (Consensus = Follower PID + Consensus)
  Control.PIDout_roll = Control.PIDout_roll + (k1*(remote.recev_buf.roll_intg - pid_roll.rollIntg) + k2*(remote.recev_buf.roll - imu.ypr[ROLL]) + k3*(remote.recev_buf.roll_rate - imu.gyro[ROLL]));

  //5- ESC update 
  if (Control.started)
  {
    ESC.update_roll(Control.thr,Control.PIDout_roll);
    printf("thr: %7.2f  ConsensusPID: %7.2f\n",Control.thr,Control.PIDout_roll);

  }
 
   return; 
}
