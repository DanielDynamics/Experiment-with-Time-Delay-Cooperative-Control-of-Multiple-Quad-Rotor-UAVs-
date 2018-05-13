# Experiment-with-Time-Delay-Cooperative-Control-of-Multiple-Quad-Rotor-UAVs-

This project is devoted to implement a real world test for a time-delay cooperative control algorithm on quadrotor-UAVs (Unmanned Aerial Vehicles). The applied control algorithm [1] is designed for multiple agents to follow a leader consensually with time varying delay and model uncertainties. The experiment mainly tested quadrotor-UAVs attitudes (roll and pitch) performance. The flight control program is coded in Raspberry-Pi Micro-Computer. The UDP (User Datagram Protocol) communication protocol is used for network communication among UAVs. A Mac is used as ground station to send commands signal. The raspberry-pi on the UAVs are set as pilots controllers. The experiment results show that the control algorithm works well and the quadrotor-UAVs can be stable at varying attitudes consensually.  

Hardware includes: Raspberry-Pi, Brushless DC motors (CW & CCW), 20 Amp ESCs, 2200mAh 3S 25C Lipo battery pack, Propellers, MPU6050 Gyroscopes, Quad-Rotor UAV frames.

Pin connection on Raspberry-Pi:
Servo           attitude    pin
Servo-0 (cw)           pitch+       7;
Servo-2 (cw)           pitch -      12;
Servo-1 (ccw)          roll+        11;
Servo-3 (ccw)          roll-        13;

The ServoBlaster program should be run before running the UAV programs.

ESC calibration is needed 

Reference: [1]  Z. Xie, M. Xin Cooperative Control for Multiple Agents with Time Varying Delay and Model Uncertainties
