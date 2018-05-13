#ifndef SOCKET_H
#define SOCKET_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <pthread.h>
#include <sys/types.h>

#include "net.h"
#include "servo.h"
#include "pid.h"
#include "dmp.h"
#include "parser.h"
#include "control.h"



class Socket{

 private:
  sockaddr_in m_address;
  unsigned short m_port;
  int m_socket;
  unsigned char m_lastdata[256];
  int get_cmd(float& ,float& ,float& ,float&);
  int get_cmd();
  int boolval;


 public:
 
 struct sockaddr_in my_addr, input_addr, output_addr;
 int sockID;
 typedef struct
 {
	float yaw;
	float pitch;
	float roll;
	float yaw_rate;
	float pitch_rate;
	float roll_rate;
	float roll_intg;
 }Buf;
 Buf recev_buf, send_buf;
 unsigned int length;
  
  Socket();
  ~Socket();

  //void set_port(int port);
  void create();
  void Close();
  void exec_remoteCMD();
  //unsigned char data[256];
  char data[256];
  pthread_t thread_RC;
  pthread_t thrdID;
  static void * Thread_func_RC(void *ptr);
  static void * Thrdfunc_recevfrom(void *ptr);
  void start_thread();

};

extern Socket remote;

#endif
