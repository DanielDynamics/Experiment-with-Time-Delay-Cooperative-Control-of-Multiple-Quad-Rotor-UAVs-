/*
  ------------------
   Time-Delay Cooperative Control for Multiple Quadrotor UAVs
  ------------------
  author : Zhentao Xie

  - Initialization of PID
  - Initialization of ESC
  - Initialization of MPU6050
  - Initialization of Remote socket


 This project is devoted to implement a real world test for a time-delay cooperative control algorithm on quadrotor-UAVs (Unmanned Aerial Vehicles). The applied control algorithm designed by Zhentao Xie & Ming Xin is to control multiple agents to follow a leader consensually with time varying delay and model uncertainties. The experiment mainly tested quadrotor-UAVs attitudes (roll and pitch) performance. 
 The flight control program is coded in Raspberry-Pi Micro-Computer. UDP (User Datagram Protocol) communication protocol is used for network communication among UAVs The attitude values are gained from MPU6050 gyroscop sensor. The experiment results show that control algorithm works well and the quadrotor-UAVs can be stable at varying attitudes consensually.
 
 This software is modified based on Vincent Jaunet (vincent.jaunet@hotmail.fr) Quadcopter Pilot control program.
*/


#include "main.h"

//-------------------------------------
//--------- Main-----------------------

int main(int argc, char *argv[])
{
  
  if(argc != 3)
  {
	 printf("Usage %s hostname port\n",argv[0]);
	 exit(1);
  }
  printf("QuadCopter Pilot v0.1\n");
  printf("----------------------\n");
  printf("\n");

  //initializing Network communication for RC to UAV
  remote.create();
  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //initializing Network communication for connection UAV to UAV
  remote.sockID = socket(AF_INET, SOCK_DGRAM, 0);// creates socket. connetctionless
	if(remote.sockID < 0)
	{
		printf("Socket error\n");
		exit(-1);
	}
	bzero(&remote.my_addr,remote.length);
	//my IP address information 
	remote.my_addr.sin_family = AF_INET; 
	remote.my_addr.sin_addr.s_addr = INADDR_ANY; //local PC IP address
	remote.my_addr.sin_port = htons(atoi(argv[2]));
	if(bind(remote.sockID,(struct sockaddr*)&remote.my_addr,remote.length)<0)
	{
		printf("Binding error\n");
		exit(-1);
	}
	
	//recepient IP address information
	struct hostent *hp = gethostbyname(argv[1]); //converts hostname. input "192.168.1.137" recepient IP address
	if(hp==0)
	{
		printf("Unknown host hp\n");
		exit(-1);
	}
	bcopy((char *)hp->h_addr, (char *)&remote.output_addr.sin_addr,hp->h_length); //IP address of destination
	remote.output_addr.sin_family = AF_INET; //symbol constant for Internet domain
	remote.output_addr.sin_port = htons(atoi(argv[2])); //port filed
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Waiting fo Start command */
  while (true){
	  
    printf("In main loop\n");
    remote.exec_remoteCMD();
    usleep(2000);

  }//end

  return 0;
}


//
