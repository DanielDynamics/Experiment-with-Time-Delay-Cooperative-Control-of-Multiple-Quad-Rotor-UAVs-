/*

  Socket class
  author : Zhentao Xie

  Description :
  The Socket class contains all the necessary routine to set
  up and use a UDP network serve on the drone waiting for
  the remote client to send request.
  The output should be parsed in order to retrieve
  the desired attitude from the remote (see parse.*)
 
 *Modified from a original Socket class created by vincent jaunet (vincent.jaunet@hotmail.fr)

  

*/

#define YAW 0
#define PITCH 1
#define ROLL 2

#define STOP_PID  666
#define START_PID 1
#define INIT 2
#define SHUTDOWN 777
#define NOTHING  0
#define UPDATE_REMOTE 9
#define UPDATE_PID_YAW_RATE 10
#define UPDATE_PID_YAW_STAB 11
#define UPDATE_PID_PR_RATE 12
#define UPDATE_PID_PR_STAB 13



#include "net.h"

Socket remote;

Socket::Socket()
{
  data[0] = '\0';
  m_port = 3000;
  m_address.sin_family = AF_INET;
  m_address.sin_addr.s_addr = INADDR_ANY;
  m_address.sin_port = htons( (unsigned short) m_port );
  boolval = 1;
  length = sizeof(struct sockaddr_in);
}


Socket::~Socket()
{
  Close();
}



void Socket::create()
{

  //Opening socket
  m_socket = socket( AF_INET, SOCK_DGRAM, 0 );
  if ( m_socket <= 0 )
    {
      printf( "Failed to create socket\n" );
      exit(EXIT_FAILURE);
    }

  //Binding to desired port number
  if ( bind( m_socket, (const sockaddr*) &m_address, sizeof(sockaddr_in) ) < 0 )
    {
      printf( "failed to bind socket\n");
      exit(EXIT_FAILURE);
    }
  if (setsockopt(m_socket, SOL_SOCKET, SO_BROADCAST, &boolval, sizeof(boolval)) < 0)
   	{
   		printf("error setting socket options\n");
   		exit(-1);
   	}
  

  printf( "Succeed to create socket! \n" );

}


void Socket::Close(){

  if ( m_socket != 0 )
    {
      close( m_socket );
      m_socket = 0;
    }
}

int Socket::get_cmd(){

  int size = sizeof(data);
  assert( size > 0 );

  if ( m_socket == 0 )
    printf("Socket is closed in get_cmd...");

  int received_bytes = -1;
  sockaddr_in from;
  socklen_t fromLength = sizeof( from );
  printf("Waiting for RC command...\n");
  received_bytes = recvfrom( m_socket, data, size, 0,
				 (sockaddr*)&from,
				 &fromLength);
  if (received_bytes == -1)
  {
    printf("received bytes = -1 \n");
    return NOTHING;
  }

  
  
  int type=0;
  
  printf("Your command is :\n");
  std::cout << data << std::endl;
  
  std::string packet( reinterpret_cast< char const* > (data));
  std::istringstream ss(packet);

  std::string sub;
  ss >> sub;

  do{
    if (sub == "START")
    {
      type = START_PID;
      break;
    }
    else if (sub == "STOP")
    {
      type = STOP_PID;
      break;
    }
    else if (sub == "EXIT")
    {
      type = SHUTDOWN;
      break;
    }
    else if (sub == "INIT")
    {
      type = INIT;
      break;
    } 
    else if (sub == "UPDATE_REMOTE")
    {
		type = UPDATE_REMOTE;
		break;
	}
    
    else { break; }
  }while(ss);

  printf("Current type is %d\n",type);
  return(type);

}

void Socket::exec_remoteCMD()
{
  //printf("In exec_remoteCMD func\n");

  switch(get_cmd())
 {
        
  case UPDATE_REMOTE:
    //set rcinput values values
    printf("Updating...\n");
    //parser.parse(data,Control.thr,Control.Ixx, Control.Beta, Control.MaxOutput);
    //parser.parse(data, Control.thr, Control.Rollset, pid_roll.m_Kp, pid_roll.m_Ki, pid_roll.m_Kd, pid_rollRate.m_Kp, pid_rollRate.m_Ki, pid_rollRate.m_Kd);
    parser.parse(data, Control.thr, Control.Rollset, pid_roll.m_Kp, pid_roll.m_Ki, pid_roll.m_Kd, Control.k1, Control.k2, Control.k3);
  break;

  case INIT:
    //intialization of IMU
   if (!Control.started && !imu.initialized)
   {
      printf("Start Initilization...\n");
      imu.set_com();
      imu.initialize();
   }
   else
   {
      printf("Initilization has been done !\n");
   }

    //initilization of PID constants

    pid_roll.set_Kpid(3.5,0.1,1.7);
    //pid_rollRate.set_Kpid (2.9,0.1,0.125);
    printf("roll_p: %f  roll_i: %f  roll_d: %f\n",pid_roll.m_Kp,pid_roll.m_Ki,pid_roll.m_Kd);
    //printf("rollrate_p: %f  rollrate_i: %f  rollrate_d: %f\n", pid_rollRate.m_Kp,pid_rollRate.m_Ki,pid_rollRate.m_Kd);
    
    Control.k1 = 0.0082;
    Control.k2 = 0.2945;
    Control.k3 = 0.1204;
    
    printf("k1: %f, k2: %f, k3: %f\n",Control.k1,Control.k2,Control.k3);
    
    printf("Initilization finished !\n");

  break;


  case STOP_PID:
    //On exit :

    Control.stop_sign();

    //stop servos
    if (ESC.Is_open_blaster()){
      ESC.stopServo();
      ESC.close_blaster();
    }


   //reset PID
   // for (int i=0;i<DIM;i++) yprSTAB[i].reset();
   // for (int i=0;i<DIM;i++) yprRATE[i].reset();
   pid_roll.reset();
   //pid_rollRate.reset();
   

    printf("PID Stopped \n");
    

  break;


  case START_PID:
    //Remote says "Start"
    

    if (Control.started)
    {
      //PID already running
      break;
    } 
    else if (!imu.initialized)
    {
      //IMU not initialized
      printf("DMP not Initalized\n Can't start...\n");
      break;
    }

    //Initializing ESCs
    printf("Initialization of ESC...\n");
    ESC.open_blaster();
    ESC.init();
    printf("                     ... DONE.\n");
    
    
    //Things are getting started !
    //launch the Alarm signal
     Control.start_sign();
   // pthread_create(&thread_RC, NULL, (void *)&Thread_func, NULL);
    start_thread();
    while(Control.started)
    {
       Control.Run_Control();

    }//end while


  }//end switch

  return;
}

void Socket::start_thread()
{
	pthread_create(&thread_RC, NULL, Socket::Thread_func_RC, this);
	pthread_create(&thrdID, NULL, Socket::Thrdfunc_recevfrom, this);
}

// UAV1 receive data from UAV2
void * Socket::Thrdfunc_recevfrom(void *ptr) 
{
	int n;
	printf("Thread func receiving...\n");
	while(1)
	{
		n = recvfrom(remote.sockID, (char*)&remote.recev_buf, sizeof(remote.recev_buf),0,(struct sockaddr*)&remote.input_addr,&remote.length);
		if(n<0)
		{
			printf("Receive error\n");
			exit(-1);
		}
		//printf("%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n",remote.recev_buf.yaw,remote.recev_buf.pitch,remote.recev_buf.roll,
		//remote.recev_buf.yaw_rate, remote.recev_buf.pitch_rate, remote.recev_buf.roll_rate);
	}
}

// UAV1 receive command from remote controller during running
void * Socket::Thread_func_RC(void *ptr) 
{
	int n = 0;
	int size_thr = sizeof(remote.data);
	if ( remote.m_socket == 0 )
    printf("Socket is closed in thread_func...");
    sockaddr_in from_thr;
    socklen_t fromLength_thr = sizeof( from_thr );
	while(1)
	{
		bzero(remote.data,256);
		
		n = recvfrom( remote.m_socket, remote.data, size_thr, 0, (sockaddr*)&from_thr, &fromLength_thr);
		if (n<0)
        {
           printf("recfrom error in thread_func \n");
        }
        
        if(remote.data[0]=='s')
		{
		   //On exit :
           Control.stop_sign();

           //stop servos
          if (ESC.Is_open_blaster())
          {
            ESC.stopServo();
            ESC.close_blaster();
          }

          //reset PID
          //for (int i=0;i<DIM;i++) yprSTAB[i].reset();
          //for (int i=0;i<DIM;i++) yprRATE[i].reset();
          pid_roll.reset();
          //pid_rollRate.reset();

           printf("Control Stopped \n");
           pthread_exit(0);
		}
		else
		{
			printf("Wrong command ! Type s to stop \n");
		}
		
    }
}



