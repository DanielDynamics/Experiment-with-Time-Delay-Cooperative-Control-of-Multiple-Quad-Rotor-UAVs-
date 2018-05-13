//
//  RC_Client.c
//  
//
//  Created by Zhentao Xie on 3/8/17.
//
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#define MSG_SIZE 256			// message size

int main(int argc, char *argv[])
{
    int sock, n;
    unsigned int length;
    struct sockaddr_in anybody, from;
    //struct hostent *hp;
    char buffer[MSG_SIZE];
    int boolval = 1;			// for a socket option
    
    if (argc != 2)
    {
        printf("usage %s hostname port\n", argv[0]);
        exit(1);
    }
    
    sock = socket(AF_INET, SOCK_DGRAM, 0); // Creates socket. Connectionless.
    if (sock < 0)
        printf("socket error\n");
    
    // change socket permissions to allow broadcast
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &boolval, sizeof(boolval)) < 0)
    {
        printf("error setting socket options\n");
        exit(-1);
    }
    
    anybody.sin_family = AF_INET;		// symbol constant for Internet domain
    anybody.sin_port = htons(atoi(argv[1]));				// port field
    anybody.sin_addr.s_addr = inet_addr("192.168.1.255");	// broadcast address
    
    length = sizeof(struct sockaddr_in);		// size of structure
    
    do
    {
        // bzero: to "clean up" the buffer. The messages aren't always the same length...
        bzero(buffer,MSG_SIZE);		// sets all values to zero. memset() could be used
        printf("Please enter the message (! to exit): ");
        fgets(buffer,MSG_SIZE-1,stdin); // MSG_SIZE-1 because a null character is added
        
        if (buffer[0] != '!')
        {
            // send message to anyone there...
            n = sendto(sock, buffer, strlen(buffer), 0,
                       (const struct sockaddr *)&anybody,length);
            if (n < 0)
                printf("Sendto error\n");
            
            // receive message
        /*
            n = recvfrom(sock, buffer, MSG_SIZE, 0, (struct sockaddr *)&from, &length);
            if (n < 0)
                printf("recvfrom error\n");
            
            printf("Received something: %s\n", buffer);
         */
         
        }
    } while (buffer[0] != '!');
    
    close(sock);
    return 0;
    
    
}




