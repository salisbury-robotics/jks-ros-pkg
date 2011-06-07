#include <ros/ros.h>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
#include <stdlib.h>             //bzero
#include <netinet/in.h> //sockaddr
#include <netdb.h>              //gethostbyname
#include <unistd.h>             //read

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop");
  int hSock = socket(AF_INET, SOCK_DGRAM, 0);
  if (hSock <=-1)
    return 1;
  struct hostent *pServer = gethostbyname("localhost");
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));

  addr.sin_family = AF_INET;
  memcpy(&addr.sin_addr.s_addr, pServer->h_addr, pServer->h_length);
  addr.sin_port = htons(50000);
  
 
  char c;
  int kfd = 0;
  struct termios cooked, raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;
  while(ros::ok())
  {
    boost::this_thread::interruption_point();
    
    // get the next event from the keyboard
    int num;
    if((num = poll(&ufd, 1, 250)) < 0)
    {
      perror("poll():");
      return 1;
    }
    else if(num > 0)
    {
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        return 1;
      }
    }
    else
      continue;
    char data[]="a";
    data[0]=c;
    int n = sendto(hSock, data, strlen(data), 0, (sockaddr*)&addr, sizeof(addr));
    if (n < 0){
      fprintf(stderr, "Error, send() failed: %s\n", strerror(errno));
      shutdown(hSock, SHUT_RDWR);
      close(hSock);
      return 1;
    }
  }
  shutdown(hSock, SHUT_RDWR);
  close(hSock);
  tcsetattr(kfd, TCSANOW, &cooked);
  return 0;
}
