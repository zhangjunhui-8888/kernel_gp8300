#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
//#include <libsocketcan.h>
#include <getopt.h>
#include <libgen.h>
#include <signal.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>
#include <stdint.h>
//#include <can_config.h>


#ifndef PF_CAN
#define PF_CAN 29
#endif
#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif
#define EXTENDED 1
#define RTR 1
#define BUF_SIZ 255



void *thread_sendmessage(void* arg)
{
	int s = *(int*)arg;
	int ret,loopcount,send_lens;
	char *interface = "can0";
	struct ifreq ifr;
	struct sockaddr_can addr;
	int family = PF_CAN;
	char send_buff[10];

	struct can_frame frame;

  printf("send message----\n");

	strncpy(ifr.ifr_name, interface, sizeof(ifr.ifr_name));
	if (ioctl(s, SIOCGIFINDEX, &ifr)) {
		perror("ioctl");
	}

	addr.can_family = family;
	addr.can_ifindex = ifr.ifr_ifindex;

  while(1)
  {
	  printf("please input data:\n");
	  fgets(send_buff,sizeof(send_buff),stdin);
	  send_buff[strlen(send_buff)-1]=0;
	  int send_lenp = sprintf(frame.data,"%s",send_buff);
    printf("send_lenp is %d\n",send_lenp);

	  frame.can_dlc = send_lenp;
    //frame.can_dlc = 2;
		frame.can_id = 0x7dd;
    //frame.data = 0x5756;
		//frame.can_id &= CAN_EFF_MASK;
		//frame.can_id |= CAN_EFF_FLAG;

		// if (RTR)
		// frame.can_id |= CAN_RTR_FLAG;


		send_lens= sendto(s,&frame,sizeof(struct can_frame),0,(struct sockaddr*)&addr,sizeof(addr));
		if(send_lens<0){
			printf("data send failed!");
		}
  }

	close(s);
}


void *thread_recvmessage(void *arg)
{
	int s = *(int*)arg;
	struct sockaddr_can addr;
	struct can_frame frame;
	char buf[BUF_SIZ];
	int n = 0,len;
	int nbytes, i;


	while (1)
	{
		nbytes=recvfrom(s,&frame,sizeof(struct can_frame),0,(struct sockaddr *)&addr,&len);

		if (frame.can_id & CAN_EFF_FLAG)
		n = snprintf(buf, BUF_SIZ, "<0x%08x> ", frame.can_id & CAN_EFF_MASK);
		else
		n = snprintf(buf, BUF_SIZ, "<0x%03x> ", frame.can_id & CAN_SFF_MASK);

		n += snprintf(buf + n, BUF_SIZ - n, "[%d] ", frame.can_dlc);

		for (i = 0; i < frame.can_dlc; i++) {
		n += snprintf(buf + n, BUF_SIZ - n, "%02x ", frame.data[i]);
		}

		if (frame.can_id & CAN_RTR_FLAG)
		n += snprintf(buf + n, BUF_SIZ - n, "remote request");

		fprintf(stdout, "recvmessage：%s\n", buf);
	}

}


int main(int argc, char *argv[])
{
	static struct can_filter *filter = NULL;
	static int filter_count = 0;
	struct ifreq ifr;
	int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
	struct sockaddr_can addr;
	char *interface = "can0";
	uint32_t id, mask;
	int s;
  pthread_t tid1,tid2;

	if ((s = socket(family, type, proto)) < 0) {
	perror("socket creat failed");
	return -1;
	}

	strncpy(ifr.ifr_name, interface, sizeof(ifr.ifr_name));
	if (ioctl(s, SIOCGIFINDEX, &ifr)) {
		perror("ioctl");
		return -1;
	}

	addr.can_family = family;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
	perror("bind failed");
	return -1;
	}

	if (filter) {
		if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filter, filter_count * sizeof(struct can_filter)) != 0) {
			perror("setsockopt failed");
			exit(1);
		}
	}

	pthread_create(&tid1,NULL,thread_sendmessage,(void*)&s);
	pthread_join(tid1,NULL);
  pthread_create(&tid2,NULL,thread_recvmessage,(void*)&s);
	pthread_join(tid2,NULL);
}
