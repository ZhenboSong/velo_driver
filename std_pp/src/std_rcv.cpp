#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <malloc.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>


typedef uint8_t		u8_t;
typedef int8_t		i8_t;
typedef uint16_t	u16_t;
typedef int16_t		i16_t;
typedef uint32_t	u32_t;
typedef int32_t		i32_t;
typedef uint64_t	u64_t;
typedef int64_t		i64_t;
typedef float		f32_t;
typedef double		f64_t;

typedef struct
{
	u64_t sn;				//报文编号
	u8_t ins_status;		//惯导状态:0-非正常状态,1-正常状态
	u8_t gps_status;		//卫星状态:0-定位不佳,1-一般状态，2-RTK差分状态,3-信标定位
	u8_t svs_num;			//导航用卫星数量
	u8_t pdop;				//导航位置精度强弱度,值越小精度越高
	u32_t nav_id;			//惯性导航报文戳
	f64_t lon_deg;			//经度,单位度
	f64_t lat_deg;			//纬度,单位度
	f64_t alt_m;			//海拔,单位米
	f64_t north_cm;			//大地坐标北向偏移,单位厘米
	f64_t east_cm;			//大地坐标东向偏移,单位厘米
	f64_t z_cm;				//大地坐标高度，单位厘米
	f32_t yaw_rad;			//航向角,单位弧度
	f32_t roll_rad;			//滚转角,单位弧度
	f32_t pitch_rad;		//俯仰角,单位弧度
	f32_t spd_cm_s;			//车辆速度,单位cm/s
	f32_t spd_l_cm_s;		//左轮速度,单位cm/s
	f32_t spd_r_cm_s;		//右轮速度,单位cm/s
} PP_DATA;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
const char inIP[200] = "192.168.3.33";
const unsigned int   port = 50001;
static int m_sockReceive;
static struct sockaddr_in  addr;
static socklen_t  addr_cli_len;

static FILE * fp;
static unsigned int max_frame_num; 

static char pData[1024];

void chatterCallback(const std_msgs::UInt32::ConstPtr& msg)
{
	memset(pData,0,1024);
	if(msg->data>=max_frame_num)
	{
		ROS_INFO("this file only has %u frames",max_frame_num);
		return;
	}
	fseek(fp,msg->data*sizeof(PP_DATA),SEEK_SET);
	fread(pData,sizeof(PP_DATA),1,fp);
	int bytes = sendto(m_sockReceive, pData, sizeof(PP_DATA), 0, (struct sockaddr*)&addr,addr_cli_len);
    if(bytes==-1)
	{
		ROS_INFO("send failed frame %u",msg->data);
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "std_rcv");
	ros::NodeHandle n;
	// string s;
	// n.param<string>("file_path", s, "default_string");

	fp = fopen("/home/song/pp_data/for_run/pp_data.bin","rb");
	if(fp==NULL)
	{
		ROS_INFO("PP FILE read failed");
		return 0;
	}
	fseek (fp, 0, SEEK_END);   
	long fsize=ftell (fp); 
	max_frame_num = (unsigned int)(fsize / sizeof(PP_DATA));
	fseek (fp, 0, SEEK_SET);
	

	m_sockReceive = socket(AF_INET, SOCK_DGRAM, 0);
    if(m_sockReceive!=-1)
    	printf("proc create success\n");
	memset(&addr,0x00,sizeof(addr));
	addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = inet_addr(inIP);
    addr.sin_port        = htons(port);
    addr_cli_len = sizeof(addr);
	
	ros::Subscriber sub = n.subscribe("std_chatter", 1000, chatterCallback);

	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();	
	fclose(fp);
	close(m_sockReceive);
	return 0;
}