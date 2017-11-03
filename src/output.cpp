#include <ros/ros.h>
#include <pthread.h>
#include <sys/signal.h>  
#include "velo_driver/input.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velo_output");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  pthread_t tid;

  // start the driver
  velodyne::InputSocket dvr(node, private_nh,2368);

  int crt_ret = pthread_create(&tid,NULL,dvr.getThread,&dvr);
  // ROS_INFO("get get get");
  // loop until shut down or end of file
  while(ros::ok() && dvr.getSignal())
  {
      dvr.resolvePacket();
      ros::spinOnce();
  }
  int kill_ret = pthread_kill(tid,0);
  return 0;
}