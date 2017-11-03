#include <string>
#include <sstream>
#include <fstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <pthread.h>

#include <angles/angles.h>
#include <math.h>

#include <ros/package.h>

#include "velo_driver/input.h"

namespace velodyne
{
  static pthread_mutex_t pack_lock = PTHREAD_MUTEX_INITIALIZER;
  static pthread_cond_t pack_new_frame = PTHREAD_COND_INITIALIZER;

  ////////////////////////////////////////////////////////////////////////
  // InputSocket class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   */
  InputSocket::InputSocket(ros::NodeHandle nh,ros::NodeHandle private_nh, uint16_t port)
  {
     private_nh.param("device_ip", devip_str_, std::string(""));
    if (!devip_str_.empty())
    {
      ROS_INFO_STREAM("Only accepting packets from IP address: "
                      << devip_str_);
    }
    private_nh.param("mode", mode_str_, std::string("NO_DUAL"));
    if (!mode_str_.empty())
    {
      ROS_INFO_STREAM("Confirm the mode dual or not");
    }

    std::string pkgPath = ros::package::getPath("velo_driver");
    std::string calibPath = pkgPath + "/params/vlp16.yaml";
    private_nh.param("calibration", config_.calibration_file,calibPath);
    if (!config_.calibration_file.empty())
    {
      ROS_INFO_STREAM("correction angles: " << config_.calibration_file);

      calibration_.read(config_.calibration_file);
      if (!calibration_.initialized) 
      {
        ROS_ERROR_STREAM("Unable to open calibration file: " << 
            config_.calibration_file);
        return ;
      }

      ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");
    }

    private_nh.param("min_angle", config_.min_angle,0);
    private_nh.param("max_angle", config_.max_angle,36000);
    ROS_INFO_STREAM("Max Angle: " << config_.max_angle <<".");
    ROS_INFO_STREAM("Min Angle:" << config_.min_angle << ".");
    
    private_nh.param("trigger_angle", config_.trigger_angle,18000);
    ROS_INFO_STREAM("Trigger Angle: " << config_.trigger_angle <<".");
    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) 
    {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
    
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    sockfd_ = -1;
    
    if (!devip_str_.empty())
    {
      inet_aton(devip_str_.c_str(),&devip_);
    }    

    // connect to Velodyne UDP port
    ROS_INFO_STREAM("Opening UDP socket: port " << port);
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1)
    {
      perror("socket");               // TODO: ROS_ERROR errno
      return;
    }
  
    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(port);          // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP
  
    if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
    {
      perror("bind");                 // TODO: ROS_ERROR errno
      return;
    }
  
    if (fcntl(sockfd_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
    {
      perror("non-block");
      return;
    }

    ROS_INFO("Velodyne socket fd is %d\n", sockfd_);
  }

  /** @brief destructor */
  InputSocket::~InputSocket(void)
  {
    (void) close(sockfd_);
  }

  /** @brief Get one velodyne packet. */
  void InputSocket::getPacket()
  {
    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    //initialize the recv tran and cal pack
    pack_num = 0;
    frame_id = -1;
    memset(&recv_pack,0,sizeof(RawRound_t));
    memset(&tran_pack,0,sizeof(RawRound_t));
    int last_angle = 0;
    int this_angle = 0;
    while (true)
    {  
        // Unfortunately, the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.

        // poll() until input available
        do
        {
          int retval = poll(fds, 1, POLL_TIMEOUT);
          if (retval < 0)             // poll() error?
          {
            if (errno != EINTR)
              ROS_ERROR("poll() error: %s", strerror(errno));
            return;
          }
          if (retval == 0)            // poll() timeout?
          {
            ROS_WARN("Velodyne poll() timeout");
            return;
          }
          if ((fds[0].revents & POLLERR)
              || (fds[0].revents & POLLHUP)
              || (fds[0].revents & POLLNVAL)) // device error?
          {
            ROS_ERROR("poll() reports Velodyne error");
            return;
          }
        } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
         
        ssize_t nbytes = recvfrom(sockfd_, &recv_pack.packages[pack_num],
                                  PACKET_SIZE,  0,
                                  (sockaddr*) &sender_address,
                                  &sender_address_len);

        if (nbytes < 0)
        {
          if (errno != EWOULDBLOCK)
            {
              perror("recvfail");
              ROS_INFO("recvfail");
              return;
            }
        }
        else if ((size_t) nbytes == PACKET_SIZE)
        {
         
          // read successful
          // if(((recv_pack.packages[pack_num].blocks[0].rotation<=config_.trigger_angle)&&
          // (recv_pack.packages[pack_num].blocks[BLOCKS_PER_PACKET-1].rotation>=config_.trigger_angle))||
          // ((last_angle<=config_.trigger_angle)&&(this_angle>=config_.trigger_angle)))
          // if(pack_num>=74)
          if((last_angle<=config_.trigger_angle)&&(this_angle>=config_.trigger_angle))
          {
            recv_pack.pack_num = pack_num+1;
            recv_pack.frame_id = frame_id++;
            recv_pack.end_time = ros::Time::now();
            
            pthread_mutex_lock(&pack_lock);
            memcpy(&tran_pack,&recv_pack,sizeof(RawRound_t));
            pthread_cond_signal(&pack_new_frame);
            pthread_mutex_unlock(&pack_lock);
			      memset(&recv_pack,0,sizeof(RawRound_t));
            pack_num  = 0;
            recv_pack.begin_time = ros::Time::now();  
          }
          else
          {
            pack_num++;
            if(pack_num>=MAX_PAKAGES_PER_ROUND)
            {
              ROS_INFO("rpm is too slow to use this driver. At least 600rpm.");
              break;
            }
          }
        }
        ROS_DEBUG_STREAM("incomplete Velodyne packet read: "
                          << nbytes << " bytes");
    }

    return;
  }

void * InputSocket::getThread(void * args)
{
  InputSocket * pController = (InputSocket*) args;
  pController->getPacket();
  return 0;
}

  int InputSocket::getSignal()
  {
    memset(&cal_pack,0,sizeof(RawRound_t));
    pthread_mutex_lock(&pack_lock);
    pthread_cond_wait(&pack_new_frame,&pack_lock);
    memcpy(&cal_pack,&tran_pack,sizeof(RawRound_t));
    pthread_mutex_unlock(&pack_lock);
    memset(&tran_pack,0,sizeof(RawRound_t));
    return 1;
  }

  void InputSocket::resolvePacket()
  {
    if(mode_str_=="NO_DUAL")
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr  outMsg(new pcl::PointCloud<pcl::PointXYZI>);
      for(int i=0;i<cal_pack.pack_num;i++)
      {
        unpack(cal_pack.packages[i],*outMsg);
      }
      pcl_conversions::toPCL(cal_pack.begin_time,outMsg->header.stamp);
      sensor_msgs::PointCloud2 pcMsg;
      pcl::toROSMsg(*outMsg,pcMsg);
      pcMsg.header.frame_id = "velodyne";
      pc_pub.publish(pcMsg);
    }
    else
    {
      for(int i=0;i<tran_pack.pack_num;i++)
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr  outMsg(new pcl::PointCloud<pcl::PointXYZI>);
        for(int i=0;i<cal_pack.pack_num;i++)
        {
          unpackVLP16Dual(cal_pack.packages[i],*outMsg);
        }
        pcl_conversions::toPCL(cal_pack.begin_time,outMsg->header.stamp);
        sensor_msgs::PointCloud2 pcMsg;
        pcl::toROSMsg(*outMsg,pcMsg);
        outMsg->header.frame_id = "velodyne";
        pc_pub.publish(pcMsg);
      }
    }
    return;
  }

  /** @brief convert raw packet to point cloud
    *
    *  @param pkt raw packet to unpack
    *  @param pc shared pointer to point cloud (points are appended)
    */
  void InputSocket::unpack(const RawPacket_t &pkt,
                        pcl::PointCloud<pcl::PointXYZI> &pc)
  {
    
    /** special parsing for the VLP16 **/
    if (calibration_.num_lasers == 16)
    {
      unpackVLP16(pkt, pc);
      return;
    }
    
    const RawPacket_t *raw = &pkt;

    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation
      int bank_origin = 0;
      if (raw->blocks[i].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }

      for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
        
        float x, y, z;
        float intensity;
        uint8_t laser_number;       ///< hardware laser number

        laser_number = j + bank_origin;
        LaserCorrection &corrections = 
          calibration_.laser_corrections[laser_number];

        /** Position Calculation */

        union TwoBytes tmp;
        tmp.bytes[0] = raw->blocks[i].data[k];
        tmp.bytes[1] = raw->blocks[i].data[k+1];
        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((raw->blocks[i].rotation >= config_.min_angle 
              && raw->blocks[i].rotation <= config_.max_angle 
              && config_.min_angle < config_.max_angle)
              ||(config_.min_angle > config_.max_angle 
              && (raw->blocks[i].rotation <= config_.max_angle 
              || raw->blocks[i].rotation >= config_.min_angle))){
          float distance = tmp.uint * DISTANCE_RESOLUTION;
          distance += corrections.dist_correction;

          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;

          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = 
            cos_rot_table_[raw->blocks[i].rotation] * cos_rot_correction + 
            sin_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;
          float sin_rot_angle = 
            sin_rot_table_[raw->blocks[i].rotation] * cos_rot_correction - 
            cos_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;

          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
            * was added to the expression due to the mathemathical
            * model we used.
            */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx=-xx;
          if (yy < 0) yy=-yy;
    
          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = 
              (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 2.4) / (25.04 - 2.4) 
              + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = 
              (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 1.93) / (25.04 - 1.93)
              + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
            * was added to the expression due to the mathemathical
            * model we used.
            */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
          ///the expression wiht '-' is proved to be better than the one with '+'
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
          /**the new term of 'vert_offset * sin_vert_angle'
            * was added to the expression due to the mathemathical
            * model we used.
            */
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
            * was added to the expression due to the mathemathical
            * model we used.
            */
          z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;

          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;

          /** Intensity Calculation */

          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[i].data[k+2];

          float focal_offset = 256 
                              * (1 - corrections.focal_distance / 13100) 
                              * (1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (abs(focal_offset - 256 * 
            (1 - static_cast<float>(tmp.uint)/65535)*(1 - static_cast<float>(tmp.uint)/65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          // convert polar coordinates to Euclidean XYZ
          pcl::PointXYZI point;
          point.x = x_coord;
          point.y = y_coord;
          point.z = z_coord;
          point.intensity = intensity;

          // append this point to the cloud
          pc.push_back(point);
          //++pc.width;
          
        }
      }
    }
  }

  /** @brief convert raw VLP16 packet to point cloud
    *
    *  @param pkt raw packet to unpack
    *  @param pc shared pointer to point cloud (points are appended)
    */
  void InputSocket::unpackVLP16(const RawPacket_t &pkt,
                             pcl::PointCloud<pcl::PointXYZI> &pc)
  {
    float azimuth;
    float azimuth_diff;
    float last_azimuth_diff=0;
    float azimuth_corrected_f;
    int azimuth_corrected;
    float x, y, z;
    float intensity;

    const RawPacket_t *raw = &pkt;

    for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                  << block << " header value is "
                                  << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      azimuth = (float)(raw->blocks[block].rotation);
      if (block < (BLOCKS_PER_PACKET-1)){
        azimuth_diff = (float)((36000 + raw->blocks[block+1].rotation - raw->blocks[block].rotation)%36000);
        last_azimuth_diff = azimuth_diff;
      }else{
        azimuth_diff = last_azimuth_diff;
      }

      for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
        for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
          LaserCorrection &corrections = calibration_.laser_corrections[dsr];

          /** Position Calculation */
          union TwoBytes tmp;
          tmp.bytes[0] = raw->blocks[block].data[k];
          tmp.bytes[1] = raw->blocks[block].data[k+1];
          
          /** correct for the laser rotation as a function of timing during the firings **/
          azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
          azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
          
          /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
          if ((azimuth_corrected >= config_.min_angle 
                && azimuth_corrected <= config_.max_angle 
                && config_.min_angle < config_.max_angle)
                ||(config_.min_angle > config_.max_angle 
                && (azimuth_corrected <= config_.max_angle 
                || azimuth_corrected >= config_.min_angle))){

            // convert polar coordinates to Euclidean XYZ
            float distance = tmp.uint * DISTANCE_RESOLUTION;
            distance += corrections.dist_correction;
            
            float cos_vert_angle = corrections.cos_vert_correction;
            float sin_vert_angle = corrections.sin_vert_correction;
            float cos_rot_correction = corrections.cos_rot_correction;
            float sin_rot_correction = corrections.sin_rot_correction;
    
            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            float cos_rot_angle = 
              cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            float sin_rot_angle = 
              sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;
    
            float horiz_offset = corrections.horiz_offset_correction;
            float vert_offset = corrections.vert_offset_correction;
    
            // Compute the distance in the xy plane (w/o accounting for rotation)
            /**the new term of 'vert_offset * sin_vert_angle'
              * was added to the expression due to the mathemathical
              * model we used.
              */
            float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
    
            // Calculate temporal X, use absolute value.
            float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
            // Calculate temporal Y, use absolute value
            float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
            if (xx < 0) xx=-xx;
            if (yy < 0) yy=-yy;
      
            // Get 2points calibration values,Linear interpolation to get distance
            // correction for X and Y, that means distance correction use
            // different value at different distance
            float distance_corr_x = 0;
            float distance_corr_y = 0;
            if (corrections.two_pt_correction_available) {
              distance_corr_x = 
                (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4) 
                + corrections.dist_correction_x;
              distance_corr_x -= corrections.dist_correction;
              distance_corr_y = 
                (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
              distance_corr_y -= corrections.dist_correction;
            }
    
            float distance_x = distance + distance_corr_x;
            /**the new term of 'vert_offset * sin_vert_angle'
              * was added to the expression due to the mathemathical
              * model we used.
              */
            xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
            x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    
            float distance_y = distance + distance_corr_y;
            /**the new term of 'vert_offset * sin_vert_angle'
              * was added to the expression due to the mathemathical
              * model we used.
              */
            xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
            y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    
            // Using distance_y is not symmetric, but the velodyne manual
            // does this.
            /**the new term of 'vert_offset * cos_vert_angle'
              * was added to the expression due to the mathemathical
              * model we used.
              */
            z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;

    
            /** Use standard ROS coordinate system (right-hand rule) */
            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;
    
            /** Intensity Calculation */
            float min_intensity = corrections.min_intensity;
            float max_intensity = corrections.max_intensity;
    
            intensity = raw->blocks[block].data[k+2];
    
            float focal_offset = 256 
                                * (1 - corrections.focal_distance / 13100) 
                                * (1 - corrections.focal_distance / 13100);
            float focal_slope = corrections.focal_slope;
            intensity += focal_slope * (abs(focal_offset - 256 * 
              (1 - tmp.uint/65535)*(1 - tmp.uint/65535)));
            intensity = (intensity < min_intensity) ? min_intensity : intensity;
            intensity = (intensity > max_intensity) ? max_intensity : intensity;

            // append this point to the cloud
            pcl::PointXYZI point;
            point.x = x_coord;
            point.y = y_coord;
            point.z = z_coord;
            point.intensity = intensity;

            pc.push_back(point);
            // ++pc.width;
          }
        }
      }
    }
  }  

  /** @brief convert raw VLP16 packet to point cloud when VLP16 runs in dual mode
    *
    *  @param pkt raw packet to unpack
    *  @param pc shared pointer to point cloud (points are appended)
    *  NOT ONLINE NOW
    */
  void InputSocket::unpackVLP16Dual(const RawPacket_t &pkt, pcl::PointCloud<pcl::PointXYZI> &pc)
  {
    float azimuth;
    float azimuth_diff;
    float last_azimuth_diff=0;
    float azimuth_corrected_f;
    int azimuth_corrected;
    float x, y, z;
    float intensity;

    const RawPacket_t *raw = &pkt;

    for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                  << block << " header value is "
                                  << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      azimuth = (float)(raw->blocks[block].rotation);
      if (block < (BLOCKS_PER_PACKET-1)){
        azimuth_diff = (float)((36000 + raw->blocks[block+1].rotation - raw->blocks[block].rotation)%36000);
        last_azimuth_diff = azimuth_diff;
      }else{
        azimuth_diff = last_azimuth_diff;
      }

      for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
        for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
          LaserCorrection &corrections = calibration_.laser_corrections[dsr];

          /** Position Calculation */
          union TwoBytes tmp;
          tmp.bytes[0] = raw->blocks[block].data[k];
          tmp.bytes[1] = raw->blocks[block].data[k+1];
          
          /** correct for the laser rotation as a function of timing during the firings **/
          azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
          azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
          
          /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
          if ((azimuth_corrected >= config_.min_angle 
                && azimuth_corrected <= config_.max_angle 
                && config_.min_angle < config_.max_angle)
                ||(config_.min_angle > config_.max_angle 
                && (azimuth_corrected <= config_.max_angle 
                || azimuth_corrected >= config_.min_angle))){

            // convert polar coordinates to Euclidean XYZ
            float distance = tmp.uint * DISTANCE_RESOLUTION;
            distance += corrections.dist_correction;
            
            float cos_vert_angle = corrections.cos_vert_correction;
            float sin_vert_angle = corrections.sin_vert_correction;
            float cos_rot_correction = corrections.cos_rot_correction;
            float sin_rot_correction = corrections.sin_rot_correction;
    
            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            float cos_rot_angle = 
              cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            float sin_rot_angle = 
              sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;
    
            float horiz_offset = corrections.horiz_offset_correction;
            float vert_offset = corrections.vert_offset_correction;
    
            // Compute the distance in the xy plane (w/o accounting for rotation)
            /**the new term of 'vert_offset * sin_vert_angle'
              * was added to the expression due to the mathemathical
              * model we used.
              */
            float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
    
            // Calculate temporal X, use absolute value.
            float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
            // Calculate temporal Y, use absolute value
            float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
            if (xx < 0) xx=-xx;
            if (yy < 0) yy=-yy;
      
            // Get 2points calibration values,Linear interpolation to get distance
            // correction for X and Y, that means distance correction use
            // different value at different distance
            float distance_corr_x = 0;
            float distance_corr_y = 0;
            if (corrections.two_pt_correction_available) {
              distance_corr_x = 
                (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4) 
                + corrections.dist_correction_x;
              distance_corr_x -= corrections.dist_correction;
              distance_corr_y = 
                (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
              distance_corr_y -= corrections.dist_correction;
            }
    
            float distance_x = distance + distance_corr_x;
            /**the new term of 'vert_offset * sin_vert_angle'
              * was added to the expression due to the mathemathical
              * model we used.
              */
            xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
            x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    
            float distance_y = distance + distance_corr_y;
            /**the new term of 'vert_offset * sin_vert_angle'
              * was added to the expression due to the mathemathical
              * model we used.
              */
            xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
            y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    
            // Using distance_y is not symmetric, but the velodyne manual
            // does this.
            /**the new term of 'vert_offset * cos_vert_angle'
              * was added to the expression due to the mathemathical
              * model we used.
              */
            z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;

    
            /** Use standard ROS coordinate system (right-hand rule) */
            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;
    
            /** Intensity Calculation */
            float min_intensity = corrections.min_intensity;
            float max_intensity = corrections.max_intensity;
    
            intensity = raw->blocks[block].data[k+2];
    
            float focal_offset = 256 
                                * (1 - corrections.focal_distance / 13100) 
                                * (1 - corrections.focal_distance / 13100);
            float focal_slope = corrections.focal_slope;
            intensity += focal_slope * (abs(focal_offset - 256 * 
              (1 - tmp.uint/65535)*(1 - tmp.uint/65535)));
            intensity = (intensity < min_intensity) ? min_intensity : intensity;
            intensity = (intensity > max_intensity) ? max_intensity : intensity;
    
            // append this point to the cloud
            pcl::PointXYZI point;
            point.x = x_coord;
            point.y = y_coord;
            point.z = z_coord;
            point.intensity = intensity;

            pc.push_back(point);
            // ++pc.width;
          }
        }
      }
    }
  }

} // velodyne namespace
