/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *  Copyright (C) 2015, Song Zhenbo  modified input&driver&rawdata class into one class
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  Velodyne 3D LIDAR data input classes
 *
 *    These classes provide raw Velodyne LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     velodyne::InputBase -- base class for accessing the data
 *                      independently of its source
 *
 *     velodyne::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     velodyne::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 *     song: output a 360-range-frame velodyne pointcloud  with ring num for each point
 */

#ifndef __INPUT_H
#define __INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "velo_driver/calibration.h"
// #include "velo_driver/point_types.h"

namespace velodyne
{
  static uint16_t DATA_PORT_NUMBER = 2368;     // default data port
  static uint16_t POSITION_PORT_NUMBER = 8308; // default position port

  /**
   * Raw Velodyne packet constants and structures.
   */
  static const int SIZE_BLOCK = 100;
  static const int RAW_SCAN_SIZE = 3;
  static const int SCANS_PER_BLOCK = 32;
  static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

  static const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
  static const uint16_t ROTATION_MAX_UNITS    = 36000u;     // [deg/100]
  static const float DISTANCE_RESOLUTION      =     0.002f; // [m]

  /** @todo make this work for both big and little-endian machines */
  static const uint16_t UPPER_BANK = 0xeeff;
  static const uint16_t LOWER_BANK = 0xddff;
  
  
  /** Special Defines for VLP16 support **/
  static const int    VLP16_FIRINGS_PER_BLOCK =   2;
  static const int    VLP16_SCANS_PER_FIRING  =  16;
  static const float  VLP16_BLOCK_TDURATION   = 110.592f;   // [µs]
  static const float  VLP16_DSR_TOFFSET       =   2.304f;   // [µs]
  static const float  VLP16_FIRING_TOFFSET    =  55.296f;   // [µs]
  

  /** \brief Raw Velodyne data block.
   *
   *  Each block contains data from either the upper or lower laser
   *  bank.  The device returns three times as many upper bank blocks.
   *
   *  use stdint.h types, so things work with both 64 and 32-bit machines
   */
  typedef struct RawBlock
  {
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
    uint8_t  data[BLOCK_DATA_SIZE];
  } RawBlock_t;

  /** used for unpacking the first two data bytes in a block
   *
   *  They are packed into the actual data stream misaligned.  I doubt
   *  this works on big endian machines.
   */
  union TwoBytes
  {
    uint16_t uint;
    uint8_t  bytes[2];
  };

  static const int PACKET_SIZE = 1206;
  static const int BLOCKS_PER_PACKET = 12;
  static const int PACKET_STATUS_SIZE = 4;
  static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
  static const int MAX_PAKAGES_PER_ROUND = 512;

  /** \brief Raw Velodyne packet.
   *
   *  revolution is described in the device manual as incrementing
   *    (mod 65536) for each physical turn of the device.  Our device
   *    seems to alternate between two different values every third
   *    packet.  One value increases, the other decreases.
   *
   *  \todo figure out if revolution is only present for one of the
   *  two types of status fields
   *
   *  status has either a temperature encoding or the microcode level
   */
  typedef struct RawPacket
  {
    RawBlock_t blocks[BLOCKS_PER_PACKET];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE]; 
  } RawPacket_t;

  typedef struct RawRound
  {
    ros::Time begin_time;
    ros::Time end_time;
    int    frame_id;
    int    pack_num;
    RawPacket_t packages[MAX_PAKAGES_PER_ROUND];
  }RawRound_t;

   /** configuration parameters */
  typedef struct Config
  {
     std::string calibration_file;     ///< calibration file name
     int min_angle;                   ///< minimum angle to publish
     int max_angle;                   ///< maximum angle to publish
     int trigger_angle;               ///angle to trigger camera or one entry frame. NOT AROUND 0!
   }Config_t;

  // Shorthand typedefs for point cloud representations
  // typedef PointXYZIR VPoint;
  // typedef PointXYZIDR VDPoint;
  // typedef pcl::PointCloud<VPoint> VPointCloud;
  // typedef pcl::PointCloud<VDPoint> VDPointCloud;

  /** @brief Live Velodyne input from socket. */
  class InputSocket
  {
  public:
    InputSocket(ros::NodeHandle nh,
                ros::NodeHandle private_nh,
                uint16_t port = DATA_PORT_NUMBER);
     ~InputSocket();

    
    static void * getThread(void * args);
    int getSignal();
    void resolvePacket();

  private:
    int sockfd_;
    in_addr devip_;
     
    uint16_t port_;
    std::string devip_str_;
    std::string mode_str_;
  
    Config_t config_;
    
    /*  save for packages in one frame */
    int pack_num;
    int frame_id;
    RawRound_t recv_pack;
    RawRound_t tran_pack;
    RawRound_t cal_pack;

    ros::NodeHandle private_nh_;
    ros::Publisher pc_pub; 

  private:

    /** 
     * Calibration file
    */
    velodyne::Calibration calibration_;
    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];

    void getPacket();
    /** add private function to unpack packages **/ 
    void unpack(const RawPacket_t &pkt, pcl::PointCloud<pcl::PointXYZI> &pc);
    void unpackVLP16(const RawPacket_t &pkt, pcl::PointCloud<pcl::PointXYZI> &pc);
    void unpackVLP16Dual(const RawPacket_t &pkt, pcl::PointCloud<pcl::PointXYZI> &pc);
    
  };

} // velodyne namespace

#endif // __INPUT_H
