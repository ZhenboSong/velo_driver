/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 *  @author Song Zhenbo
 */

#ifndef __POINT_TYPES_H
#define __POINT_TYPES_H

#include <pcl/point_types.h>

namespace velodyne
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;


  /** Euclidean Velodyne coordinate, including intensity ,dual and ring number. */
  struct PointXYZIDR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint8_t  dual;
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace velodyne


POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))


POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne::PointXYZIDR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint8_t,dual,dual)
                                  (uint16_t, ring, ring))

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H

