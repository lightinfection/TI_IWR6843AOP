#pragma once
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE

#include <pcl/memory.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct PointTI
{
  PCL_ADD_POINT4D;
  float intensity;
  float velocity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointTI,        
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, velocity, velocity));

struct PointTIRGB
{
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  float intensity;
  float velocity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointTIRGB,        
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, velocity, velocity)
                                   (uint32_t, rgb, rgb));

#endif