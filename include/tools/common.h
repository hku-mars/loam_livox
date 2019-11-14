// Author: Jiarong Lin          ziv.lin.ljr@gmail.com

#pragma once

#include <cmath>

#include <pcl/point_types.h>
#define printf_line printf( " %s %d \r\n", __FILE__, __LINE__ );
typedef pcl::PointXYZI PointType;

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
