/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  PassThruAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef LASER_SCAN_PASSTHRU_FILTER_H
#define LASER_SCAN_PASSTHRU_FILTER_H
/**
\author Tully Foote
@b ScanPassThruFilter takes input scans and corrects for PassThru angle assuming a flat target.
This is useful for ground plane extraction

**/

#include "filters/filter_base.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // PointCloud2ConstIterator
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "laser_geometry/laser_geometry.hpp"

namespace laser_filters
{

class LaserScanPassThruFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>, public rclcpp_lifecycle::LifecycleNode
{
public:
  LaserScanPassThruFilter()
      : rclcpp_lifecycle::LifecycleNode("laser_scan_PassThru_filter"),  up_and_running_(false) {}

  bool configure()
  {
    return true;
  }

  virtual ~LaserScanPassThruFilter()
  {
  }

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan ;

    up_and_running_ = true;
    return true;
  }


private:
  bool up_and_running_;
} ;

}

#endif // LASER_SCAN_PassThru_FILTER_H
