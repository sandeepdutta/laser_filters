/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, JSK (The University of Tokyo).
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
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef POINTCLOUD_PASSTHRU_FILTER_H
#define POINTCLOUD_PASSTHRU_FILTER_H
/**
\author Yohei Kakiuchi
@b ScanRangeFilter takes input scans and filters within indicated range.
**/

#include "filters/filter_base.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace laser_filters
{

class PointCloudPassThruFilter : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
{
public:

  bool configure()
  {
    RCLCPP_INFO(logging_interface_->get_logger(), "PointCloudPassThru : Configured");
    return true;
  }

  virtual ~PointCloudPassThruFilter()
  {

  }

  bool update(const sensor_msgs::msg::PointCloud2& input_cloud, sensor_msgs::msg::PointCloud2& output_cloud)
  {
    output_cloud = input_cloud;
    return true;
  }
} ;

}

#endif // POINTCLOUD_PASSTHRU_FILTER_H