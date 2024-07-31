#pragma once
#include <rclcpp/rclcpp.hpp>
#include "Header.h"

void init_pgo_system(rclcpp::Node::SharedPtr &node);

void pgo_handle(PointXYZIRPYT &this_pose6d, PointCloudType::Ptr &feats_undistort, PointCloudType::Ptr &submap_fix);
