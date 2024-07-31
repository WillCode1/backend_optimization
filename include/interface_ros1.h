#pragma once
#include "pgo/Backend.hpp"

void load_ros_parameters(bool &path_en, bool &scan_pub_en, bool &dense_pub_en,
                         std::string &gnss_topic, std::string &map_frame, std::string &lidar_frame);
void load_parameters(Backend &backend);
void load_pgm_parameters(bool &save_globalmap_en, bool &save_pgm, double &pgm_resolution, float &min_z, float &max_z);

void init_pgo_system(ros::NodeHandle &nh);
void pgo_handle(PointXYZIRPYT &this_pose6d, PointCloudType::Ptr &feats_undistort, PointCloudType::Ptr &submap_fix);
