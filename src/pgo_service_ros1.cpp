#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pgo/Backend.hpp"
#include "backend_optimization/BackendOpt.h"
FILE *location_log = nullptr;
Backend backend;

bool pgo_callback(backend_optimization::BackendOptRequest &request, backend_optimization::BackendOptResponse &response)
{
    PointXYZIRPYT this_pose6d;
    PointCloudType::Ptr feats_undistort(new PointCloudType());
    PointCloudType::Ptr submap_fix(new PointCloudType());

    this_pose6d.x = request.pose[0];
    this_pose6d.y = request.pose[1];
    this_pose6d.z = request.pose[2];
    this_pose6d.roll = request.pose[3];
    this_pose6d.pitch = request.pose[4];
    this_pose6d.yaw = request.pose[5];
    this_pose6d.time = request.pose[6];

    pcl::fromROSMsg(request.cloud_undistort, *feats_undistort);
    backend.run(this_pose6d, feats_undistort, submap_fix);
    if (submap_fix->size())
    {
        response.pose_fix.emplace_back(this_pose6d.x);
        response.pose_fix.emplace_back(this_pose6d.y);
        response.pose_fix.emplace_back(this_pose6d.z);
        response.pose_fix.emplace_back(this_pose6d.roll);
        response.pose_fix.emplace_back(this_pose6d.pitch);
        response.pose_fix.emplace_back(this_pose6d.yaw);
        response.pose_fix.emplace_back(this_pose6d.time);
        pcl::toROSMsg(*submap_fix, response.submap_fix);
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pgo_service");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("pgo_service", pgo_callback);
    ROS_INFO("pgo service ready!");
    return 0;
}
