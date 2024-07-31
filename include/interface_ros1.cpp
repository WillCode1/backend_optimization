#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "pgo/Backend.hpp"
#include "interface_ros1.h"

FILE *location_log = nullptr;
bool showOptimizedPose = true;
double globalMapVisualizationSearchRadius = 1000;
double globalMapVisualizationPoseDensity = 10;
double globalMapVisualizationLeafSize = 1;
double lidar_end_time = 0;
bool path_en = true, scan_pub_en = false, dense_pub_en = false;
Backend backend;

ros::Publisher pubLaserCloudFull;
ros::Publisher pubOdomAftMapped;
ros::Publisher pubLidarPath;
ros::Publisher pubOdomNotFix;
ros::Publisher pubLoopConstraintEdge;
ros::Subscriber sub_gnss;
ros::Publisher pubGlobalmap;
std::thread visualizeMapThread;
ros::Subscriber sub_initpose;

std::string map_frame;
std::string lidar_frame;
std::string gnss_topic;
bool save_globalmap_en = false;
bool save_pgm = false;
double pgm_resolution;
float min_z, max_z;

bool flg_exit = false;
void SigHandle(int sig)
{
    flg_exit = true;
    LOG_WARN("catch sig %d", sig);
}

void gnss_cbk(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    backend.gnss->gnss_handler(GnssPose(msg->header.stamp.toSec(), V3D(msg->latitude, msg->longitude, msg->altitude)));
    backend.relocalization->gnss_pose = GnssPose(msg->header.stamp.toSec(), V3D(msg->latitude, msg->longitude, msg->altitude));
}

#ifdef UrbanLoco
void UrbanLoco_cbk(const nav_msgs::OdometryConstPtr &msg)
{
    backend.gnss->UrbanLoco_handler(GnssPose(msg->header.stamp.toSec(),
                                             V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
                                             QD(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
                                             V3D(msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35])));
    backend.relocalization->gnss_pose = GnssPose(msg->header.stamp.toSec(),
                                                 V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
                                                 QD(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));
}
#endif

void publish_cloud(const ros::Publisher &pubCloud, PointCloudType::Ptr cloud, const double& lidar_end_time, const std::string& frame_id)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time().fromSec(lidar_end_time);
    cloud_msg.header.frame_id = frame_id;
    pubCloud.publish(cloud_msg);
}

void publish_cloud_world(const ros::Publisher &pubLaserCloudFull, PointCloudType::Ptr laserCloud, const PointXYZIRPYT &pose, const double& lidar_end_time)
{
    PointCloudType::Ptr laserCloudWorld(new PointCloudType(laserCloud->size(), 1));
    pointcloudLidarToWorld(laserCloud, laserCloudWorld, pose);
    publish_cloud(pubLaserCloudFull, laserCloudWorld, lidar_end_time, map_frame);
}

// 设置输出的t,q，在publish_odometry，publish_path调用
template <typename T>
void set_posestamp(T &out, const QD &rot, const V3D &pos)
{
    out.pose.position.x = pos(0);
    out.pose.position.y = pos(1);
    out.pose.position.z = pos(2);
    out.pose.orientation.x = rot.coeffs()[0];
    out.pose.orientation.y = rot.coeffs()[1];
    out.pose.orientation.z = rot.coeffs()[2];
    out.pose.orientation.w = rot.coeffs()[3];
}

void publish_tf(const geometry_msgs::Pose &pose, const double& lidar_end_time)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // lidar -> map
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(lidar_end_time), map_frame, lidar_frame));
}

// 发布里程计
void publish_odometry(const ros::Publisher &pubOdomAftMapped, const PointXYZIRPYT &state, const double& lidar_end_time, bool need_publish_tf = true)
{
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = lidar_frame;
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    const QD &lidar_rot = EigenMath::RPY2Quaternion(V3D(state.roll, state.pitch, state.yaw));
    const V3D &lidar_pos = V3D(state.x, state.y, state.z);
    set_posestamp(odomAftMapped.pose, lidar_rot, lidar_pos);
    pubOdomAftMapped.publish(odomAftMapped);
    if (need_publish_tf)
        publish_tf(odomAftMapped.pose.pose, lidar_end_time);
}

void publish_lidar_keyframe_trajectory(const ros::Publisher &pubPath, const pcl::PointCloud<PointXYZIRPYT> &trajectory, const double &lidar_end_time)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id = map_frame;

    geometry_msgs::PoseStamped msg_lidar_pose;
    for (const auto &point : trajectory)
    {
        msg_lidar_pose.pose.position.x = point.x;
        msg_lidar_pose.pose.position.y = point.y;
        msg_lidar_pose.pose.position.z = point.z;
        auto quat = EigenMath::RPY2Quaternion(V3D(point.roll, point.pitch, point.yaw));
        msg_lidar_pose.pose.orientation.x = quat.x();
        msg_lidar_pose.pose.orientation.y = quat.y();
        msg_lidar_pose.pose.orientation.z = quat.z();
        msg_lidar_pose.pose.orientation.w = quat.w();

        msg_lidar_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
        msg_lidar_pose.header.frame_id = map_frame;

        path.poses.push_back(msg_lidar_pose);
    }

    pubPath.publish(path);
}

void visualize_loop_closure_constraints(const ros::Publisher &pubLoopConstraintEdge, const double &timestamp,
                                        const unordered_map<int, int> &loop_constraint_records,
                                        const pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d)
{
    if (loop_constraint_records.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = map_frame;
    markerNode.header.stamp = ros::Time().fromSec(timestamp);
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = map_frame;
    markerEdge.header.stamp = ros::Time().fromSec(timestamp);
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loop_constraint_records.begin(); it != loop_constraint_records.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::Point p;
        p.x = keyframe_pose6d->points[key_cur].x;
        p.y = keyframe_pose6d->points[key_cur].y;
        p.z = keyframe_pose6d->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = keyframe_pose6d->points[key_pre].x;
        p.y = keyframe_pose6d->points[key_pre].y;
        p.z = keyframe_pose6d->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
}

void visualize_globalmap_thread(const ros::Publisher &pubGlobalmap)
{
    while (!flg_exit)
    {
        this_thread::sleep_for(std::chrono::seconds(1));
        auto submap_visual = backend.get_submap_visual(globalMapVisualizationSearchRadius, globalMapVisualizationPoseDensity, globalMapVisualizationLeafSize, showOptimizedPose);
        if (submap_visual == nullptr)
            continue;
        publish_cloud(pubGlobalmap, submap_visual, lidar_end_time, map_frame);
    }
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    const geometry_msgs::Pose &pose = msg->pose.pose;
    const auto &ori = msg->pose.pose.orientation;
    Eigen::Quaterniond quat(ori.w, ori.x, ori.y, ori.z);
    auto rpy = EigenMath::Quaternion2RPY(quat);
    // prior pose in map(imu pose)
    Pose init_pose;
    init_pose.x = pose.position.x;
    init_pose.y = pose.position.y;
    init_pose.z = pose.position.z;
    init_pose.roll = rpy.x();
    init_pose.pitch = rpy.y();
    init_pose.yaw = rpy.z();
    backend.relocalization->set_init_pose(init_pose);
}

void pgo_callback(PointXYZIRPYT &this_pose6d, PointCloudType::Ptr &feats_undistort, PointCloudType::Ptr &submap_fix)
{
    publish_odometry(pubOdomAftMapped, this_pose6d, this_pose6d.time);

    backend.run(this_pose6d, feats_undistort, submap_fix);

    lidar_end_time = this_pose6d.time;

    /******* Publish odometry *******/
    publish_odometry(pubOdomNotFix, this_pose6d, this_pose6d.time, false);

    /******* Publish points *******/
    if (path_en)
    {
        publish_lidar_keyframe_trajectory(pubLidarPath, *backend.keyframe_pose6d_optimized, this_pose6d.time);
    }
    if (scan_pub_en)
        if (dense_pub_en)
            publish_cloud_world(pubLaserCloudFull, feats_undistort, this_pose6d, this_pose6d.time);
        // else
        //     publish_cloud(pubLaserCloudFull, frontend.feats_down_world, this_pose6d.time, map_frame);

    visualize_loop_closure_constraints(pubLoopConstraintEdge, this_pose6d.time, backend.loopClosure->loop_constraint_records, backend.loopClosure->copy_keyframe_pose6d);
}

void load_ros_parameters()
{
    ros::param::param("publish/path_en", path_en, false);
    ros::param::param("publish/scan_publish_en", scan_pub_en, false);
    ros::param::param("publish/dense_publish_en", dense_pub_en, false);

    ros::param::param("common/gnss_topic", gnss_topic, std::string("/gps/fix"));
    ros::param::param("common/map_frame", map_frame, std::string("camera_init"));
    ros::param::param("common/lidar_frame", lidar_frame, std::string("lidar"));
}

void load_parameters()
{
    vector<double> extrinT;
    vector<double> extrinR;
    V3D extrinT_eigen;
    M3D extrinR_eigen;

    ros::param::param("mapping/keyframe_add_dist_threshold", backend.backend->keyframe_add_dist_threshold, 1.f);
    ros::param::param("mapping/keyframe_add_angle_threshold", backend.backend->keyframe_add_angle_threshold, 0.2f);
    ros::param::param("mapping/pose_cov_threshold", backend.backend->pose_cov_threshold, 25.f);
    ros::param::param("mapping/gnssValidInterval", backend.gnss->gnssValidInterval, 0.2f);
    ros::param::param("mapping/gpsCovThreshold", backend.gnss->gpsCovThreshold, 2.f);
    ros::param::param("mapping/useGpsElevation", backend.gnss->useGpsElevation, false);

    ros::param::param("mapping/extrinsic_gnss_T", extrinT, vector<double>());
    ros::param::param("mapping/extrinsic_gnss_R", extrinR, vector<double>());
    extrinT_eigen << VEC_FROM_ARRAY(extrinT);
    extrinR_eigen << MAT_FROM_ARRAY(extrinR);
    backend.gnss->set_extrinsic(extrinT_eigen, extrinR_eigen);

    ros::param::param("mapping/recontruct_kdtree", backend.backend->recontruct_kdtree, true);
    ros::param::param("mapping/ikdtree_reconstruct_keyframe_num", backend.backend->ikdtree_reconstruct_keyframe_num, 10);
    ros::param::param("mapping/ikdtree_reconstruct_downsamp_size", backend.backend->ikdtree_reconstruct_downsamp_size, 0.1f);

    ros::param::param("mapping/loop_closure_enable_flag", backend.loop_closure_enable_flag, false);
    ros::param::param("mapping/loop_closure_interval", backend.loop_closure_interval, 1000);
    ros::param::param("mapping/loop_keyframe_num_thld", backend.loopClosure->loop_keyframe_num_thld, 50);
    ros::param::param("mapping/loop_closure_search_radius", backend.loopClosure->loop_closure_search_radius, 10.f);
    ros::param::param("mapping/loop_closure_keyframe_interval", backend.loopClosure->loop_closure_keyframe_interval, 30);
    ros::param::param("mapping/keyframe_search_num", backend.loopClosure->keyframe_search_num, 20);
    ros::param::param("mapping/loop_closure_fitness_score_thld", backend.loopClosure->loop_closure_fitness_score_thld, 0.05f);
    ros::param::param("mapping/icp_downsamp_size", backend.loopClosure->icp_downsamp_size, 0.1f);
    ros::param::param("mapping/manually_loop_vaild_period", backend.loopClosure->loop_vaild_period["manually"], vector<double>());
    ros::param::param("mapping/odom_loop_vaild_period", backend.loopClosure->loop_vaild_period["odom"], vector<double>());
    ros::param::param("mapping/scancontext_loop_vaild_period", backend.loopClosure->loop_vaild_period["scancontext"], vector<double>());

    ros::param::param("official/save_keyframe_en", backend.save_keyframe_en, true);
    ros::param::param("official/save_keyframe_descriptor_en", backend.save_keyframe_descriptor_en, true);
    ros::param::param("official/save_resolution", backend.save_resolution, 0.1f);
    ros::param::param("official/map_path", backend.map_path, std::string(""));
    if (backend.map_path.compare("") != 0)
    {
        backend.globalmap_path = backend.map_path + "/globalmap.pcd";
        backend.trajectory_path = backend.map_path + "/trajectory.pcd";
        backend.keyframe_path = backend.map_path + "/keyframe/";
        backend.scd_path = backend.map_path + "/scancontext/";
    }
    else
        backend.map_path = PCD_FILE_DIR("");

    ros::param::param("scan_context/lidar_height", backend.relocalization->sc_manager->LIDAR_HEIGHT, 2.0);
    ros::param::param("scan_context/sc_dist_thres", backend.relocalization->sc_manager->SC_DIST_THRES, 0.5);

    if (false)
    {
        ros::param::param("utm_origin/zone", backend.relocalization->utm_origin.zone, std::string("51N"));
        ros::param::param("utm_origin/east", backend.relocalization->utm_origin.east, 0.);
        ros::param::param("utm_origin/north", backend.relocalization->utm_origin.north, 0.);
        ros::param::param("utm_origin/up", backend.relocalization->utm_origin.up, 0.);

        ros::param::param("mapping/extrinsicT_imu2gnss", extrinT, vector<double>());
        ros::param::param("mapping/extrinsicR_imu2gnss", extrinR, vector<double>());
        extrinT_eigen << VEC_FROM_ARRAY(extrinT);
        extrinR_eigen << MAT_FROM_ARRAY(extrinR);
        backend.relocalization->set_extrinsic(extrinT_eigen, extrinR_eigen);

        ros::param::param("relocalization_cfg/algorithm_type", backend.relocalization->algorithm_type, std::string("UNKONW"));

        BnbOptions match_option;
        ros::param::param("bnb3d/linear_xy_window_size", match_option.linear_xy_window_size, 10.);
        ros::param::param("bnb3d/linear_z_window_size", match_option.linear_z_window_size, 1.);
        ros::param::param("bnb3d/angular_search_window", match_option.angular_search_window, 30.);
        ros::param::param("bnb3d/pc_resolutions", match_option.pc_resolutions, vector<double>());
        ros::param::param("bnb3d/bnb_depth", match_option.bnb_depth, 5);
        ros::param::param("bnb3d/min_score", match_option.min_score, 0.1);
        ros::param::param("bnb3d/enough_score", match_option.enough_score, 0.8);
        ros::param::param("bnb3d/min_xy_resolution", match_option.min_xy_resolution, 0.2);
        ros::param::param("bnb3d/min_z_resolution", match_option.min_z_resolution, 0.1);
        ros::param::param("bnb3d/min_angular_resolution", match_option.min_angular_resolution, 0.1);
        ros::param::param("bnb3d/filter_size_scan", match_option.filter_size_scan, 0.1);
        ros::param::param("bnb3d/debug_mode", match_option.debug_mode, false);

        ros::param::param("mapping/extrinsic_T", extrinT, vector<double>());
        ros::param::param("mapping/extrinsic_R", extrinR, vector<double>());
        extrinT_eigen << VEC_FROM_ARRAY(extrinT);
        extrinR_eigen << MAT_FROM_ARRAY(extrinR);
        V3D ext_rpy = EigenMath::RotationMatrix2RPY(extrinR_eigen);
        Pose lidar_extrinsic;
        lidar_extrinsic.x = extrinT_eigen.x();
        lidar_extrinsic.y = extrinT_eigen.y();
        lidar_extrinsic.z = extrinT_eigen.z();
        lidar_extrinsic.roll = ext_rpy.x();
        lidar_extrinsic.pitch = ext_rpy.y();
        lidar_extrinsic.yaw = ext_rpy.z();
        backend.relocalization->set_bnb3d_param(match_option, lidar_extrinsic);

        double step_size, resolution;
        ros::param::param("ndt/step_size", step_size, 0.1);
        ros::param::param("ndt/resolution", resolution, 1.);
        backend.relocalization->set_ndt_param(step_size, resolution);

        bool use_gicp;
        double gicp_downsample, filter_range, search_radius, teps, feps, fitness_score;
        ros::param::param("gicp/use_gicp", use_gicp, true);
        ros::param::param("gicp/filter_range", filter_range, 80.);
        ros::param::param("gicp/gicp_downsample", gicp_downsample, 0.2);
        ros::param::param("gicp/search_radius", search_radius, 0.5);
        ros::param::param("gicp/teps", teps, 1e-3);
        ros::param::param("gicp/feps", feps, 1e-3);
        ros::param::param("gicp/fitness_score", fitness_score, 0.3);
        backend.relocalization->set_gicp_param(use_gicp, filter_range, gicp_downsample, search_radius, teps, feps, fitness_score);
    }

    backend.init_system_mode();
}

void load_pgm_parameters()
{
    ros::param::param("official/save_globalmap_en", save_globalmap_en, false);
    ros::param::param("official/save_pgm", save_pgm, false);
    ros::param::param("official/pgm_resolution", pgm_resolution, 0.05);
    ros::param::param("official/min_z", min_z, -1.5f);
    ros::param::param("official/max_z", max_z, 0.1f);
}

void init_pgo_system(ros::NodeHandle &nh)
{
    ros::param::param("showOptimizedPose", showOptimizedPose, true);
    ros::param::param("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1000.);
    ros::param::param("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.);
    ros::param::param("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.);

    load_ros_parameters();
    load_parameters();
    load_pgm_parameters();

    /*** ROS subscribe initialization ***/
#ifdef UrbanLoco
    sub_gnss = nh.subscribe(gnss_topic, 200000, UrbanLoco_cbk);
#else
    sub_gnss = nh.subscribe(gnss_topic, 200000, gnss_cbk);
#endif
    pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/odom_fix", 100000);
    pubLidarPath = nh.advertise<nav_msgs::Path>("/lidar_keyframe_trajectory", 100000);
    pubOdomNotFix = nh.advertise<nav_msgs::Odometry>("/odom_not_fix", 100000);
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints", 1);

    pubGlobalmap = nh.advertise<sensor_msgs::PointCloud2>("/map_global", 1);
    visualizeMapThread = std::thread(&visualize_globalmap_thread, pubGlobalmap);
    sub_initpose = nh.subscribe("/initialpose", 1, initialPoseCallback);
}
