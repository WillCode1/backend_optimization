#include <csignal>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "pgo/Backend.hpp"
#include "interface_ros2.h"

FILE *location_log = nullptr;
bool showOptimizedPose = true;
double globalMapVisualizationSearchRadius = 1000;
double globalMapVisualizationPoseDensity = 10;
double globalMapVisualizationLeafSize = 1;
double lidar_end_time = 0;
bool path_en = true, scan_pub_en = false, dense_pub_en = false;
Backend backend;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLidarPath;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomNotFix;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalmap;
std::thread visualizeMapThread;
rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initpose;
std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

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

void gnss_cbk(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    backend.gnss->gnss_handler(GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, V3D(msg->latitude, msg->longitude, msg->altitude)));
    backend.relocalization->gnss_pose = GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, V3D(msg->latitude, msg->longitude, msg->altitude));
}

#ifdef UrbanLoco
void UrbanLoco_cbk(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    backend.gnss->UrbanLoco_handler(GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
                                             V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
                                             QD(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
                                             V3D(msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35])));
    backend.relocalization->gnss_pose = GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
                                                 V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
                                                 QD(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));
}
#endif

void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubCloud, PointCloudType::Ptr cloud, const double &lidar_end_time, const std::string &frame_id)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    cloud_msg.header.frame_id = frame_id;
    pubCloud->publish(cloud_msg);
}

void publish_cloud_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull, PointCloudType::Ptr laserCloud, const PointXYZIRPYT &state, const double &lidar_end_time)
{
    PointCloudType::Ptr laserCloudWorld(new PointCloudType(laserCloud->size(), 1));
    pointcloudLidarToWorld(laserCloud, laserCloudWorld, state);
    publish_cloud(pubLaserCloudFull, laserCloudWorld, lidar_end_time, map_frame);
}

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

void publish_tf(const geometry_msgs::msg::Pose &pose, const double &lidar_end_time)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    // lidar -> map
    transform_stamped.header.frame_id = map_frame;
    transform_stamped.child_frame_id = lidar_frame;
    transform_stamped.transform.translation.x = pose.position.x;
    transform_stamped.transform.translation.y = pose.position.y;
    transform_stamped.transform.translation.z = pose.position.z;
    transform_stamped.transform.rotation.x = pose.orientation.x;
    transform_stamped.transform.rotation.y = pose.orientation.y;
    transform_stamped.transform.rotation.z = pose.orientation.z;
    transform_stamped.transform.rotation.w = pose.orientation.w;
    broadcaster->sendTransform(transform_stamped);
}

// 发布里程计
void publish_odometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped,
                      const PointXYZIRPYT &state, const double &lidar_end_time, bool need_publish_tf = true)
{
    nav_msgs::msg::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = lidar_frame;
    odomAftMapped.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    const QD &lidar_rot = EigenMath::RPY2Quaternion(V3D(state.roll, state.pitch, state.yaw));
    const V3D &lidar_pos = V3D(state.x, state.y, state.z);
    set_posestamp(odomAftMapped.pose, lidar_rot, lidar_pos);
    pubOdomAftMapped->publish(odomAftMapped);
    if (need_publish_tf)
        publish_tf(odomAftMapped.pose.pose, lidar_end_time);
}

void publish_lidar_keyframe_trajectory(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath, const pcl::PointCloud<PointXYZIRPYT> &trajectory, const double &lidar_end_time)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    path.header.frame_id = map_frame;

    geometry_msgs::msg::PoseStamped msg_lidar_pose;
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

        msg_lidar_pose.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
        msg_lidar_pose.header.frame_id = map_frame;

        path.poses.push_back(msg_lidar_pose);
    }

    pubPath->publish(path);
}

void visualize_loop_closure_constraints(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pubLoopConstraintEdge, const double &timestamp,
                                        const unordered_map<int, int> &loop_constraint_records,
                                        const pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d)
{
    if (loop_constraint_records.empty())
        return;

    visualization_msgs::msg::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = map_frame;
    markerNode.header.stamp = rclcpp::Time(timestamp * 1e9);
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
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
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = map_frame;
    markerEdge.header.stamp = rclcpp::Time(timestamp * 1e9);
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
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
        geometry_msgs::msg::Point p;
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
    pubLoopConstraintEdge->publish(markerArray);
}

void visualize_globalmap_thread(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalmap)
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

void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const geometry_msgs::msg::Pose &pose = msg->pose.pose;
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

void pgo_handle(PointXYZIRPYT &this_pose6d, PointCloudType::Ptr &feats_undistort, PointCloudType::Ptr &submap_fix)
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

void load_ros_parameters(rclcpp::Node::SharedPtr &node)
{
    node->declare_parameter("path_en", false);
    node->declare_parameter("scan_publish_en", false);
    node->declare_parameter("dense_publish_en", false);

    node->get_parameter("path_en", path_en);
    node->get_parameter("scan_publish_en", scan_pub_en);
    node->get_parameter("dense_publish_en", dense_pub_en);

    node->declare_parameter("gnss_topic", "/gps/fix");
    node->declare_parameter("map_frame", "camera_init");
    node->declare_parameter("lidar_frame", "lidar");

    node->get_parameter("gnss_topic", gnss_topic);
    node->get_parameter("map_frame", map_frame);
    node->get_parameter("lidar_frame", lidar_frame);
}

void load_parameters(rclcpp::Node::SharedPtr &node)
{
    vector<double> extrinT;
    vector<double> extrinR;
    V3D extrinT_eigen;
    M3D extrinR_eigen;

    node->declare_parameter("keyframe_add_dist_threshold", 1.f);
    node->declare_parameter("keyframe_add_angle_threshold", 0.2f);
    node->declare_parameter("pose_cov_threshold", 25.f);
    node->declare_parameter("gnssValidInterval", 0.2f);
    node->declare_parameter("gpsCovThreshold", 2.f);
    node->declare_parameter("useGpsElevation", false);
    node->declare_parameter("extrinsic_gnss_T", vector<double>());
    node->declare_parameter("extrinsic_gnss_R", vector<double>());
    node->declare_parameter("recontruct_kdtree", true);
    node->declare_parameter("ikdtree_reconstruct_keyframe_num", 10);
    node->declare_parameter("ikdtree_reconstruct_downsamp_size", 0.1f);
    node->declare_parameter("loop_closure_enable_flag", false);
    node->declare_parameter("loop_closure_interval", 1000);
    node->declare_parameter("loop_keyframe_num_thld", 50);
    node->declare_parameter("loop_closure_search_radius", 10.f);
    node->declare_parameter("loop_closure_keyframe_interval", 30);
    node->declare_parameter("keyframe_search_num", 20);
    node->declare_parameter("loop_closure_fitness_score_thld", 0.05);
    node->declare_parameter("icp_downsamp_size", 0.1);
    node->declare_parameter("manually_loop_vaild_period", vector<double>());
    node->declare_parameter("odom_loop_vaild_period", vector<double>());
    node->declare_parameter("scancontext_loop_vaild_period", vector<double>());
    node->declare_parameter("save_keyframe_en", true);
    node->declare_parameter("save_keyframe_descriptor_en", true);
    node->declare_parameter("save_resolution", 0.1f);
    node->declare_parameter("map_path", "");
    node->declare_parameter("lidar_height", 2.0);
    node->declare_parameter("sc_dist_thres", 0.5);

    node->get_parameter("keyframe_add_dist_threshold", backend.backend->keyframe_add_dist_threshold);
    node->get_parameter("keyframe_add_angle_threshold", backend.backend->keyframe_add_angle_threshold);
    node->get_parameter("pose_cov_threshold", backend.backend->pose_cov_threshold);
    node->get_parameter("gnssValidInterval", backend.gnss->gnssValidInterval);
    node->get_parameter("gpsCovThreshold", backend.gnss->gpsCovThreshold);
    node->get_parameter("useGpsElevation", backend.gnss->useGpsElevation);

    node->get_parameter("extrinsic_gnss_T", extrinT);
    node->get_parameter("extrinsic_gnss_R", extrinR);
    extrinT_eigen << VEC_FROM_ARRAY(extrinT);
    extrinR_eigen << MAT_FROM_ARRAY(extrinR);
    backend.gnss->set_extrinsic(extrinT_eigen, extrinR_eigen);

    node->get_parameter("recontruct_kdtree", backend.backend->recontruct_kdtree);
    node->get_parameter("ikdtree_reconstruct_keyframe_num", backend.backend->ikdtree_reconstruct_keyframe_num);
    node->get_parameter("ikdtree_reconstruct_downsamp_size", backend.backend->ikdtree_reconstruct_downsamp_size);

    node->get_parameter("loop_closure_enable_flag", backend.loop_closure_enable_flag);
    node->get_parameter("loop_closure_interval", backend.loop_closure_interval);
    node->get_parameter("loop_keyframe_num_thld", backend.loopClosure->loop_keyframe_num_thld);
    node->get_parameter("loop_closure_search_radius", backend.loopClosure->loop_closure_search_radius);
    node->get_parameter("loop_closure_keyframe_interval", backend.loopClosure->loop_closure_keyframe_interval);
    node->get_parameter("keyframe_search_num", backend.loopClosure->keyframe_search_num);
    node->get_parameter("loop_closure_fitness_score_thld", backend.loopClosure->loop_closure_fitness_score_thld);
    node->get_parameter("icp_downsamp_size", backend.loopClosure->icp_downsamp_size);
    node->get_parameter("manually_loop_vaild_period", backend.loopClosure->loop_vaild_period["manually"]);
    node->get_parameter("odom_loop_vaild_period", backend.loopClosure->loop_vaild_period["odom"]);
    node->get_parameter("scancontext_loop_vaild_period", backend.loopClosure->loop_vaild_period["scancontext"]);

    node->get_parameter("save_keyframe_en", backend.save_keyframe_en);
    node->get_parameter("save_keyframe_descriptor_en", backend.save_keyframe_descriptor_en);
    node->get_parameter("save_resolution", backend.save_resolution);
    node->get_parameter("map_path", backend.map_path);
    if (backend.map_path.compare("") != 0)
    {
        backend.globalmap_path = backend.map_path + "/globalmap.pcd";
        backend.trajectory_path = backend.map_path + "/trajectory.pcd";
        backend.keyframe_path = backend.map_path + "/keyframe/";
        backend.scd_path = backend.map_path + "/scancontext/";
    }
    else
        backend.map_path = PCD_FILE_DIR("");

    node->get_parameter("lidar_height", backend.relocalization->sc_manager->LIDAR_HEIGHT);
    node->get_parameter("sc_dist_thres", backend.relocalization->sc_manager->SC_DIST_THRES);

    if (false)
    {
        node->declare_parameter("utm_origin_zone", "51N");
        node->declare_parameter("utm_origin_east", 0.);
        node->declare_parameter("utm_origin_north", 0.);
        node->declare_parameter("utm_origin_up", 0.);
        node->declare_parameter("extrinsicT_imu2gnss", vector<double>());
        node->declare_parameter("extrinsicR_imu2gnss", vector<double>());
        node->declare_parameter("relocal_cfg_algorithm_type", "UNKONW");

        node->get_parameter("utm_origin_zone", backend.relocalization->utm_origin.zone);
        node->get_parameter("utm_origin_east", backend.relocalization->utm_origin.east);
        node->get_parameter("utm_origin_north", backend.relocalization->utm_origin.north);
        node->get_parameter("utm_origin_up", backend.relocalization->utm_origin.up);

        node->get_parameter("extrinsicT_imu2gnss", extrinT);
        node->get_parameter("extrinsicR_imu2gnss", extrinR);
        extrinT_eigen << VEC_FROM_ARRAY(extrinT);
        extrinR_eigen << MAT_FROM_ARRAY(extrinR);
        backend.relocalization->set_extrinsic(extrinT_eigen, extrinR_eigen);

        node->get_parameter("relocal_cfg_algorithm_type", backend.relocalization->algorithm_type);

        BnbOptions match_option;
        node->declare_parameter("bnb3d_linear_xy_window_size", 10.);
        node->declare_parameter("bnb3d_linear_z_window_size", 1.);
        node->declare_parameter("bnb3d_angular_search_window", 30.);
        node->declare_parameter("bnb3d_pc_resolutions", vector<double>());
        node->declare_parameter("bnb3d_bnb_depth", 5);
        node->declare_parameter("bnb3d_min_score", 0.1);
        node->declare_parameter("bnb3d_enough_score", 0.8);
        node->declare_parameter("bnb3d_min_xy_resolution", 0.2);
        node->declare_parameter("bnb3d_min_z_resolution", 0.1);
        node->declare_parameter("bnb3d_min_angular_resolution", 0.1);
        node->declare_parameter("bnb3d_filter_size_scan", 0.1);
        node->declare_parameter("bnb3d_debug_mode", false);

        node->get_parameter("bnb3d_linear_xy_window_size", match_option.linear_xy_window_size);
        node->get_parameter("bnb3d_linear_z_window_size", match_option.linear_z_window_size);
        node->get_parameter("bnb3d_angular_search_window", match_option.angular_search_window);
        node->get_parameter("bnb3d_pc_resolutions", match_option.pc_resolutions);
        node->get_parameter("bnb3d_depth", match_option.bnb_depth);
        node->get_parameter("bnb3d_min_score", match_option.min_score);
        node->get_parameter("bnb3d_enough_score", match_option.enough_score);
        node->get_parameter("bnb3d_min_xy_resolution", match_option.min_xy_resolution);
        node->get_parameter("bnb3d_min_z_resolution", match_option.min_z_resolution);
        node->get_parameter("bnb3d_min_angular_resolution", match_option.min_angular_resolution);
        node->get_parameter("bnb3d_filter_size_scan", match_option.filter_size_scan);
        node->get_parameter("bnb3d_debug_mode", match_option.debug_mode);

        node->declare_parameter("extrinsic_T", vector<double>());
        node->declare_parameter("extrinsic_R", vector<double>());
        node->get_parameter("extrinsic_T", extrinT);
        node->get_parameter("extrinsic_R", extrinR);
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
        node->declare_parameter("ndt_step_size", 0.1);
        node->declare_parameter("ndt_resolution", 1.);
        node->get_parameter("ndt_step_size", step_size);
        node->get_parameter("ndt_resolution", resolution);
        backend.relocalization->set_ndt_param(step_size, resolution);

        bool use_gicp;
        double gicp_downsample, filter_range, search_radius, teps, feps, fitness_score;
        node->declare_parameter("gicp_use_gicp", true);
        node->declare_parameter("gicp_filter_range", 80.);
        node->declare_parameter("gicp_downsample", 0.2);
        node->declare_parameter("gicp_search_radius", 0.5);
        node->declare_parameter("gicp_teps", 1e-3);
        node->declare_parameter("gicp_feps", 1e-3);
        node->declare_parameter("gicp_fitness_score", 0.3);

        node->get_parameter("gicp_use_gicp", use_gicp);
        node->get_parameter("gicp_filter_range", filter_range);
        node->get_parameter("gicp_gicp_downsample", gicp_downsample);
        node->get_parameter("gicp_search_radius", search_radius);
        node->get_parameter("gicp_teps", teps);
        node->get_parameter("gicp_feps", feps);
        node->get_parameter("gicp_fitness_score", fitness_score);
        backend.relocalization->set_gicp_param(use_gicp, filter_range, gicp_downsample, search_radius, teps, feps, fitness_score);
    }

    backend.init_system_mode();
}

void load_pgm_parameters(rclcpp::Node::SharedPtr &node)
{
    node->declare_parameter("save_globalmap_en", true);
    node->declare_parameter("save_pgm", false);
    node->declare_parameter("pgm_resolution", 0.05);
    node->declare_parameter("min_z", -1.5f);
    node->declare_parameter("max_z", 0.1f);

    node->get_parameter("save_globalmap_en", save_globalmap_en);
    node->get_parameter("save_pgm", save_pgm);
    node->get_parameter("pgm_resolution", pgm_resolution);
    node->get_parameter("min_z", min_z);
    node->get_parameter("max_z", max_z);
}

void init_pgo_system(rclcpp::Node::SharedPtr &node)
{
    node->declare_parameter("showOptimizedPose", true);
    node->declare_parameter("globalMapVisualizationSearchRadius", 1000.);
    node->declare_parameter("globalMapVisualizationPoseDensity", 10.);
    node->declare_parameter("globalMapVisualizationLeafSize", 1.);
    node->get_parameter("showOptimizedPose", showOptimizedPose);
    node->get_parameter("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
    node->get_parameter("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
    node->get_parameter("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

    load_ros_parameters(node);
    load_parameters(node);
    load_pgm_parameters(node);

    /*** ROS subscribe initialization ***/
    // 发布当前正在扫描的点云，topic名字为/cloud_registered
    pubLaserCloudFull = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1000);
    pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>("/odom_fix", 1000);
    pubLidarPath = node->create_publisher<nav_msgs::msg::Path>("/lidar_keyframe_trajectory", 1000);
    pubOdomNotFix = node->create_publisher<nav_msgs::msg::Odometry>("/odom_not_fix", 1000);
    pubLoopConstraintEdge = node->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure_constraints", 1);

    pubGlobalmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_global", 1);
    visualizeMapThread = std::thread(&visualize_globalmap_thread, pubGlobalmap);
    sub_initpose = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseCallback);
    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
}
