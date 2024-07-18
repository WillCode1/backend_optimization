#pragma once
#include <ros/ros.h>
#include "pgo/Backend.hpp"

inline void load_ros_parameters(bool &path_en, bool &scan_pub_en, bool &dense_pub_en,
                                std::string &gnss_topic, std::string &map_frame, std::string &lidar_frame)
{
    ros::param::param("publish/path_en", path_en, false);
    ros::param::param("publish/scan_publish_en", scan_pub_en, false);
    ros::param::param("publish/dense_publish_en", dense_pub_en, false);

    ros::param::param("common/gnss_topic", gnss_topic, std::string("/gps/fix"));
    ros::param::param("common/map_frame", map_frame, std::string("camera_init"));
    ros::param::param("common/lidar_frame", lidar_frame, std::string("lidar"));
}

inline void load_parameters(Backend &backend)
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

inline void load_pgm_parameters(bool &save_globalmap_en, bool &save_pgm, double &pgm_resolution, float &min_z, float &max_z)
{
    ros::param::param("official/save_globalmap_en", save_globalmap_en, false);
    ros::param::param("official/save_pgm", save_pgm, false);
    ros::param::param("official/pgm_resolution", pgm_resolution, 0.05);
    ros::param::param("official/min_z", min_z, -1.5f);
    ros::param::param("official/max_z", max_z, 0.1f);
}
