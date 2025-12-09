/**
 * Local Pointcloud Simulator - Configuration Header
 * 
 * This file defines the configuration class for the local pointcloud simulator ROS node.
 * It loads ROS-specific parameters from a YAML configuration file using yaml_loader.
 * 
 * Note: Other LiDAR rendering parameters are passed directly to marsim_render config,
 * not stored in this class.
 */

#ifndef LOCAL_POINTCLOUD_SIM_CONFIG_HPP
#define LOCAL_POINTCLOUD_SIM_CONFIG_HPP

#include <marsim_render/yaml_loader.hpp>
#include <string>

#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/"+(name)))

namespace local_pc_sim {

    class LocalPointCloudSimConfig {
    public:
        LocalPointCloudSimConfig() = default;

        explicit LocalPointCloudSimConfig(const std::string& cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            
            // ROS Topic Names
            loader.LoadParam("pose_topic", pose_topic, std::string("/lidar_slam/pose"), false);
            loader.LoadParam("odom_topic", odom_topic, std::string("/lidar_slam/odom"), false);
            loader.LoadParam("use_odom", use_odom, false, false);
            loader.LoadParam("local_pc_topic", local_pc_topic, std::string("/cloud_registered"), false);
            loader.LoadParam("global_pc_topic", global_pc_topic, std::string("/global_pc"), false);
            
            // Frame ID for point cloud messages
            loader.LoadParam("frame_id", frame_id, std::string("map"), false);
            
            // Sensing rate (needed for ROS node timer)
            loader.LoadParam("sensing_rate", sensing_rate, 10, false);
        }

        // ROS Topic Configuration
        std::string pose_topic = "/lidar_slam/pose";
        std::string odom_topic = "/lidar_slam/odom";
        bool use_odom = false;  // If true, use odom_topic; if false, use pose_topic
        std::string local_pc_topic = "/cloud_registered";
        std::string global_pc_topic = "/global_pc";
        
        // Frame ID for point cloud messages
        std::string frame_id = "map";
        
        // Sensing rate (Hz) - needed for timer
        int sensing_rate = 10;
    };
}

#endif // LOCAL_POINTCLOUD_SIM_CONFIG_HPP
