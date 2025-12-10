/**
 * Local Pointcloud Simulator - ROS Node
 * 
 * This node receives pose or odometry messages and publishes local and global
 * point clouds using the marsim_render library.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "marsim_render/marsim_render.hpp"
#include "local_sensing_sim/local_pointcloud_sim_config.hpp"

typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<float, 3, 1> Vec3f;

class LocalPointCloudSimulator {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_, odom_sub_;
    ros::Publisher local_pc_pub_, global_pc_pub_;
    ros::Timer local_pc_timer_;

    std::shared_ptr<marsim::MarsimRender> render_ptr_;
    local_pc_sim::LocalPointCloudSimConfig config_;

    // Current pose state
    Vec3f current_position_;
    Eigen::Quaternionf current_quaternion_;
    ros::Time last_pose_time_;
    bool pose_received_;

public:
    LocalPointCloudSimulator(const ros::NodeHandle& nh, const std::string& config_path)
        : nh_(nh), config_(config_path), pose_received_(false) {
        
        ROS_INFO("Initializing LocalPointCloudSimulator with config: %s", config_path.c_str());

        // Initialize render engine
        render_ptr_ = std::make_shared<marsim::MarsimRender>(config_path);

        // Setup subscribers based on configuration
        if (config_.use_odom) {
            ROS_INFO("Subscribing to odometry topic: %s", config_.odom_topic.c_str());
            odom_sub_ = nh_.subscribe(config_.odom_topic, 10, &LocalPointCloudSimulator::odomCallback, this);
        } else {
            ROS_INFO("Subscribing to pose topic: %s", config_.pose_topic.c_str());
            pose_sub_ = nh_.subscribe(config_.pose_topic, 10, &LocalPointCloudSimulator::poseCallback, this);
        }

        // Setup publishers
        ROS_INFO("Publishing to local_pc_topic: %s", config_.local_pc_topic.c_str());
        local_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.local_pc_topic, 10);

        ROS_INFO("Publishing to global_pc_topic: %s", config_.global_pc_topic.c_str());
        global_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.global_pc_topic, 10);

        // Initialize position and orientation
        current_position_ = Vec3f(0, 0, 0);
        current_quaternion_ = Eigen::Quaternionf::Identity();
        last_pose_time_ = ros::Time::now();

        ROS_INFO("LocalPointCloudSimulator initialized successfully");
    }

    ~LocalPointCloudSimulator() {
        ROS_INFO("Shutting down LocalPointCloudSimulator");
    }

    void getSensingRate(double& rate) const {
        rate = config_.sensing_rate;
    }

    void publishLocalPointCloud() {
        if (!pose_received_) {
            if (config_.use_odom) {
                ROS_WARN_ONCE("Waiting for odom message on topic: %s", config_.odom_topic.c_str());
            }
            else {
                ROS_WARN_ONCE("Waiting for pose message on topic: %s", config_.pose_topic.c_str());
            }
            return;
        }

        // Check if there are subscribers
        if (local_pc_pub_.getNumSubscribers() <= 0) {
            return;
        }

        try {
            // Render point cloud from current position
            pcl::PointCloud<marsim::PointType>::Ptr local_cloud(
                new pcl::PointCloud<marsim::PointType>
            );

            render_ptr_->renderOnceInWorld(
                current_position_,
                current_quaternion_,
                last_pose_time_.toSec(),
                local_cloud
            );

            // Convert to ROS message
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*local_cloud, pc_msg);
            pc_msg.header.frame_id = config_.frame_id;
            pc_msg.header.stamp = last_pose_time_;

            local_pc_pub_.publish(pc_msg);

            ROS_DEBUG("Published local point cloud with %zu points", local_cloud->size());
        } catch (const std::exception& e) {
            ROS_ERROR("Error publishing local point cloud: %s", e.what());
        }
    }

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_position_ = Vec3f(
            static_cast<float>(msg->pose.position.x),
            static_cast<float>(msg->pose.position.y),
            static_cast<float>(msg->pose.position.z)
        );

        current_quaternion_ = Eigen::Quaternionf(
            static_cast<float>(msg->pose.orientation.w),
            static_cast<float>(msg->pose.orientation.x),
            static_cast<float>(msg->pose.orientation.y),
            static_cast<float>(msg->pose.orientation.z)
        );

        last_pose_time_ = msg->header.stamp;
        pose_received_ = true;

        // Publish global point cloud if there are subscribers
        publishGlobalPointCloud();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_ = Vec3f(
            static_cast<float>(msg->pose.pose.position.x),
            static_cast<float>(msg->pose.pose.position.y),
            static_cast<float>(msg->pose.pose.position.z)
        );

        current_quaternion_ = Eigen::Quaternionf(
            static_cast<float>(msg->pose.pose.orientation.w),
            static_cast<float>(msg->pose.pose.orientation.x),
            static_cast<float>(msg->pose.pose.orientation.y),
            static_cast<float>(msg->pose.pose.orientation.z)
        );

        last_pose_time_ = msg->header.stamp;
        pose_received_ = true;

        // Publish global point cloud if there are subscribers
        publishGlobalPointCloud();
    }

    void publishGlobalPointCloud() {
        static int last_sub_num = 0;
        int sub_num = global_pc_pub_.getNumSubscribers();

        // Only publish if number of subscribers changed and there are subscribers
        if (sub_num > 0 && last_sub_num != sub_num) {
            try {
                // Small delay to allow subscriber to fully initialize
                ros::Duration(0.5).sleep();

                pcl::PointCloud<marsim::PointType>::Ptr global_cloud(
                    new pcl::PointCloud<marsim::PointType>
                );

                render_ptr_->getGlobalMap(global_cloud);

                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(*global_cloud, pc_msg);
                pc_msg.header.frame_id = config_.frame_id;
                pc_msg.header.stamp = ros::Time::now();

                global_pc_pub_.publish(pc_msg);

                ROS_INFO("Published global point cloud with %zu points", global_cloud->size());
            } catch (const std::exception& e) {
                ROS_ERROR("Error publishing global point cloud: %s", e.what());
            }
        }
        last_sub_num = sub_num;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_pointcloud_sim");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string config_path;
    std::string default_config_path = std::string(ROOT_DIR) + "config/local_pointcloud_sim.yaml";

    // Try to get config from parameters
    if (nh_private.getParam("config_path", config_path)) {
        ROS_INFO("Loading config from ROS param 'config_path': %s", config_path.c_str());
    } else {
        config_path = default_config_path;
        ROS_INFO("Using default config path: %s", config_path.c_str());
    }

    try {
        LocalPointCloudSimulator simulator(nh, config_path);
        ROS_INFO("LocalPointCloudSimulator node started");
        ros::AsyncSpinner spinner(1);
        spinner.start();

        /* Main publish local point cloud loop */
        double sensing_rate;
        simulator.getSensingRate(sensing_rate);
        ros::Rate rate(sensing_rate);
        while (ros::ok()) {
            simulator.publishLocalPointCloud();
            rate.sleep();
        }

        ros::waitForShutdown();
    } catch (const std::exception& e) {
        ROS_FATAL("Failed to start LocalPointCloudSimulator: %s", e.what());
        return 1;
    }

    return 0;
}
