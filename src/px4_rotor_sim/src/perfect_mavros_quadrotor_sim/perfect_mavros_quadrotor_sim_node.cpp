/**
 * @file perfect_mavros_quadrotor_sim_node.cpp
 * @brief Perfect MAVROS drone simulator - directly maps setpoint to state
 * @author Peixuan Shu 
 * @version 1.0
 * @date Jan 2026
 * 
 * This node creates an ideal drone simulation that:
 * - Subscribes to mavros/setpoint_raw/local (only responds in OFFBOARD mode when armed)
 * - Publishes desired position/velocity/attitude directly to:
 *   - mavros/local_position/pose
 *   - mavros/local_position/velocity_local
 *   - mavros/local_position/odom
 * - Publishes mavros/state
 * - Provides services:
 *   - mavros/set_mode (supports PX4 flight modes)
 *   - mavros/cmd/arming (arm/disarm control)
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

class PerfectMavrosDrone
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // Subscribers
    ros::Subscriber setpoint_sub_;
    
    // Service servers
    ros::ServiceServer set_mode_srv_;
    ros::ServiceServer arming_srv_;
    
    // Publishers
    ros::Publisher pose_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher state_pub_;
    
    // Timers
    ros::Timer pose_timer_;
    ros::Timer velocity_timer_;
    ros::Timer odom_timer_;
    ros::Timer state_timer_;
    
    // Current state
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_velocity_;
    nav_msgs::Odometry current_odom_;
    mavros_msgs::State current_state_;
    
    // Parameters
    double pose_publish_rate_;
    double velocity_publish_rate_;
    double odom_publish_rate_;
    double state_publish_rate_;
    
    // Initial state parameters
    double init_x_;
    double init_y_;
    double init_z_;
    double init_yaw_;
    
    // Time tracking for integration
    ros::Time last_callback_time_;
    bool first_callback_;
    
public:
    PerfectMavrosDrone() : nh_private_("~"), first_callback_(true)
    {
        // Load parameters
        nh_private_.param<double>("pose_publish_rate", pose_publish_rate_, 30.0);
        nh_private_.param<double>("velocity_publish_rate", velocity_publish_rate_, 30.0);
        nh_private_.param<double>("odom_publish_rate", odom_publish_rate_, 30.0);
        nh_private_.param<double>("state_publish_rate", state_publish_rate_, 10.0);
        
        // Load initial state parameters
        nh_private_.param<double>("init_x", init_x_, 0.0);
        nh_private_.param<double>("init_y", init_y_, 0.0);
        nh_private_.param<double>("init_z", init_z_, 0.0);
        
        // Load yaw in degrees and convert to radians
        double init_yaw_deg = 0.0;
        nh_private_.param<double>("init_yaw", init_yaw_deg, 0.0);
        init_yaw_ = init_yaw_deg * M_PI / 180.0;
        
        // Setup topic names (relative paths)
        std::string setpoint_topic = "mavros/setpoint_raw/local";
        std::string pose_topic = "mavros/local_position/pose";
        std::string velocity_topic = "mavros/local_position/velocity_local";
        std::string odom_topic = "mavros/local_position/odom";
        std::string state_topic = "mavros/state";
        std::string set_mode_service = "mavros/set_mode";
        std::string arming_service = "mavros/cmd/arming";
        
        // Initialize subscribers
        setpoint_sub_ = nh_.subscribe(setpoint_topic, 10, 
            &PerfectMavrosDrone::setpointCallback, this);
        
        // Initialize service servers
        set_mode_srv_ = nh_.advertiseService(set_mode_service, 
            &PerfectMavrosDrone::setModeCallback, this);
        arming_srv_ = nh_.advertiseService(arming_service, 
            &PerfectMavrosDrone::armingCallback, this);
        
        // Initialize publishers
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
        velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(velocity_topic, 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 10);
        state_pub_ = nh_.advertise<mavros_msgs::State>(state_topic, 10);
        
        // Initialize state
        initializeState();
        
        // Setup timers for periodic publishing
        pose_timer_ = nh_.createTimer(
            ros::Duration(1.0 / pose_publish_rate_),
            &PerfectMavrosDrone::poseTimerCallback, this);
        
        velocity_timer_ = nh_.createTimer(
            ros::Duration(1.0 / velocity_publish_rate_),
            &PerfectMavrosDrone::velocityTimerCallback, this);
        
        odom_timer_ = nh_.createTimer(
            ros::Duration(1.0 / odom_publish_rate_),
            &PerfectMavrosDrone::odomTimerCallback, this);
        
        state_timer_ = nh_.createTimer(
            ros::Duration(1.0 / state_publish_rate_),
            &PerfectMavrosDrone::stateTimerCallback, this);
        
        ROS_INFO("[PerfectMavrosDrone] Initialized");
        ROS_INFO("[PerfectMavrosDrone] Initial state - Position: (%.2f, %.2f, %.2f), Yaw: %.2f deg (%.2f rad)",
                 init_x_, init_y_, init_z_, init_yaw_deg, init_yaw_);
        ROS_INFO("[PerfectMavrosDrone] Subscribing to: %s", setpoint_topic.c_str());
        ROS_INFO("[PerfectMavrosDrone] Publishing to: %s, %s, %s, %s", 
                 pose_topic.c_str(), velocity_topic.c_str(), 
                 odom_topic.c_str(), state_topic.c_str());
        ROS_INFO("[PerfectMavrosDrone] Publish rates - Pose: %.1fHz, Velocity: %.1fHz, Odom: %.1fHz, State: %.1fHz",
                 pose_publish_rate_, velocity_publish_rate_, odom_publish_rate_, state_publish_rate_);
    }
    
    void initializeState()
    {
        // Initialize pose with parameters
        current_pose_.header.frame_id = "map";
        current_pose_.pose.position.x = init_x_;
        current_pose_.pose.position.y = init_y_;
        current_pose_.pose.position.z = init_z_;
        
        // Set initial orientation from yaw
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, init_yaw_);
        current_pose_.pose.orientation = tf2::toMsg(q);
        
        // Initialize velocity
        current_velocity_.header.frame_id = "map";
        current_velocity_.twist.linear.x = 0.0;
        current_velocity_.twist.linear.y = 0.0;
        current_velocity_.twist.linear.z = 0.0;
        current_velocity_.twist.angular.x = 0.0;
        current_velocity_.twist.angular.y = 0.0;
        current_velocity_.twist.angular.z = 0.0;
        
        // Initialize odometry
        current_odom_.header.frame_id = "map";
        current_odom_.child_frame_id = "base_link";
        current_odom_.pose.pose = current_pose_.pose;
        current_odom_.twist.twist = current_velocity_.twist;
        
        // Initialize state - armed and in OFFBOARD mode
        current_state_.connected = true;
        current_state_.armed = true;
        current_state_.guided = true;
        current_state_.manual_input = false;
        current_state_.mode = "OFFBOARD";
        current_state_.system_status = 4;
    }
    
    void setpointCallback(const mavros_msgs::PositionTarget::ConstPtr& msg)
    {
        // Only respond to setpoint commands in OFFBOARD mode and when armed
        if (current_state_.mode != "OFFBOARD" || !current_state_.armed)
        {
            // Not in OFFBOARD mode or not armed - keep position and yaw, set roll/pitch/velocities to zero
            ros::Time now = ros::Time::now();
            
            current_pose_.header.stamp = now;
            // Position remains unchanged
            
            // Extract current yaw and set roll/pitch to zero
            tf2::Quaternion q_current;
            tf2::fromMsg(current_pose_.pose.orientation, q_current);
            tf2::Matrix3x3 m(q_current);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            // Reconstruct orientation with zero roll and pitch
            tf2::Quaternion q_hover;
            q_hover.setRPY(0.0, 0.0, yaw);
            current_pose_.pose.orientation = tf2::toMsg(q_hover);
            
            // Set all velocities to zero
            current_velocity_.header.stamp = now;
            current_velocity_.twist.linear.x = 0.0;
            current_velocity_.twist.linear.y = 0.0;
            current_velocity_.twist.linear.z = 0.0;
            current_velocity_.twist.angular.x = 0.0;
            current_velocity_.twist.angular.y = 0.0;
            current_velocity_.twist.angular.z = 0.0;
            
            // Update odometry
            current_odom_.header.stamp = now;
            current_odom_.pose.pose = current_pose_.pose;
            current_odom_.twist.twist = current_velocity_.twist;
            
            return;
        }
        
        ros::Time now = ros::Time::now();
        
        // Calculate dt based on actual time
        double dt = 0.0;
        if (!first_callback_)
        {
            dt = (now - last_callback_time_).toSec();
        }
        else
        {
            first_callback_ = false;
        }
        last_callback_time_ = now;
        
        // Update pose based on setpoint
        current_pose_.header.stamp = now;
        
        // Position setpoint - integrate velocity if position not provided
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_PX)
        {
            // Position X ignored, integrate velocity
            if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_VX) && dt > 0.0)
            {
                current_pose_.pose.position.x += msg->velocity.x * dt;
            }
        }
        else
        {
            current_pose_.pose.position.x = msg->position.x;
        }
        
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_PY)
        {
            // Position Y ignored, integrate velocity
            if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_VY) && dt > 0.0)
            {
                current_pose_.pose.position.y += msg->velocity.y * dt;
            }
        }
        else
        {
            current_pose_.pose.position.y = msg->position.y;
        }
        
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_PZ)
        {
            // Position Z ignored, integrate velocity
            if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_VZ) && dt > 0.0)
            {
                current_pose_.pose.position.z += msg->velocity.z * dt;
            }
        }
        else
        {
            current_pose_.pose.position.z = msg->position.z;
        }
        
        // Velocity setpoint
        current_velocity_.header.stamp = now;
        
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_VX)
        {
            // Velocity X ignored
        }
        else
        {
            current_velocity_.twist.linear.x = msg->velocity.x;
        }
        
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_VY)
        {
            // Velocity Y ignored
        }
        else
        {
            current_velocity_.twist.linear.y = msg->velocity.y;
        }
        
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_VZ)
        {
            // Velocity Z ignored
        }
        else
        {
            current_velocity_.twist.linear.z = msg->velocity.z;
        }
        
        // Attitude setpoint (yaw) - integrate yaw rate if yaw not provided
        double yaw = 0.0;
        
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_YAW)
        {
            // Yaw ignored, extract current yaw and integrate yaw rate if provided
            tf2::Quaternion q_current;
            tf2::fromMsg(current_pose_.pose.orientation, q_current);
            tf2::Matrix3x3 m(q_current);
            double roll, pitch;
            m.getRPY(roll, pitch, yaw);
            
            // Integrate yaw rate if provided
            if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_YAW_RATE) && dt > 0.0)
            {
                yaw += msg->yaw_rate * dt;
            }
        }
        else
        {
            yaw = msg->yaw;
        }
        
        // Calculate orientation from acceleration direction and yaw
        if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_AFX) ||
            !(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_AFY) ||
            !(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_AFZ))
        {
            // Get acceleration and remove gravity
            double ax = (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_AFX) ? 0.0 : msg->acceleration_or_force.x;
            double ay = (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_AFY) ? 0.0 : msg->acceleration_or_force.y;
            double az = (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_AFZ) ? 0.0 : msg->acceleration_or_force.z;
            
            // Remove gravity (assuming ENU frame: gravity is in -z direction)
            az += 9.81;
            
            // Calculate orientation from acceleration and yaw
            current_pose_.pose.orientation = calculateOrientationFromAcceleration(ax, ay, az, yaw);
        }
        else
        {
            // No acceleration command, use yaw only
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            current_pose_.pose.orientation = tf2::toMsg(q);
        }
        
        // Yaw rate
        if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_YAW_RATE))
        {
            current_velocity_.twist.angular.z = msg->yaw_rate;
        }
        
        // Update odometry
        current_odom_.header.stamp = now;
        current_odom_.pose.pose = current_pose_.pose;
        current_odom_.twist.twist = current_velocity_.twist;
    }
    
    void poseTimerCallback(const ros::TimerEvent& event)
    {
        // Periodically publish pose
        current_pose_.header.stamp = ros::Time::now();
        pose_pub_.publish(current_pose_);
    }
    
    void velocityTimerCallback(const ros::TimerEvent& event)
    {
        // Periodically publish velocity
        current_velocity_.header.stamp = ros::Time::now();
        velocity_pub_.publish(current_velocity_);
    }
    
    void odomTimerCallback(const ros::TimerEvent& event)
    {
        // Periodically publish odometry
        current_odom_.header.stamp = ros::Time::now();
        odom_pub_.publish(current_odom_);
    }
    
    void stateTimerCallback(const ros::TimerEvent& event)
    {
        // Periodically publish state
        current_state_.header.stamp = ros::Time::now();
        state_pub_.publish(current_state_);
    }
    
    bool setModeCallback(mavros_msgs::SetMode::Request& req,
                         mavros_msgs::SetMode::Response& res)
    {
        ROS_INFO("[PerfectMavrosDrone] Received set_mode request: %s", req.custom_mode.c_str());
        
        // Define supported PX4 flight modes
        const std::vector<std::string> supported_modes = {
            "MANUAL",
            "ACRO",
            "ALTCTL",
            "POSCTL",
            "OFFBOARD",
            "STABILIZED",
            "RATTITUDE",
            "MISSION",
            "AUTO.LOITER",
            "AUTO.RTL",
            "AUTO.LAND",
            "AUTO.TAKEOFF",
            "AUTO.FOLLOW_TARGET",
            "AUTO.PRECLAND"
        };
        
        // Check if the requested mode is supported
        bool mode_supported = false;
        for (const auto& mode : supported_modes)
        {
            if (req.custom_mode == mode)
            {
                mode_supported = true;
                break;
            }
        }
        
        if (!mode_supported)
        {
            ROS_WARN("[PerfectMavrosDrone] Unsupported mode: %s. Mode not changed.", req.custom_mode.c_str());
            res.mode_sent = false;
            return true;
        }
        
        // Update the current mode
        current_state_.mode = req.custom_mode;
        ROS_INFO("[PerfectMavrosDrone] Mode set to %s", req.custom_mode.c_str());
        
        // Return success
        res.mode_sent = true;
        return true;
    }
    
    bool armingCallback(mavros_msgs::CommandBool::Request& req,
                        mavros_msgs::CommandBool::Response& res)
    {
        ROS_INFO("[PerfectMavrosDrone] Received arming request: %s", req.value ? "ARM" : "DISARM");
        
        // Update the armed status
        current_state_.armed = req.value;
        
        if (req.value)
        {
            ROS_INFO("[PerfectMavrosDrone] Drone ARMED");
        }
        else
        {
            ROS_INFO("[PerfectMavrosDrone] Drone DISARMED");
        }
        
        // Return success
        res.success = true;
        res.result = 0;
        return true;
    }

private:
    /**
     * @brief Calculate quaternion from acceleration direction and yaw angle
     * @param ax Acceleration X in ENU frame (with gravity removed)
     * @param ay Acceleration Y in ENU frame (with gravity removed)
     * @param az Acceleration Z in ENU frame (with gravity removed)
     * @param yaw Yaw angle in radians
     * @return Quaternion representing the orientation
     * 
     * The acceleration vector (after removing gravity) represents the thrust direction,
     * which is the body z-axis direction in the world (ENU) frame.
     */
    geometry_msgs::Quaternion calculateOrientationFromAcceleration(double ax, double ay, double az, double yaw)
    {
        // ======= follow PX4 yaw definition and calculation method (body_x在水平面的投影与yaw方向一致， ENU and FLU frame) =========

        // std::cout << "[PerfectMavrosDrone] Calculating orientation from acceleration: "
        //           << "ax=" << ax << ", ay=" << ay << ", az=" << az << ", yaw=" << yaw * 180.0 / M_PI << " deg" << std::endl;


        /*========== wrong transform from body_z+yaw -> rot_mat/quaternion  =========

        // Acceleration vector (already with gravity removed)
        Eigen::Vector3d acc(ax, ay, az);
        double a_T = acc.norm();
        
        if (a_T < 0.01)  // No significant acceleration
        {
            // Use yaw only (hover attitude)
            Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            geometry_msgs::Quaternion q_msg;
            q_msg.w = q.w();
            q_msg.x = q.x();
            q_msg.y = q.y();
            q_msg.z = q.z();
            return q_msg;
        }
        
        // Body z-axis (thrust direction) in world frame
        Eigen::Vector3d zB = acc.normalized();
        
        // Desired heading direction (projection of body x-axis on horizontal plane)
        Eigen::Vector3d xC(std::cos(yaw), std::sin(yaw), 0.0);
        
        // Body y-axis: yB = zB × xC
        Eigen::Vector3d yB = zB.cross(xC).normalized();
        
        // Check if yB is valid (not near zero)
        if (yB.norm() < 0.01)
        {
            // Thrust is nearly vertical, use yaw only
            Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            geometry_msgs::Quaternion q_msg;
            q_msg.w = q.w();
            q_msg.x = q.x();
            q_msg.y = q.y();
            q_msg.z = q.z();
            return q_msg;
        }
        
        // Body x-axis: xB = yB × zB
        Eigen::Vector3d xB = yB.cross(zB);
        
        // Construct rotation matrix: R = [xB, yB, zB]
        // Each column represents a body axis in world coordinates
        Eigen::Matrix3d R;
        R << xB, yB, zB;
        
        // Convert to quaternion
        Eigen::Quaterniond q(R);
        
        // Convert to ROS message
        geometry_msgs::Quaternion q_msg;
        q_msg.w = q.w();
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();

        */

        // Acceleration vector (gravity already removed)
        Eigen::Vector3d acc(ax, ay, az);
        double a_T = acc.norm();

        // ===== 1. 无有效推力：yaw-only =====
        if (a_T < 0.01) {
            Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            geometry_msgs::Quaternion q_msg;
            q_msg.w = q.w();
            q_msg.x = q.x();
            q_msg.y = q.y();
            q_msg.z = q.z();
            return q_msg;
        }

        // ===== 2. body_z：推力方向（世界系）=====
        Eigen::Vector3d zB = acc.normalized();

        // ===== 3. PX4 的 y_C：yaw 定义的水平法向 =====
        // 对应 PX4: Vector3f y_C{-sinf(yaw), cosf(yaw), 0.f};
        Eigen::Vector3d yC(-std::sin(yaw), std::cos(yaw), 0.0);

        // ===== 4. body_x = y_C × body_z =====
        Eigen::Vector3d xB = yC.cross(zB);

        // 倒飞保护（PX4 同款）
        if (zB.z() < 0.0) {
            xB = -xB;
        }

        // ===== 5. 推力几乎水平：yaw 退化 =====
        if (std::abs(zB.z()) < 1e-6 || xB.norm() < 1e-6) {
            Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            geometry_msgs::Quaternion q_msg;
            q_msg.w = q.w();
            q_msg.x = q.x();
            q_msg.y = q.y();
            q_msg.z = q.z();
            return q_msg;
        }

        xB.normalize();

        // ===== 6. body_y = body_z × body_x =====
        Eigen::Vector3d yB = zB.cross(xB);

        // ===== 7. 构造旋转矩阵 R = [xB yB zB] =====
        Eigen::Matrix3d R;
        R.col(0) = xB;
        R.col(1) = yB;
        R.col(2) = zB;

        // ===== 8. 转 quaternion =====
        Eigen::Quaterniond q(R);

        geometry_msgs::Quaternion q_msg;
        q_msg.w = q.w();
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();

        // tf2::Quaternion q_tf2;
        // tf2::fromMsg(q_msg, q_tf2);
        // tf2::Matrix3x3 m(q_tf2);
        // double roll, pitch, yaw_rad;
        // m.getRPY(roll, pitch, yaw_rad);
        // double yaw_deg = yaw_rad * 180.0 / M_PI;
        // ROS_INFO("[PerfectMavrosDrone] Calculated orientation - Yaw: %.2f deg (%.2f rad)", yaw_deg, yaw_rad);

        // // 航向角（机体 x 轴在水平面投影）
        // double heading = atan2(xB.y(), xB.x()) * 180.0 / M_PI;
        // ROS_INFO("Heading from xB projection: %.2f deg", heading);
        
        return q_msg;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perfect_mavros_quadrotor_sim");
    
    PerfectMavrosDrone drone;
    
    ROS_INFO("[PerfectMavrosDrone] Node started. Waiting for setpoint commands...");
    
    ros::spin();
    
    return 0;
}
