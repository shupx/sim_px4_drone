/**
 * @file perfect_mavros_quadrotor_sim_node.cpp
 * @brief Perfect MAVROS drone simulator - directly maps setpoint to state
 * @author Peixuan Shu 
 * @version 1.0
 * @date Jan 2026
 * 
 * This node creates an ideal drone simulation that:
 * - Subscribes to mavros/setpoint_raw/local
 * - Publishes desired position/velocity/attitude directly to:
 *   - mavros/local_position/pose
 *   - mavros/local_position/velocity_local
 *   - mavros/local_position/odom
 * - Publishes mavros/state with armed=true and mode=OFFBOARD
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class PerfectMavrosDrone
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // Subscribers
    ros::Subscriber setpoint_sub_;
    
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
    
public:
    PerfectMavrosDrone() : nh_private_("~")
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
        
        // Initialize subscribers
        setpoint_sub_ = nh_.subscribe(setpoint_topic, 10, 
            &PerfectMavrosDrone::setpointCallback, this);
        
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
        ros::Time now = ros::Time::now();
        
        // Update pose based on setpoint
        current_pose_.header.stamp = now;
        
        // Position setpoint
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_PX)
        {
            // Position X ignored, keep current
        }
        else
        {
            current_pose_.pose.position.x = msg->position.x;
        }
        
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_PY)
        {
            // Position Y ignored, keep current
        }
        else
        {
            current_pose_.pose.position.y = msg->position.y;
        }
        
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_PZ)
        {
            // Position Z ignored, keep current
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
        
        // Attitude setpoint (yaw)
        if (msg->type_mask & mavros_msgs::PositionTarget::IGNORE_YAW)
        {
            // Yaw ignored, keep current orientation
        }
        else
        {
            // Convert yaw to quaternion
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, msg->yaw);
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perfect_mavros_quadrotor_sim");
    
    PerfectMavrosDrone drone;
    
    ROS_INFO("[PerfectMavrosDrone] Node started. Waiting for setpoint commands...");
    
    ros::spin();
    
    return 0;
}
