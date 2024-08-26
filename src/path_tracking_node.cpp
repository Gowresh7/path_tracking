#include <ros/ros.h>    
#include <nav_msgs/Path.h>  
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>  //for 
#include <tf/tf.h>  //for robot yaw 
#include <cmath>  // for M_PI
#include <visualization_msgs/Marker.h>

// Base class for Path Tracking Controllers
class PathTrackingController {
public:
    virtual void computeControl(const nav_msgs::Odometry& odom, ackermann_msgs::AckermannDrive& cmd_vel, const nav_msgs::Path& path, const geometry_msgs::PoseStamped& current_pose, double lookahead_distance, double wheelbase, double vehicle_speed) = 0;
    virtual ~PathTrackingController() = default;
};

// PurePursuit Implementation for Path Tracking
class PurePursuitController : public PathTrackingController {
public:
    void computeControl(const nav_msgs::Odometry& odom, ackermann_msgs::AckermannDrive& cmd_vel, const nav_msgs::Path& path, const geometry_msgs::PoseStamped& current_pose, double lookahead_distance, double wheelbase, double vehicle_speed) override {

        geometry_msgs::PoseStamped lookahead_point = getLookaheadPoint(path, current_pose, lookahead_distance);

        double steering_angle = computeSteeringAngle(current_pose, lookahead_point, lookahead_distance, wheelbase);

        cmd_vel.speed = vehicle_speed;
        cmd_vel.steering_angle = steering_angle;
    }

    geometry_msgs::PoseStamped getLookaheadPoint(const nav_msgs::Path& path, const geometry_msgs::PoseStamped &current_pose, double lookahead_distance)
    {
        geometry_msgs::PoseStamped lookahead_point;
        double min_distance_to_lookahead = std::numeric_limits<double>::max();
        double lower_bound = lookahead_distance - 0.3;
        double upper_bound = lookahead_distance + 0.3;

        double yaw = tf::getYaw(current_pose.pose.orientation);

        for (const auto &pose : path.poses)
        {
            double dx = pose.pose.position.x - current_pose.pose.position.x;
            double dy = pose.pose.position.y - current_pose.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            double path_heading = std::atan2(dy, dx);

            double angle_difference = std::abs(yaw - path_heading);

            if (distance >= lower_bound && distance <= upper_bound && angle_difference < M_PI_2)
            {
                lookahead_point = pose;
                break;
            }
            if (distance < min_distance_to_lookahead)
            {
                min_distance_to_lookahead = distance;
                lookahead_point = pose;
            }
        }

        return lookahead_point;
    }


     double computeSteeringAngle(const geometry_msgs::PoseStamped &current_pose, const geometry_msgs::PoseStamped &lookahead_point, double lookahead_distance, double wheelbase)
    {
        double dx = lookahead_point.pose.position.x - current_pose.pose.position.x;
        double dy = lookahead_point.pose.position.y - current_pose.pose.position.y;

        // Get the yaw angle of the robot
        double yaw = tf::getYaw(current_pose.pose.orientation);
        //ROS_INFO("%f", lookahead_point.pose.position.z);

        // Transform the lookahead point to the vehicle's frame of reference
        double transformed_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
        double transformed_y = std::cos(yaw) * dy - std::sin(yaw) * dx;

        double distance = std::sqrt(dx * dx + dy * dy);
        //double alpha = lookahead_point.pose.position.z - yaw;


        double steering_angle = std::atan2(2 * wheelbase * transformed_y, distance * distance);
        //steering_angle = steering_angle * 2;

        //steering_angle = steering_angle * steering_angle;
        steering_angle = std::max(-0.61, std::min(steering_angle, 0.61));
        //ROS_INFO("%f, %f", lookahead_point.pose.position.x, lookahead_point.pose.position.y);
        return steering_angle;
    }
};

enum class State {
    IDLE,
    TRACKING,
    GOAL_REACHED,
    ERROR
};

class PathTrackingNode {
public:
    State current_state_;
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher state_marker_pub_;
    nav_msgs::Path path_;
    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped current_pose_;
    
    ros::Time goal_reached_time_;
    double lookahead_distance_;
    double vehicle_speed_;
    double wheelbase_;
    std::string controller_type_str;

    bool isGoalReached(const geometry_msgs::PoseStamped &current_pose)
    {
        // Check if the robot is close enough to the final point in the path
        double dx = path_.poses.back().pose.position.x - current_pose.pose.position.x;
        double dy = path_.poses.back().pose.position.y - current_pose.pose.position.y;
        double distance_to_goal = std::sqrt(dx * dx + dy * dy);

        return distance_to_goal < 6.0;  // Goal is considered reached if within lookahead distance
    }

    PathTrackingNode() : current_state_(State::IDLE) {
        
        path_sub_ = nh_.subscribe("/gps_path", 10, &PathTrackingNode::pathCallback, this);
        odom_sub_ = nh_.subscribe("/gem/base_footprint/odom", 10, &PathTrackingNode::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 10);
        state_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("state_marker", 1);

        nh_.param("lookahead_distance", lookahead_distance_, 6.0);  // default value 1.0
        nh_.param("vehicle_speed", vehicle_speed_, 2.8);            // default value 1.0
        nh_.param("wheelbase", wheelbase_, 1.75); 
        nh_.param("controller_type", controller_type_str, std::string("PURE_PURSUIT"));

        

        // Initialize the chosen path tracking controller (can extend with other types)

        if (controller_type_str == "PURE_PURSUIT"){
            ROS_INFO("Loading Pure Pursuit Controller!");
            controller_ = std::make_unique<PurePursuitController>();
        }
        else {
            ROS_INFO("Unknown controller defined! Defaulting to PurePursuit");
            controller_ = std::make_unique<PurePursuitController>();
        }
        
        
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
        // Handle path data
        path_ = *path_msg;
        if (path_.poses.empty())
        {
            ROS_WARN("Path is empty!");
            current_state_ = State::ERROR;
            ROS_INFO("ERROR");
            return;
        }
        current_state_ = State::TRACKING;
        ROS_INFO("ROBOT IS FOLLOWING PATH");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        // Handle odometry data
        odom_ = *odom_msg;
        current_pose_.pose = odom_.pose.pose;
        current_pose_.header = odom_.header;
        //ROS_INFO(" Odom received");
        
    }

    void controlLoop() {
        ackermann_msgs::AckermannDrive cmd_vel_msg;

        ros::Time current_time;
        ros::Duration duration;


        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_footprint"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "controller_state";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 2.5;  // above the robot

        marker.scale.z = 0.5;  // text height
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;


        switch (current_state_) {
            case State::IDLE:
                // No movement, robot is idle
                goal_reached_time_ = ros::Time(0);
                marker.text = "IDLE";
                marker.color.r = 0.0;
                marker.color.g = 1.0;  // green
                
                break;

            case State::TRACKING:
                //ROS_INFO("ROBOT IS FOLLOWING PATH");
                controller_->computeControl(odom_, cmd_vel_msg, path_, current_pose_,lookahead_distance_, wheelbase_, vehicle_speed_);
                cmd_vel_pub_.publish(cmd_vel_msg);
                // Check if the goal is reached or if an error occurs
                if (isGoalReached(current_pose_)) {
                        ROS_INFO("Goal_Reached");
                        current_state_ = State::GOAL_REACHED;
                        ROS_INFO("ROBOT HAS REACHED GOAL");
                        goal_reached_time_ = ros::Time::now();
                }

                marker.text = "TRACKING";
                marker.color.r = 0.0;
                marker.color.b = 1.0;  // blue

                break;

            case State::GOAL_REACHED:
                // Stop the robot, goal has been reached
                //ROS_INFO("ROBOT HAS REACHED GOAL");
                cmd_vel_msg.speed = 0;
                cmd_vel_msg.steering_angle = 0;
                cmd_vel_pub_.publish(cmd_vel_msg);

                marker.text = "GOAL REACHED";
                marker.color.g = 1.0;  // green

                
                current_time = ros::Time::now();
                duration = current_time - goal_reached_time_;

                if (duration.toSec() >= 5.0) {
                    current_state_ = State::IDLE;
                }
                break;

            case State::ERROR:
                // Handle error state
                //ROS_INFO("ERROR");
                marker.text = "ERROR";
                marker.color.r = 1.0;  // red
                break;
            
        }
        state_marker_pub_.publish(marker);
    }
private:
    std::unique_ptr<PathTrackingController> controller_;  // Pointer to the path tracking controller
};

#ifndef RUNNING_TEST
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracking_node");
    PathTrackingNode node;
    ros::Rate loop_rate(10);  // 10 Hz loop rate, can be adjusted
    ROS_INFO("ROBOT IS IDLE");
    while (ros::ok()) {
        ros::spinOnce();
        node.controlLoop();
        loop_rate.sleep();
    }
    return 0;
}
#endif