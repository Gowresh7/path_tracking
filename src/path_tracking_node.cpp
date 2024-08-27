#include <ros/ros.h>                        //standard ROS library
#include <nav_msgs/Path.h>                  //for input path and control trajectory
#include <nav_msgs/Odometry.h>              //for current pose of robot
#include <ackermann_msgs/AckermannDrive.h>  //for controlling GEM vehicle
#include <tf/tf.h>                          //for robot yaw 
#include <cmath>                            //for M_PI
#include <visualization_msgs/Marker.h>      //for visualizing state of the robot 

// Base class for Path Tracking Controllers
class PathTrackingController {
public:
    //Virtual function that can be used by any sub classes
    virtual void computeControl(const nav_msgs::Odometry& odom, ackermann_msgs::AckermannDrive& cmd_vel, const nav_msgs::Path& path, const geometry_msgs::PoseStamped& current_pose, double lookahead_distance, double wheelbase, double vehicle_speed) = 0;
    virtual ~PathTrackingController() = default;
};
                                                //####################################################################
                                                //########## PurePursuit Implementation for Path Tracking   ##########
                                                //####################################################################
class PurePursuitController : public PathTrackingController {
public:
    //Implementation of the virtual control for Pure Pursuit Controller
    void computeControl(const nav_msgs::Odometry& odom, ackermann_msgs::AckermannDrive& cmd_vel, const nav_msgs::Path& path, const geometry_msgs::PoseStamped& current_pose, double lookahead_distance, double wheelbase, double vehicle_speed) override {

        //get the next point in the input path to travel to
        geometry_msgs::PoseStamped lookahead_point = getLookaheadPoint(path, current_pose, lookahead_distance);

        //get the steering angle required to reach the lookahead point
        double steering_angle = computeSteeringAngle(current_pose, lookahead_point, lookahead_distance, wheelbase);

        //poulate the steering angle and specified speed in the AckermannDrive message
        cmd_vel.speed = vehicle_speed;
        cmd_vel.steering_angle = steering_angle;
    }

    //Function to find the next point in the input path
    geometry_msgs::PoseStamped getLookaheadPoint(const nav_msgs::Path& path, const geometry_msgs::PoseStamped &current_pose, double lookahead_distance)
    {
        geometry_msgs::PoseStamped lookahead_point;     //lookahead point decalred as posestamped
        double min_distance_to_lookahead = std::numeric_limits<double>::max();  //minimum distance variable to ensure there is alway a target to follow
        double lower_bound = lookahead_distance - 0.3;                          //lower bound for the lookahead distance
        double upper_bound = lookahead_distance + 0.3;                          //upper bound for the lookahead distance

        double yaw = tf::getYaw(current_pose.pose.orientation);                 //get the robot heading from current pose

        for (const auto &pose : path.poses)                                     //loop to iterate over the path points
        {
            //find the distance between the current robot point and the current path point
            double dx = pose.pose.position.x - current_pose.pose.position.x;    
            double dy = pose.pose.position.y - current_pose.pose.position.y;    
            double distance = std::sqrt(dx * dx + dy * dy);             

            //find the heading of the current path point
            double path_heading = std::atan2(dy, dx);

            //find the angle difference between robot heading and the path heading
            double angle_difference = std::abs(yaw - path_heading);

            //distance should be within the bounds and the angle difference should be less than 90 degrees to avoid robot going backward
            if (distance >= lower_bound && distance <= upper_bound && angle_difference < M_PI_2)
            {
                //if the conditions satisfy, select the current path point as lookahead point and exit loop
                lookahead_point = pose;      
                break;
            }

            //if above condition is not satisfied, ensure there is a closer point to the robot to avoid jerky movements
            if (distance < min_distance_to_lookahead)
            {
                min_distance_to_lookahead = distance;
                lookahead_point = pose;
            }
        }

        return lookahead_point; //return the lookahead point for next step
    }


    //Function to calculate the required steering angle, based on the selected lookahead point
     double computeSteeringAngle(const geometry_msgs::PoseStamped &current_pose, const geometry_msgs::PoseStamped &lookahead_point, double lookahead_distance, double wheelbase)
    {
        double dx = lookahead_point.pose.position.x - current_pose.pose.position.x;
        double dy = lookahead_point.pose.position.y - current_pose.pose.position.y;

        // get the robot heading from current pose
        double yaw = tf::getYaw(current_pose.pose.orientation);
        

        // Transform the lookahead point to the vehicle frame
        double transformed_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
        double transformed_y = std::cos(yaw) * dy - std::sin(yaw) * dx;

        double distance = std::sqrt(dx * dx + dy * dy);
        //double alpha = lookahead_point.pose.position.z - yaw;

        //implement the pure pursuit equation to find the steering angle
        double steering_angle = std::atan2(2 * wheelbase * transformed_y, distance * distance);

        //restrict the steering angle within a range (taken as reference from GEM Simulator)
        steering_angle = std::max(-0.61, std::min(steering_angle, 0.61));
        //ROS_INFO("%f, %f", lookahead_point.pose.position.x, lookahead_point.pose.position.y);
        return steering_angle;  //return the steering angle 
    }
};

//Machine State Definitions for vehicle navigation
enum class State {
    IDLE,
    TRACKING,
    GOAL_REACHED,
    ERROR
};

//ROS Base Class for the Controller implementation. 
class PathTrackingNode {
public:
    State current_state_;                   //declare the robot current machine state

    ros::NodeHandle nh_;                    //Initialise the ros node

    //Subscribers and publishers 
    ros::Subscriber path_sub_;             
    ros::Subscriber odom_sub_;              
    ros::Publisher cmd_vel_pub_;            
    ros::Publisher trajectory_pub_;         
    ros::Publisher state_marker_pub_;      

    //Variable Declarations for the ros messages 
    nav_msgs::Path path_;                   
    nav_msgs::Odometry odom_;               
    geometry_msgs::PoseStamped current_pose_;   
    
    ros::Time goal_reached_time_;           //Used as a timeout to change the GOAL_REACHED state to IDLE

    //variable declarations for various parameters
    double lookahead_distance_;             
    double vehicle_speed_;
    double wheelbase_;
    double goal_tolerance;
    std::string controller_type_str;

    //Function that checks whether the robot has reached near the final path point
    bool isGoalReached(const geometry_msgs::PoseStamped &current_pose)
    {
        // Check if the robot is close enough to the final point in the path
        double dx = path_.poses.back().pose.position.x - current_pose.pose.position.x;
        double dy = path_.poses.back().pose.position.y - current_pose.pose.position.y;
        double distance_to_goal = std::sqrt(dx * dx + dy * dy);

        return distance_to_goal < goal_tolerance;  // Goal is considered reached if within the tolerance
    }

    //Function to generate the predicted path the robot will take
    nav_msgs::Path generatePredictedTrajectory(const ackermann_msgs::AckermannDrive& cmd_vel) {
        nav_msgs::Path predicted_path;
        predicted_path.header.frame_id = "world";
        predicted_path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped predicted_pose = current_pose_;

        double dt = 0.1;  // time step for prediction
        double total_time = 2.0;  // predict 2 seconds into the future

        for (double t = 0; t < total_time; t += dt) {
            double yaw = tf::getYaw(predicted_pose.pose.orientation);

            // Update the pose based on the current steering angle and speed
            predicted_pose.pose.position.x += cmd_vel.speed * dt * cos(yaw);
            predicted_pose.pose.position.y += cmd_vel.speed * dt * sin(yaw);
            yaw += (cmd_vel.speed / wheelbase_) * tan(cmd_vel.steering_angle) * dt;

            predicted_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            predicted_path.poses.push_back(predicted_pose);
        }

        return predicted_path;
    }

    //Initialise the constructor in Idle state
    PathTrackingNode() : current_state_(State::IDLE) {
        
        //Subscribe and map the callback functions
        path_sub_ = nh_.subscribe("/gps_path", 10, &PathTrackingNode::pathCallback, this);
        odom_sub_ = nh_.subscribe("/gem/base_footprint/odom", 10, &PathTrackingNode::odomCallback, this);

        //Broadcast the publishers
        cmd_vel_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 10);
        state_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("state_marker", 1);
        trajectory_pub_ = nh_.advertise<nav_msgs::Path>("controller_trajectory", 1);

        //Initialise the ros params
        nh_.param("lookahead_distance", lookahead_distance_, 6.0);  // default value 1.0
        nh_.param("vehicle_speed", vehicle_speed_, 2.8);            // default value 1.0
        nh_.param("wheelbase", wheelbase_, 1.75); 
        nh_.param("goal_tolerance", goal_tolerance, 6.0);
        nh_.param("controller_type", controller_type_str, std::string("PURE_PURSUIT"));

        

        // Initialize the chosen path tracking controller (can extend with other types)

        if (controller_type_str == "PURE_PURSUIT"){
            ROS_INFO("Loading Pure Pursuit Controller!");
            controller_ = std::make_unique<PurePursuitController>(); //create an instance of the Pure Pursuit Class
        }
        else {
            ROS_INFO("Unknown controller defined! Defaulting to PurePursuit");
            controller_ = std::make_unique<PurePursuitController>();
        }
        
        
    }

    //Input path callback function. Switches the machine state to TRACKING if a valid path is found and switches to ERROR for an invalid path
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

    //Odometry callback function to collect the robot's current pose
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        // Handle odometry data
        odom_ = *odom_msg;
        current_pose_.pose = odom_.pose.pose;
        current_pose_.header = odom_.header;
        //ROS_INFO(" Odom received");
        
    }

    //The main loop that gets executed based on the current robot state
    void controlLoop() {
        ackermann_msgs::AckermannDrive cmd_vel_msg; //declare ackermann commands for publishing

        //variable declarations for calculating timeout
        ros::Time current_time;                     
        ros::Duration duration;         

        //variable declaration for storing predicted trajectory
        nav_msgs::Path predicted_trajectory;

        //Text visualization for showcasing the current robot state
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_footprint"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "controller_state";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; //makes it readable by the viewer in Rviz, irrespective of the view perspective
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 2.5;  // display text right above the robot

        marker.scale.z = 0.5;  // text height
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;


        //Switch statement for the various robot states
        switch (current_state_) {
            case State::IDLE:
                // No movement, robot is idle
                goal_reached_time_ = ros::Time(0);

                //marker text for IDLE state
                marker.text = "IDLE";
                marker.color.r = 0.0;
                marker.color.g = 1.0;  // green
                
                break;

            case State::TRACKING:
                //Compute the steering angles with the given data and publish the ackermann commands
                controller_->computeControl(odom_, cmd_vel_msg, path_, current_pose_,lookahead_distance_, wheelbase_, vehicle_speed_);
                cmd_vel_pub_.publish(cmd_vel_msg);

                // Generate and publish the predicted trajectory
                predicted_trajectory = generatePredictedTrajectory(cmd_vel_msg);
                trajectory_pub_.publish(predicted_trajectory);

                // Check if the goal is reached 
                if (isGoalReached(current_pose_)) {
                        ROS_INFO("Goal_Reached");
                        current_state_ = State::GOAL_REACHED;
                        ROS_INFO("ROBOT HAS REACHED GOAL");
                        goal_reached_time_ = ros::Time::now();
                }

                //marker text for TRACKING state
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

                //marker text for GOAL_REACHED state
                marker.text = "GOAL REACHED";
                marker.color.g = 1.0;  // green

                //get the current time
                current_time = ros::Time::now();
                duration = current_time - goal_reached_time_;   //find the duration between goal reached time and current time

                //switch the state to IDLE if duration is more than 5 seconds
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
        //publish the text marker of current state
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