#define RUNNING_TEST
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include "../src/path_tracking_node.cpp"  // Adjust the include path

class PathTrackingNodeTest : public ::testing::Test {
protected:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber cmd_vel_sub_;
    ackermann_msgs::AckermannDrive last_cmd_vel_;
    PathTrackingNode path_tracking_node_;

    void SetUp() override {
        // Initialize publishers and subscribers
        path_pub_ = nh_.advertise<nav_msgs::Path>("/gps_path", 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gem/base_footprint/odom", 10);
        cmd_vel_sub_ = nh_.subscribe("/gem/ackermann_cmd", 10, &PathTrackingNodeTest::cmdVelCallback, this);
    }

    void cmdVelCallback(const ackermann_msgs::AckermannDrive::ConstPtr& cmd_vel_msg) {
        ROS_INFO("AckermannDrive message received: speed = %f, steering_angle = %f", cmd_vel_msg->speed, cmd_vel_msg->steering_angle);
        last_cmd_vel_ = *cmd_vel_msg;
    }

    //Function to create path points for the etst cases
    nav_msgs::Path createSimplePath() {
        nav_msgs::Path path;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";

        for (int i = 0; i < 10; ++i) {
            pose.pose.position.x = i * 1.0;
            pose.pose.position.y = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
            path.poses.push_back(pose);
        }
        return path;
    }

    //Function to create desired current pose of the robot
    nav_msgs::Odometry createOdom(double x, double y, double yaw) {
        nav_msgs::Odometry odom;
        odom.header.seq = 1;

        odom.header.frame_id = "world";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        return odom;
    }
};

//Testing the transition from IDLE to TRACKING

TEST_F(PathTrackingNodeTest, TestTrackingState) {
    // Publish path and odom data, then verify controller output

    
    nav_msgs::Odometry odom = createOdom(0.0, 0.0, 0.0);
    odom_pub_.publish(odom);
    ros::Duration(1.0).sleep();
    odom_pub_.publish(odom);
    ros::spinOnce();
    ros::Duration(1.0).sleep();

    nav_msgs::Path path = createSimplePath();
    path_pub_.publish(path);
    ros::Duration(1.0).sleep();
    path_pub_.publish(path);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    ros::Duration(1.0).sleep();  // Allow time for the callback to be processed

    // Wait until the velocity command is published or timeout
    ros::Time start_time = ros::Time::now();
    while (ros::Time::now() - start_time < ros::Duration(2.0)) {
        ros::spinOnce();
        if (last_cmd_vel_.speed != 0.0) {
            break;
        }
        ros::Duration(0.1).sleep();
    }
    
    // Check that the controller output is published
    EXPECT_EQ(path_tracking_node_.current_state_, State::TRACKING);

}

//Testing transition from TRACKING to GOAL_REACHED
TEST_F(PathTrackingNodeTest, TestGoalReachedState) {
    
    path_tracking_node_.current_state_= State::TRACKING;

    // Simulate the robot being close to the goal
    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;

    path_tracking_node_.path_.poses.push_back(current_pose);
    current_pose.pose.position.x = 1.0;
    path_tracking_node_.path_.poses.push_back(current_pose);

    path_tracking_node_.odomCallback(boost::make_shared<nav_msgs::Odometry>(nav_msgs::Odometry()));
    path_tracking_node_.controlLoop();

    EXPECT_EQ(path_tracking_node_.current_state_, State::GOAL_REACHED);

}

//Testing the transition from IDLE to Error state for an empty path
TEST_F(PathTrackingNodeTest, TestErrorState) {
    // Publish a path and set the odometry close to the goal
    

    //nav_msgs::Odometry odom = createOdom(9.8, 0.0, 0.0);  // Close to the final point in the path
    //odom_pub_.publish(odom);
    //ros::Duration(1.0).sleep();
    //odom_pub_.publish(odom);

    nav_msgs::Path path;
    path_pub_.publish(path);
    ros::Duration(1.0).sleep();
    path_pub_.publish(path);



    ros::spinOnce();
    ros::Duration(1.0).sleep();  // Allow time for the callback to be processed

    // Verify that the robot stops when the goal is reached
    EXPECT_EQ(path_tracking_node_.current_state_, State::ERROR);
    //EXPECT_NEAR(last_cmd_vel_.speed, 0.0, 1e-3);
    //EXPECT_NEAR(last_cmd_vel_.steering_angle, 0.0, 1e-3);
}


//Testing if steering angle is close to zero if the lookahed point is exactly in the front
TEST_F(PathTrackingNodeTest, TestSteeringAngleStraight) {
   
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped lookahead_point;
    double lookahead_distance = 5.0;
    double wheelbase = 1.75;
    
    // Simulate a straight path
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0); // Facing straight

    lookahead_point.pose.position.x = 5.0;
    lookahead_point.pose.position.y = 0.0;

    PurePursuitController controller;
    double steering_angle = controller.computeSteeringAngle(current_pose, lookahead_point, lookahead_distance, wheelbase);
    
    EXPECT_NEAR(steering_angle, 0.0, 0.01);
}


//Testing if steering angle is maximum permissible value for a curved path
TEST_F(PathTrackingNodeTest, TestSteeringTurn) {

    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped lookahead_point;
    double lookahead_distance = 5.0;
    double wheelbase = 1.75;
    
    // Simulate a sharp turn
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0); // Facing straight

    lookahead_point.pose.position.x = 3.0;
    lookahead_point.pose.position.y = 3.0;

    PurePursuitController controller;
    double steering_angle = controller.computeSteeringAngle(current_pose, lookahead_point, lookahead_distance, wheelbase);
    
    EXPECT_TRUE(steering_angle > 0);
    EXPECT_LE(steering_angle, 0.61);
}

//Testing if correct lookahead points are identified for a curved path
TEST_F(PathTrackingNodeTest, TestSteeringCurve) {

    double lookahead_distance = 20.0;

    nav_msgs::Path path;
    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;

    // Add several points to simulate a curved path
    geometry_msgs::PoseStamped point1;
    point1.pose.position.x = 5.0;
    point1.pose.position.y = 0.0;
    path.poses.push_back(point1);

    geometry_msgs::PoseStamped point2;
    point2.pose.position.x = 10.0;
    point2.pose.position.y = 5.0;
    path.poses.push_back(point2);

    PurePursuitController controller;
    geometry_msgs::PoseStamped lookahead_point = controller.getLookaheadPoint(path, current_pose, 7.0);

    EXPECT_EQ(lookahead_point.pose.position.x, 10.0);
    EXPECT_EQ(lookahead_point.pose.position.y, 5.0);
}

//Testing for correct lookahead point in a straight line
TEST_F(PathTrackingNodeTest, TestLookAhead) {
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 10.0;
    pose.pose.position.y = 0.0;
    path.poses.push_back(pose);

    PurePursuitController controller;
    geometry_msgs::PoseStamped lookahead_point = controller.getLookaheadPoint(path, pose, 5.0);
    
    EXPECT_EQ(lookahead_point.pose.position.x, 10.0);
    EXPECT_EQ(lookahead_point.pose.position.y, 0.0);
}


//Testing for goal reached function
TEST_F(PathTrackingNodeTest, TestIsGoalReached) {
    
    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;

    path_tracking_node_.path_.poses.push_back(current_pose);
    current_pose.pose.position.x = 100.0;  // far away goal
    path_tracking_node_.path_.poses.push_back(current_pose);

    current_pose.pose.position.x = 0.0;

    EXPECT_FALSE(path_tracking_node_.isGoalReached(current_pose));

   
}


//Testing whether ackermann commands are published in IDLE state
TEST_F(PathTrackingNodeTest, TestIdleControls) {

    path_tracking_node_.current_state_ = State::IDLE;



    path_tracking_node_.controlLoop();
    ros::spinOnce();

    EXPECT_EQ(last_cmd_vel_.speed, 0);
    EXPECT_EQ(last_cmd_vel_.steering_angle, 0);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_tracking_node_test");
    testing::InitGoogleTest(&argc, argv);
    
    return RUN_ALL_TESTS();
}
