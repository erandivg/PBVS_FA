#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamValue.h>
#include <Eigen/Dense>
#include "vision_utils.h"
#include <std_msgs/Float64MultiArray.h>

ros::Publisher vel_pub, accel_pub;
mavros_msgs::State current_state; 
Vector3D_t Uv,Uw;


void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void UvCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) { 
    if (msg->data.size() != 3) {
        ROS_WARN("The translation message must contain 3 elements.");
        return;
    }

    // Extract the translation vector from the message and assign it to Uv
    Uv(0) = msg->data[0];  // x
    Uv(1) = msg->data[1];  // y
    Uv(2) = msg->data[2];  // z

}

void UwCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) { 
    if (msg->data.size() != 3) {
        ROS_WARN("The translation message must contain 3 elements.");
        return;
    }

    Uw(0) = msg->data[0];  // x
    Uw(1) = msg->data[1];  // y
    Uw(2) = msg->data[2];  // z

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber Uv_sub    = nh.subscribe("/Uv", 10, UvCallback);
    ros::Subscriber Uw_sub    = nh.subscribe("/Uw", 10, UwCallback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    

    ros::Rate rate(30.0);

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped vel_msg;

    bool paramsSet = false;
    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }


        ros::Duration elapsed_time = ros::Time::now() - start_time;

        if (elapsed_time <= ros::Duration(15.0)) {
            vel_msg.twist.linear.x = 0.0;
            vel_msg.twist.linear.y = 0.0;
            vel_msg.twist.linear.z = 1.8;
            vel_msg.twist.angular.x = 0.;
            vel_msg.twist.angular.y = 0.;
            vel_msg.twist.angular.z = 0.;
            vel_pub.publish(vel_msg);
        } else {
            vel_msg.twist.linear.x = -Uv(0);
            vel_msg.twist.linear.y = -Uv(2);
            vel_msg.twist.linear.z = -Uv(1);
            vel_msg.twist.angular.x = -Uw(0);
            vel_msg.twist.angular.y = -Uw(2);
            vel_msg.twist.angular.z = Uw(1);
            vel_pub.publish(vel_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
