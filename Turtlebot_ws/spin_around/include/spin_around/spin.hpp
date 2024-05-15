#pragma once
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>

#include "eigen3/Eigen/Dense"

#include "geometry_msgs/msg/pose_array.hpp" 
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>




class Spin : public rclcpp::Node {

    private:
        Eigen::Matrix4d marker2camera_Matrix;
        Eigen::Matrix4d camera2base_Matrix;
        Eigen::Matrix4d marker2base_Matrix;
        Eigen::Matrix4d rot_matrix1;  // -------------- // These two matrices maybe the the adjustment and calibration result
        Eigen::Matrix4d rot_matrix2;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr marker_sub;
    
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        rclcpp::TimerBase::SharedPtr spin_timer;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

        geometry_msgs::msg::Twist move_cmd; 


    public:
        Spin(const std::string& name);
        Eigen::Matrix4d quat2matrix(const Eigen::Quaterniond& quat, const Eigen::Vector3d& pos);
        void ar_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        void spin_cb();
};