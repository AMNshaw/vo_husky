#pragma once

#ifndef GAZEBO_GT_SUB_H
#define GAZEBO_GT_SUB_H

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <sophus/se3.hpp> 


class Gazebo_gt_sub : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Gazebo_gt_sub();

    bool Init() { return init_;}
    Sophus::SE3d HuskyPose() { return husky_pose_; }

    Sophus::SE3d husky_pose_;
    Sophus::SE3d husky_pose_init_;

private:
    void gazebo_gt_cb(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr gz_modelStates_sub_;

    bool init_ = false;
};

#endif