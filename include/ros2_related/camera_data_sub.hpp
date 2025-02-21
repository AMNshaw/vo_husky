#pragma once

#ifndef CAMERA_DATA_SUB_H
#define CAMERA_DATA_SUB_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

#include "vo_husky/camera.h"
#include "vo_husky/frame.h"


class Camera_data_sub : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Camera_data_sub();

    vo_husky::Frame::Ptr NextFrame();

    vo_husky::Camera::Ptr GetCamera() { return camera_;}

    bool Init();

    void ResetImg();

private:
    void camera_info_cb_(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr& img_color_msg,
                                const sensor_msgs::msg::Image::ConstSharedPtr& img_depth_msg);

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_depth_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;

    double start_time_ = 0;
    double time_stamp_ = 0;    

    bool init_info_ = false;
    bool init_img_ = false;
    bool new_img_arrived_ = false;

    vo_husky::Camera::Ptr camera_ = nullptr;

    cv::Mat image_color_;
    cv::Mat image_depth_;

    std::mutex img_mutex_;
};

#endif