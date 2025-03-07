#include <rclcpp/rclcpp.hpp>
#include "ros2_related/camera_data_sub.hpp"
#include "ros2_related/gazebo_gt_sub.hpp"

#include "vo_husky/visual_odometry.h"

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto node_cam = std::make_shared<Camera_data_sub>();
    auto node_gt = std::make_shared<Gazebo_gt_sub>();

    RCLCPP_INFO(rclcpp::get_logger("main_node"), "Creating subscriber spin thread...");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_cam);
    executor.add_node(node_gt);
    
    
    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    RCLCPP_INFO(rclcpp::get_logger("main_node"), "Subscriber spin thread created");
    RCLCPP_INFO(rclcpp::get_logger("main_node"), "Waiting for subscriber initialization");

    rclcpp::WallRate loop_rate(60);
    while (rclcpp::ok()) {
        int init = 0;
        if (!node_cam->Init()){
        }
        else init++;
        if (!node_gt->Init()){
        }
        else init++;
        if(init == 2) break;
        loop_rate.sleep();
    }
    RCLCPP_INFO(node_cam->get_logger(), "Subscriber initialized");
    RCLCPP_INFO(node_gt->get_logger(), "Subscriber initialized");
    vo_husky::VisualOdometry::Ptr vo = std::make_shared<vo_husky::VisualOdometry>(
        node_cam->GetCamera());

    if (!vo->Init()){
        RCLCPP_ERROR(rclcpp::get_logger("main_node"), "Visual Odometry initialization failed");
        return -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("main_node"), "Visual Odometry initialized");

    while (rclcpp::ok()) {
        vo_husky::Frame::Ptr new_frame = node_cam->NextFrame();
        if (new_frame == nullptr) {
            // std::cout << "Pass\n";
            continue;
        }
        new_frame->SetGroundTruthPose(node_gt->HuskyPose());

        if(!vo->step(new_frame)){
            RCLCPP_ERROR(node_cam->get_logger(), "Visual Odometry failed");
        }
        node_cam->ResetImg();
        loop_rate.sleep();
    }
    vo->End();

    // 結束前關閉 ROS 並等待 spin 線程結束
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
