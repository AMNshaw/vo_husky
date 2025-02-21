#include "ros2_related/gazebo_gt_sub.hpp"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

Gazebo_gt_sub::Gazebo_gt_sub() : Node("gazebo_gt_sub") {
    // 訂閱 gazebo 的 model_states topic
    gz_modelStates_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        "gazebo/model_states", 10,
        std::bind(&Gazebo_gt_sub::gazebo_gt_cb, this, std::placeholders::_1)
    );
}

void Gazebo_gt_sub::gazebo_gt_cb(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    // 遍歷所有模型，尋找名稱為 "husky" 的模型

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "husky") {
            // 取得 husky 的位姿資訊
            const auto& pose = msg->pose[i];

            // 根據 geometry_msgs 的 quaternion（欄位順序為 x, y, z, w），
            // 建立 Eigen::Quaterniond 時建議用 (w, x, y, z) 的順序
            Eigen::Quaterniond q(pose.orientation.w,
                                 pose.orientation.x,
                                 pose.orientation.y,
                                 pose.orientation.z);

            // 建立平移向量
            Eigen::Vector3d t(pose.position.x,
                              pose.position.y,
                              pose.position.z);

            // 用 Sophus 封裝成 SE3d (旋轉和平移)
            Sophus::SE3d husky_pose_absolute = Sophus::SE3d(q, t);
            if(!init_){
                init_ = true;
                husky_pose_init_ = husky_pose_absolute;
            }
            husky_pose_ = husky_pose_init_.inverse()*husky_pose_absolute ;

            // 可透過 log 輸出位姿資訊進行除錯
            // RCLCPP_INFO(this->get_logger(), "Updated husky pose: translation [%f, %f, %f]",
            //             t.x(), t.y(), t.z());
            break;  // 找到 husky 後就跳出迴圈
        }
    }
}
