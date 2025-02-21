#include "ros2_related/camera_data_sub.hpp"

Camera_data_sub::Camera_data_sub() : Node("Camera_data_sub") {
        // Subscribe to the topic
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_realsense_sensor/camera_info", 10,
            std::bind(&Camera_data_sub::camera_info_cb_, this, std::placeholders::_1));

        image_color_sub_.subscribe(this, "/camera_realsense_sensor/image_raw");
        image_depth_sub_.subscribe(this, "/camera_realsense_sensor/depth/image_raw");

        sync_ = std::make_shared<Sync>(SyncPolicy(10), image_color_sub_, image_depth_sub_);
        sync_->registerCallback(std::bind(&Camera_data_sub::image_cb, this, std::placeholders::_1, std::placeholders::_2));
}

void Camera_data_sub::camera_info_cb_(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if(!init_info_)
        init_info_ = true;

    camera_info_sub_.reset();

    Vec3 t; t << -0.2438, 0, 0.88728;
    Mat33 R;
    Eigen::Quaterniond q(0.5, 0.5, -0.5, 0.5);
    R << 0, -1, 0,
        0, 0, -1,
        1, 0, 0;
    t = -R * t;

    camera_ = std::allocate_shared<vo_husky::Camera>(
        Eigen::aligned_allocator<vo_husky::Camera>(),
        msg->width, msg->height,
        msg->k[0], msg->k[4], msg->k[2], msg->k[5], 
        Sophus::SE3d(R, t));


    start_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
}

void Camera_data_sub::image_cb(const sensor_msgs::msg::Image::ConstSharedPtr& img_color_msg,
                                const sensor_msgs::msg::Image::ConstSharedPtr& img_depth_msg){
    if(!init_info_)
        return;
    
    std::lock_guard<std::mutex> lock(img_mutex_);
    try {
        if(!init_img_) 
            init_img_ = true;
        // Convert ROS2 messages to OpenCV images
        cv_bridge::CvImagePtr cv_color_ptr = cv_bridge::toCvCopy(img_color_msg, sensor_msgs::image_encodings::TYPE_8UC3);
        cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

        // Store in class variables or process as needed
        image_color_ = cv_color_ptr->image;
        image_depth_ = cv_depth_ptr->image;


        time_stamp_ = img_color_msg->header.stamp.sec + img_color_msg->header.stamp.nanosec*1e-9 - start_time_;
        // RCLCPP_INFO(this->get_logger(), "Timestamp: %f",time_stamp_);

        new_img_arrived_ = true;

        // RCLCPP_INFO(this->get_logger(), "Received new image");


    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

bool Camera_data_sub::Init(){
    if (!init_img_) return false;

    RCLCPP_INFO(this->get_logger(), "Initialize Camera Info:");
    std::cout << "Width: " << camera_->Width() << ", Height: " << camera_->Height() << "\n";
    std::cout << "Intrinsic Matrix:\n" << camera_->K() << "\n";
    std::cout << "Extrinsic:\n";
    std::cout << "q: "
            << camera_->Ext().unit_quaternion().x() << ", "
            << camera_->Ext().unit_quaternion().y() << ", "
            << camera_->Ext().unit_quaternion().z() << ", "
            << camera_->Ext().unit_quaternion().w() << std::endl;
    // std::cout << "R:\n" << camera_->Ext().rotationMatrix() << std::endl;
    std::cout << "t: "
            << camera_->Ext().translation().x() << ", "
            << camera_->Ext().translation().y() << ", "
            << camera_->Ext().translation().z() << std::endl;

    return true;   
}

vo_husky::Frame::Ptr Camera_data_sub::NextFrame(){
    std::lock_guard<std::mutex> lock(img_mutex_);
    if (!new_img_arrived_) return nullptr;
        
    auto new_frame = vo_husky::Frame::CreateFrame();
    new_frame->img_color_ = image_color_;
    cv::Mat img_grey;
    cv::cvtColor(image_color_, img_grey, cv::COLOR_BGR2GRAY);
    new_frame->img_grey_ = img_grey;
    new_frame->img_depth_ = image_depth_;
    new_frame->time_stamp_ = time_stamp_;
    return new_frame;
}

void Camera_data_sub::ResetImg(){
    std::lock_guard<std::mutex> lock(img_mutex_);
    new_img_arrived_ = false;
}