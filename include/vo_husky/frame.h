#pragma once

#ifndef VOHUSKY_FRAME_H
#define VOHUSKY_FRAME_H

#include "vo_husky/camera.h"
#include "vo_husky/common_include.h"

namespace vo_husky {

struct Feature;

/**
 * Frame 
 * 
 * Each frame is assigned a unique ID, and keyframes are also assigned a separate keyframe ID.
*/
struct Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;           // Unique ID of the frame
    unsigned long keyframe_id_ = 0;  // Unique ID of the keyframe (if applicable)
    bool is_keyframe_ = false;       // Flag indicating whether this frame is a keyframe
    double time_stamp_;              // Timestamp of the frame

    SE3 pose_est_;                   // Estimated pose: transformation from body frame to world frame (T_b2w)
    SE3 pose_gt_;                    // Ground truth pose: transformation from body frame to world frame (T_b2w)
    
    std::mutex mutex_pose_;          // Mutex to prevent race conditions when accessing/modifying the pose
    cv::Mat img_color_, img_grey_, img_depth_;  // Color, grayscale, and depth images

    std::vector<std::shared_ptr<Feature>> features_; // Extracted features from the image

   public:  
    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &img_color, const Mat &img_depth);

    SE3 Pose_EST() {
        std::unique_lock<std::mutex> lck(mutex_pose_);
        return pose_est_;
    }

    SE3 Pose_GT() {
        return pose_gt_;
    }

    void SetEstimatePose(const SE3 &pose_est) {
        std::unique_lock<std::mutex> lck(mutex_pose_);
        pose_est_ = pose_est;
    }

    void SetGroundTruthPose(const SE3 &pose_gt){
        pose_gt_ = pose_gt;
    }

    void ShowCurrPose(std::string prefix);

    // Create a new frame and assign a unique ID
    static std::shared_ptr<Frame> CreateFrame();

    // Set this frame as a keyframe and assign a keyframe ID
    void SetKeyFrame();
    
};

}  // namespace vo_husky

#endif  // VOHUSKY_FRAME_H
