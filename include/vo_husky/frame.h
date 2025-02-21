#pragma once

#ifndef VOHUSKY_FRAME_H
#define VOHUSKY_FRAME_H

#include "vo_husky/camera.h"
#include "vo_husky/common_include.h"

namespace vo_husky {

// forward declare
struct Feature;

/**
 * Frame
 * Asign ID to every frame, also asign ID to keyframe
 */
struct Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;           // id of this frame
    unsigned long keyframe_id_ = 0;  // id of key frame
    bool is_keyframe_ = false;       // if this frame is keyframe
    double time_stamp_;              // 
    SE3 pose_est_;                       // Transformation from camera frame to world frame
    SE3 pose_gt_;                       // Transformation from camera frame to world frame
    std::mutex mutex_pose_;          // pose locker
    cv::Mat img_color_, img_grey_, img_depth_;  // color and depth image

    // extracted features in image
    std::vector<std::shared_ptr<Feature>> features_;

   public:  // data members
    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &img_color, const Mat &img_depth);

    // set and get pose, thread safe
    SE3 Pose_EST();

    SE3 Pose_GT();

    void ShowCurrPose(std::string prefix);

    void SetEstimatePose(const SE3 &pose_est);

    void SetGroundTruthPose(const SE3 &pose_gt);

    // set keyframe and its id
    void SetKeyFrame();

    // create a frame and asign id 
    static std::shared_ptr<Frame> CreateFrame();
};

}  // namespace myslam

#endif  // MYSLAM_FRAME_H
