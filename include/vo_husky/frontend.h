#pragma once
#ifndef VOHUSKY_FRONTEND_H
#define VOHUSKY_FRONTEND_H

#include <opencv2/features2d.hpp>
#include <string>

#include "vo_husky/common_include.h"
#include "vo_husky/frame.h"
#include "vo_husky/map.h"
#include "vo_husky/backend.h"
#include "vo_husky/viewer.h"
#include "vo_husky/feature.h"

namespace vo_husky {

class Backend;
class Viewer;

enum class FrontendStatus { INITING, TRACKING, LOST };

/**
 * 前端
 * 估计当前帧Pose，在满足关键帧条件时向地图加入关键帧并触发优化
 */
class Frontend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    /// 外部接口，添加一个帧并计算其定位结果
    bool AddFrame(Frame::Ptr frame);

    /// Set函数
    void SetMap(Map::Ptr map) { map_ = map; }

    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    FrontendStatus GetStatus() const { return status_; }

    void SetCamera(Camera::Ptr camera) { camera_ = camera;}

   private:
    /**
     * Track with last frame
     * @return num of tracked points
     */
    int TrackLastFrame();

    /**
     * Estimate pose between frames
     * @return num of inliers
     */
    int EstimatePose();

    /**
     * Reset when lost
     * @return true if success
     */
    bool Reset();

    /**
     * estimate current frame's pose
     * @return num of inliers
     */
    int PoseOnlyBA();

    /**
     * set current frame as a keyframe and insert it into backend
     * @return true if success
     */
    bool InsertKeyframe();

    /**
     * Try init the frontend with stereo images saved in current_frame_
     * @return true if success
     */
    bool RGBdInit();

    /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * @return
     */
    int DetectFeatures();

    /**
     * Build the initial map with single image
     * @return true if succeed
     */
    bool BuildInitMap();

    /**
     * Find the 3D MapPoints in current frame
     * @return num of points
     */
    int CalculateNewMapPoints();

    /**
     * Set the features in keyframe as new observation of the map points
     */
    void SetObservationsForKeyFrame();

    // data
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;  // 当前帧
    Frame::Ptr last_frame_ = nullptr;     // 上一帧
    Camera::Ptr camera_ = nullptr; //camera

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes
    std::vector<Feature::Ptr> inlier_features_pnp_;

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 90;
    int num_features_tracking_bad_ = 10;
    int num_features_needed_for_keyframe_ = 70;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv

    // backend
};

}  // namespace vo_husky

#endif  // VOHUSKY_FRONTEND_H
