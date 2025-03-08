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
 * Frontend
 * 
 * Estimates the current frame's pose, inserts keyframes, and activates backend optimization
 * when specific conditions are met.
 */
class Frontend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    // Add a frame and process it.
    bool AddFrame(Frame::Ptr frame);

    // Set the map.
    void SetMap(Map::Ptr map) { map_ = map; }

    // Set the backend.
    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

    // Set the viewer.
    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    // Get the current status of the frontend.
    FrontendStatus GetStatus() const { return status_; }

    // Set the camera.
    void SetCamera(Camera::Ptr camera) { camera_ = camera; }

   private:
    // Track features from the last frame. Returns the number of tracked points.
    int TrackLastFrame();

    // Estimate the pose between frames. Returns the number of inliers.
    int EstimatePose();

    // Reset the frontend when tracking is lost. Returns true if successful.
    bool Reset();

    // Perform pose-only bundle adjustment. Returns the number of inliers.
    int PoseOnlyBA();

    // Insert the current frame as a keyframe and add it to the backend. Returns true if successful.
    bool InsertKeyframe();

    // Initialize the frontend with RGB-D images in the current frame. Returns true if successful.
    bool RGBdInit();

    // Detect features in the current frame. Returns the number of detected keypoints.
    int DetectFeatures();

    // Build the initial map using a single image. Returns true if successful.
    bool BuildInitMap();

    // Compute new map points from the current frame. Returns the number of points.
    int CalculateNewMapPoints();

    // Set the features in the keyframe as new observations for the corresponding map points.
    void SetObservationsForKeyFrame();

    // Data members

    FrontendStatus status_ = FrontendStatus::INITING;  // Current status of the frontend

    Frame::Ptr current_frame_ = nullptr;  // Current frame
    Frame::Ptr last_frame_ = nullptr;     // Last frame
    Camera::Ptr camera_ = nullptr;        // Camera associated with the frontend

    Map::Ptr map_ = nullptr;                    // The map (owned by the system)
    std::shared_ptr<Backend> backend_ = nullptr;  // Backend module for optimization
    std::shared_ptr<Viewer> viewer_ = nullptr;    // Viewer module for visualization

    SE3 relative_motion_;  // Relative motion between current and last frame, used for initial pose guess

    int tracking_inliers_ = 0;                // Number of inliers from feature tracking
    std::vector<Feature::Ptr> inlier_features_pnp_;  // Inlier features used in pose estimation

    // Parameters
    int num_features_ = 200;                // Maximum number of features to detect
    int num_features_init_ = 100;           // Number of features for initialization
    int num_features_tracking_ = 90;        // Minimum number of features needed for tracking
    int num_features_tracking_bad_ = 15;    // Threshold for a bad tracking result
    int num_features_needed_for_keyframe_ = 70;  // Minimum features required to insert a new keyframe

    // Utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // OpenCV feature detector
};

}  // namespace vo_husky

#endif  // VOHUSKY_FRONTEND_H
