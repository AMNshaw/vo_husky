#ifndef VOHUSKY_VIEWER_H
#define VOHUSKY_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include "vo_husky/common_include.h"
#include "vo_husky/frame.h"
#include "vo_husky/map.h"

namespace vo_husky {

/**
 * Viewer
 * 
 * Handles visualization using Pangolin and OpenCV.
 */
class Viewer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    // Set the map.
    void SetMap(Map::Ptr map) { map_ = map; }

    // Set the camera.
    void SetCamera(Camera::Ptr camera) { camera_ = camera; }

    // Close the viewer.
    void Close();

    // Add the current frame to be visualized.
    void AddCurrentFrame(Frame::Ptr current_frame);

    // Update the map for visualization.
    void UpdateMap();

   private:
    // Main loop for the viewer thread.
    void ThreadLoop();

    // Generate an image of the current frame.
    cv::Mat PlotFrameImage();

    // Plot a comparison between ground truth and estimated poses.
    void PlotPoseComparison(const SE3& groundtruth, const SE3& estimate);

    // Draw the 3D pose of a frame.
    void Draw3DPose(Frame::Ptr frame);

    // Draw the map points.
    void DrawMapPoints();

    // Adjust the view to follow the current frame.
    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    // Data members:
    Frame::Ptr current_frame_ = nullptr;    // Current frame for display
    Map::Ptr map_ = nullptr;                  // Map containing keyframes and landmarks
    Camera::Ptr camera_ = nullptr;            // Camera used for visualization

    cv::Mat canvas_;                        // Canvas for drawing images

    std::thread viewer_thread_;             // Thread running the viewer loop
    bool viewer_running_ = true;            // Flag indicating if the viewer is active

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;  // Active keyframes
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_; // Active landmarks
    bool map_updated_ = false;              // Flag indicating map updates

    std::mutex viewer_data_mutex_;          // Mutex protecting viewer data
};

}  // namespace vo_husky

#endif  // VOHUSKY_VIEWER_H
