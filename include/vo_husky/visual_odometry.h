#pragma once
#ifndef VOHUSKY_VISUAL_ODOMETRY_H
#define VOHUSKY_VISUAL_ODOMETRY_H

#include "vo_husky/backend.h"
#include "vo_husky/common_include.h"
#include "vo_husky/frontend.h"
#include "vo_husky/viewer.h"
#include "vo_husky/camera.h"
#include "vo_husky/frame.h"

namespace vo_husky {

/**
 * VisualOdometry
 *
 * Manages the visual odometry process by integrating the frontend, backend,
 * and viewer modules. It handles the initialization, processing of frames,
 * and closing of the visual odometry system.
 */
class VisualOdometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<VisualOdometry> Ptr;

    // Constructor: Initialize VisualOdometry with a camera.
    VisualOdometry(Camera::Ptr camera);

    /**
     * Initialize the VisualOdometry system.
     * @return true if initialization is successful.
     */
    bool Init();

    /**
     * Close the Visual Odometry system.
     */
    void End();

    /**
     * Process the next frame.
     * @param newFrame The new frame to be processed.
     * @return true if the step is successful.
     */
    bool step(Frame::Ptr newFrame);

    // Get the current status of the frontend.
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
    bool inited_ = false;            // Flag to indicate if the system has been initialized.
    std::string config_file_path_;   // Path to configuration file.

    Camera::Ptr camera_ = nullptr;   // Camera used in the system.
    Frontend::Ptr frontend_ = nullptr; // Frontend module for tracking.
    Backend::Ptr backend_ = nullptr;   // Backend module for optimization.
    Map::Ptr map_ = nullptr;           // Map that holds keyframes and landmarks.
    Viewer::Ptr viewer_ = nullptr;     // Viewer module for visualization.
};

}  // namespace vo_husky

#endif  // VOHUSKY_VISUAL_ODOMETRY_H
