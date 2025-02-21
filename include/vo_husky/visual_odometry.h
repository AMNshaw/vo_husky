#pragma once
#ifndef VOHUSKY_VISUAL_ODOMETRY_H
#define VOHUSKY_VISUAL_ODOMETRY_H

// #include "vo_husky/backend.h"
#include "vo_husky/common_include.h"
#include "vo_husky/frontend.h"
#include "vo_husky/viewer.h"
#include "vo_husky/camera.h"
#include "vo_husky/frame.h"

namespace vo_husky {


class VisualOdometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<VisualOdometry> Ptr;

    /// constructor with config file
    VisualOdometry(Camera::Ptr camera);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * start vo in the dataset
     */
    void End();


    bool step(Frame::Ptr newFrame);

    /**
     * Make a step forward in dataset
     */

    /// 获取前端状态
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
    bool inited_ = false;
    std::string config_file_path_;

    Camera::Ptr camera_ = nullptr;
    Frontend::Ptr frontend_ = nullptr;
    // Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

};
}  // namespace vo_husky

#endif  // VOHUSKY_VISUAL_ODOMETRY_H
