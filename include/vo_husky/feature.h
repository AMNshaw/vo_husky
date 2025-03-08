#pragma once

#ifndef VOHUSKY_FEATURE_H
#define VOHUSKY_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "vo_husky/common_include.h"

namespace vo_husky {

struct Frame;
struct MapPoint;

/**
 * 2D Feature 
 * 
 * This structure represents a 2D feature detected in an RGB-D image. Each feature may be 
 * associated with a corresponding MapPoint in the 3D map.
*/
struct Feature {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Feature> Ptr;

    cv::KeyPoint kp_;                    // 2D keypoint representing the pixel location in the image
    std::weak_ptr<MapPoint> map_point_;  // The associated 3D map point, if available
    std::weak_ptr<Frame> frame_;         // The frame that contains this feature (use weak_ptr to prevent circular references)

    bool is_outlier_ = false;            // Flag indicating whether this feature is considered an outlier

   public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp);
};
}  // namespace vo_husky

#endif  // VOHUSKY_FEATURE_H
