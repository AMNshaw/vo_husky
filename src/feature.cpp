#include "vo_husky/feature.h"

namespace vo_husky {

Feature::Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), kp_(kp) {}

}