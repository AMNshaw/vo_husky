#pragma once
#ifndef VOHUSKY_CAMERA_H
#define VOHUSKY_CAMERA_H

#include "vo_husky/common_include.h"

namespace vo_husky {

/**
 * RGBD Camera
 * 
 * Uses the pinhole camera model.
 * Handles the transformation from body to camera, and provides
 * various coordinate conversion functions.
 */
class Camera {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Camera> Ptr;
    
    SE3 ext_;     // Extrinsic transformation: from body to camera
    SE3 ext_inv_; // Inverse extrinsic transformation

    Camera();
    Camera(int width, int height,
           double fx, double fy, double cx, double cy, 
           SE3 ext);

    // Get image width.
    int Width() const { return width_; }
    
    // Get image height.
    int Height() const { return height_; }

    // Get the extrinsic transformation.
    const SE3& Ext() const { return ext_; }
    
    // Return the camera intrinsic matrix.
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_,
             0, fy_, cy_,
             0, 0, 1;
        return k;
    }

    // Transform a point from world coordinates to camera coordinates.
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_b2w);

    // Transform a point from camera coordinates to world coordinates.
    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_b2w);

    // Project a point from camera coordinates to pixel coordinates.
    Vec2 camera2pixel(const Vec3 &p_c);

    // Convert pixel coordinates to camera coordinates (with depth, default is 1).
    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    // Convert pixel coordinates to world coordinates.
    Vec3 pixel2world(const Vec2 &p_p, double depth, const SE3 &T_b2w);

    // Project a point from world coordinates to pixel coordinates.
    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_b2w);

private:
    int width_ = 0, height_ = 0;         // Image dimensions
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0; // Camera intrinsic parameters
};

}  // namespace vo_husky

#endif  // VOHUSKY_CAMERA_H
