#pragma once
#ifndef VOHUSKY_CAMERA_H
#define VOHUSKY_CAMERA_H

#include "vo_husky/common_include.h"

namespace vo_husky {

// Pinhole stereo camera model
class Camera {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Camera> Ptr;
    SE3 ext_;            // extrinsic, from stereo camera to single camera
    SE3 ext_inv_;  

    Camera();
    Camera(int width, int height,
            double fx, double fy, double cx, double cy, 
            SE3 ext);

    int Width() const {return width_;}
    int Height() const {return height_;}

    const SE3& Ext() const { return ext_; }
    // return intrinsic matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    // coordinate transform: world, camera, pixel
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_b2w);

    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_b2w);

    Vec2 camera2pixel(const Vec3 &p_c);

    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world(const Vec2 &p_p, double depth, const SE3 &T_b2w);

    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_b2w);

private:
    int width_ = 0, height_ = 0;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0; // Camera intrinsics

};

}  // namespace myslam
#endif  // MYSLAM_CAMERA_H
