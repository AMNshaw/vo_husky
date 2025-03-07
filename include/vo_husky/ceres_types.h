#pragma once
#ifndef CERES_TYPES
#define CERES_TYPES

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>

#include <Eigen/Geometry>
#include <iostream>
#include "vo_husky/common_include.h"
#include "vo_husky/camera.h"

class SophusSE3Manifold : public ceres::Manifold {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // SE3 李代數空間自由度
    static const int DOF = 6;

    // Ceres 會呼叫 AmbientSize() 和 TangentSize() 來獲得維度
    int AmbientSize() const override { return DOF; }
    int TangentSize() const override { return DOF; }

    // Plus: x + delta -> x_plus_delta
    // x, delta, x_plus_delta 全都是 double[6]
    // 分別對應 SE3 的李代數向量 (用 Sophus exp/log)
    bool Plus(const double* x,
            const double* delta,
            double* x_plus_delta) const override {
    // 將 x 和 delta 映射為 6 維向量
        Eigen::Map<const Vec6> se3(x);
        Eigen::Map<const Vec6> se3_delta(delta);

        // 使用 Sophus 指數映射，轉成 SE3
        SE3 T = SE3::exp(se3);
        SE3 T_delta = SE3::exp(se3_delta);

        // 假設是右乘更新
        SE3 T_updated = T_delta * T;

        // 把更新後的 SE3 再用 log 映射回 6 維
        Vec6 se3_updated = T_updated.log();

        // 寫回 output
        Eigen::Map<Vec6> out_map(x_plus_delta);
        out_map = se3_updated;
        return true;
    }

    bool PlusJacobian(const double* /*x*/, double* jacobian) const override {
    // 这里返回单位矩阵作为近似
        const int DOF = 6;
        for (int i = 0; i < DOF * DOF; i++) {
            jacobian[i] = 0.0;
        }
        for (int i = 0; i < DOF; i++) {
            jacobian[i * DOF + i] = 1.0;
        }
        return true;
    }


    bool Minus(const double* y,
             const double* x,
             double* y_minus_x) const override {
        // 若不提供解析解:
        return false;
    }

    bool MinusJacobian(const double* /*x*/,
                        double* /*jacobian*/) const override {
        return false;  // 用數值微分
    }

};


// 我們的CostFunction: 2維輸出, 6維參數(AngleAxis + t)
class ReprojectionErrCeres : public ceres::SizedCostFunction<2, 6> {
public:
    // 輸入: 3D點 pos3d_, 內參 camera_->K(), 觀測像素 measurement_
    // 在這裡我們把需要的量都存下來
    ReprojectionErrCeres(const Eigen::Vector3d& pos3d,
                           const Eigen::Vector2d& measurement,
                           const vo_husky::Camera::Ptr& camera)
        : pos3d_(pos3d), measurement_(measurement), camera_(camera) {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override
    {
        // -------------------------
        // (1) 取出參數 Pose_EST
        // -------------------------
        const double* pose = parameters[0];
        // pose[0..2] = angle-axis, pose[3..5] = translation

        Vec6 xi; xi << pose[0], pose[1], pose[2], pose[3], pose[4], pose[5];
        SE3 T_b2w = SE3::exp(xi);
        Vec3 p_cam = camera_->world2camera(pos3d_, T_b2w);

        // 將 R, t 應用到 3D點: pos_cam = R * pos3d + t
        double X = p_cam(0);
        double Y = p_cam(1);
        double Z = p_cam(2);

        // 投影: x' = fx * X/Z + cx,  y' = fy * Y/Z + cy
        double fx = camera_->K()(0,0);
        double fy = camera_->K()(1,1);
        double cx = camera_->K()(0,2);
        double cy = camera_->K()(1,2);

        double invz = 1.0 / (Z + 1e-12); // 避免除0
        double proj_x = fx * (X * invz) + cx;
        double proj_y = fy * (Y * invz) + cy;

        // residual = (測量值 - 預測值)
        residuals[0] = measurement_(0) - proj_x;
        residuals[1] = measurement_(1) - proj_y;

        // -------------------------
        // (2) 若需要jacobians, 就填
        // -------------------------
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            double* jac = jacobians[0];  // (2 x 6) row-major

            double z_inv = invz;
            double z_inv2 = z_inv * z_inv;
 
            jac[0] = -fx * z_inv;                  // d_res0/d_pose0
            jac[1] = 0;                            // d_res0/d_pose1
            jac[2] = fx * X * z_inv2;              // d_res0/d_pose2
            jac[3] = fx * Y * z_inv2;              // d_res0/d_pose3
            jac[4] = -fx - fx * X * X * z_inv2;    // d_res0/d_pose4
            jac[5] = fx * Y * z_inv;               // d_res0/d_pose5

            // Row1
            jac[6] = 0;                            // d_res1/d_pose0
            jac[7] = -fy * z_inv;                  // d_res1/d_pose1
            jac[8] = fy * Y * z_inv2;              // d_res1/d_pose2
            jac[9] = fy + fy * Y * Y * z_inv2;     // d_res1/d_pose3
            jac[10] = -fy * X * Y * z_inv2;        // d_res1/d_pose4
            jac[11] = -fy * X * z_inv;             // d_res1/d_pose5

        }

        return true;
    }

    static double ComputeError(const double* pose, 
                                const Vec3& pt_world, 
                                const Vec2& measurement,
                                const vo_husky::Camera::Ptr& camera){
        Vec6 xi; xi << pose[0], pose[1], pose[2], pose[3], pose[4], pose[5];
        SE3 T_b2w = SE3::exp(xi);
        Vec3 p_cam = camera->world2camera(pt_world, T_b2w);

        // std::cout << "pose:\n" << xi << "\n";
        // std::cout << "pt_world:\n" << pt_world << "\n";
        // std::cout << "p_cam:\n" << p_cam << "\n";
        // std::cout << "measurement:\n" << measurement << "\n";

        double invz = 1.0 / (p_cam(2) + 1e-12);
        double proj_x = camera->K()(0,0) * (p_cam(0) * invz) + camera->K()(0,2);
        double proj_y = camera->K()(1,1) * (p_cam(1) * invz) + camera->K()(1,2);

        // std::cout << "proj_x: " << proj_x << "proj_y: " << proj_y << "\n";

        double dx = measurement(0) - proj_x;
        double dy = measurement(1) - proj_y;
        return dx * dx + dy * dy;
    }

private:
    Vec3  pos3d_;       // 3D point
    Vec2  measurement_; // 2D measurement
    vo_husky::Camera::Ptr camera_;
};

#endif