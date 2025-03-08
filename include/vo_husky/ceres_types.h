#pragma once
#ifndef CERES_TYPES
#define CERES_TYPES

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>

#include <Eigen/Geometry>
#include <iostream>
#include "vo_husky/common_include.h"
#include "vo_husky/camera.h"

/**
 * Manifold for SE3 representation in Ceres.
 * Defines the SE3 Lie algebra space used in optimization.
 */
class SophusSE3Manifold : public ceres::Manifold {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const int DOF = 6;  // Degrees of freedom in SE3 Lie algebra.

    // Ceres calls AmbientSize() and TangentSize() to determine dimensions.
    int AmbientSize() const override { return DOF; }
    int TangentSize() const override { return DOF; }

    // Plus operation: x + delta -> x_plus_delta
    // x, delta, and x_plus_delta are all double[6] representing SE3 in Lie algebra.
    bool Plus(const double* x,
              const double* delta,
              double* x_plus_delta) const override {
        // Map x and delta to 6D vectors.
        Eigen::Map<const Vec6> se3(x);
        Eigen::Map<const Vec6> se3_delta(delta);

        // Convert to SE3 using the exponential map.
        SE3 T = SE3::exp(se3);
        SE3 T_delta = SE3::exp(se3_delta);

        // Assume left multiplication update.
        SE3 T_updated = T_delta * T;

        // Convert updated SE3 back to Lie algebra using the logarithm map.
        Vec6 se3_updated = T_updated.log();

        // Write the result to output.
        Eigen::Map<Vec6> out_map(x_plus_delta);
        out_map = se3_updated;
        return true;
    }

    // Compute the Jacobian of the Plus operation.
    bool PlusJacobian(const double* /*x*/, double* jacobian) const override {
        // Return an identity matrix as an approximation.
        for (int i = 0; i < DOF * DOF; i++) {
            jacobian[i] = 0.0;
        }
        for (int i = 0; i < DOF; i++) {
            jacobian[i * DOF + i] = 1.0;
        }
        return true;
    }

    // Minus operation: computes the difference between y and x.
    bool Minus(const double* y,
               const double* x,
               double* y_minus_x) const override {
        return false;  // No analytic solution provided.
    }

    bool MinusJacobian(const double* /*x*/, double* /*jacobian*/) const override {
        return false;  // Use numerical differentiation.
    }
};

/**
 * Cost function for reprojection error in Ceres.
 * 2D output, 6D parameter space (AngleAxis + translation).
 */
class ReprojectionErrCeres : public ceres::SizedCostFunction<2, 6> {
public:

    //Constructor that stores the 3D point, camera intrinsics, and the observed pixel location.
    ReprojectionErrCeres(const Eigen::Vector3d& pos3d,
                         const Eigen::Vector2d& measurement,
                         const vo_husky::Camera::Ptr& camera)
        : pos3d_(pos3d), measurement_(measurement), camera_(camera) {}

    // Evaluate function computing residuals and optionally Jacobians.
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override
    {
        // -------------------------
        // (1) Retrieve pose parameters.
        // -------------------------
        const double* pose = parameters[0];
        // pose[0..2] = angle-axis, pose[3..5] = translation.

        Vec6 xi;
        xi << pose[0], pose[1], pose[2], pose[3], pose[4], pose[5];
        SE3 T_b2w = SE3::exp(xi);
        Vec3 p_cam = camera_->world2camera(pos3d_, T_b2w);

        double X = p_cam(0);
        double Y = p_cam(1);
        double Z = p_cam(2);

        // Projection: x' = fx * X/Z + cx, y' = fy * Y/Z + cy
        double fx = camera_->K()(0,0);
        double fy = camera_->K()(1,1);
        double cx = camera_->K()(0,2);
        double cy = camera_->K()(1,2);

        double invz = 1.0 / (Z + 1e-12);  // Avoid division by zero.
        double proj_x = fx * (X * invz) + cx;
        double proj_y = fy * (Y * invz) + cy;

        // Compute residuals (measured - predicted).
        residuals[0] = measurement_(0) - proj_x;
        residuals[1] = measurement_(1) - proj_y;

        // -------------------------
        // (2) Compute Jacobians if needed.
        // -------------------------
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            double* jac = jacobians[0];  // (2 x 6) row-major order.

            double z_inv = invz;
            double z_inv2 = z_inv * z_inv;

            jac[0] = -fx * z_inv;                  // d_res0/d_pose0
            jac[1] = 0;                            // d_res0/d_pose1
            jac[2] = fx * X * z_inv2;              // d_res0/d_pose2
            jac[3] = fx * Y * z_inv2;              // d_res0/d_pose3
            jac[4] = -fx - fx * X * X * z_inv2;    // d_res0/d_pose4
            jac[5] = fx * Y * z_inv;               // d_res0/d_pose5

            jac[6] = 0;                            // d_res1/d_pose0
            jac[7] = -fy * z_inv;                  // d_res1/d_pose1
            jac[8] = fy * Y * z_inv2;              // d_res1/d_pose2
            jac[9] = fy + fy * Y * Y * z_inv2;     // d_res1/d_pose3
            jac[10] = -fy * X * Y * z_inv2;        // d_res1/d_pose4
            jac[11] = -fy * X * z_inv;             // d_res1/d_pose5
        }

        return true;
    }

    /**
     * Compute the reprojection error for a given pose.
     */
    static double ComputeError(const double* pose, 
                               const Vec3& pt_world, 
                               const Vec2& measurement,
                               const vo_husky::Camera::Ptr& camera) {
        Vec6 xi;
        xi << pose[0], pose[1], pose[2], pose[3], pose[4], pose[5];
        SE3 T_b2w = SE3::exp(xi);
        Vec3 p_cam = camera->world2camera(pt_world, T_b2w);

        double invz = 1.0 / (p_cam(2) + 1e-12);
        double proj_x = camera->K()(0,0) * (p_cam(0) * invz) + camera->K()(0,2);
        double proj_y = camera->K()(1,1) * (p_cam(1) * invz) + camera->K()(1,2);

        double dx = measurement(0) - proj_x;
        double dy = measurement(1) - proj_y;
        return dx * dx + dy * dy;
    }

private:
    Vec3 pos3d_;           // 3D world point.
    Vec2 measurement_;     // 2D observed measurement.
    vo_husky::Camera::Ptr camera_;  // Camera model used for projection.
};

#endif
