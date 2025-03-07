#pragma once
#ifndef G2O_TPYES_H
#define G2O_TPYES_H

#include "vo_husky/common_include.h"
#include "vo_husky/camera.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

class VertexPose : public g2o::BaseVertex<6, SE3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override {
        _estimate = SE3();
    }

    virtual void oplusImpl(const double *update) override {
        Vec6 upd;
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = SE3::exp(upd) * _estimate;
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true;}
};

class Vertex3dMapPoint : public g2o::BaseVertex<3, Vec3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override { _estimate = Vec3::Zero(); }

    virtual void oplusImpl(const double *update) override {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true;}
};

class EdgeReprojection : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, Vertex3dMapPoint>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeReprojection(const vo_husky::Camera::Ptr camera) : camera_(camera) {}

    virtual void computeError() override {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const Vertex3dMapPoint *v1 = static_cast<Vertex3dMapPoint *>(_vertices[1]);
        SE3 T_b2w = v0->estimate();
        Vec3 mapPoint = v1->estimate();
        Vec2 pos_pixel = camera_->world2pixel(mapPoint, T_b2w);
        _error = _measurement - pos_pixel;
    }

    virtual void linearizeOplus() override {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const Vertex3dMapPoint *v1 = static_cast<Vertex3dMapPoint *>(_vertices[1]);
        SE3 T_b2w = v0->estimate();
        SE3 T_w2b = T_b2w.inverse();
        Vec3 mapPoint = v1->estimate();
        Vec3 pos_cam = camera_->world2camera(mapPoint, T_b2w);
        double fx = camera_->K()(0, 0);
        double fy = camera_->K()(1, 1);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Zinv = 1.0 / (Z + 1e-18);
        double Zinv2 = Zinv * Zinv;
        _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
            -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            -fy * X * Zinv;

        _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                           camera_->Ext().rotationMatrix() * T_w2b.rotationMatrix();
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true;}


private:
    vo_husky::Camera::Ptr camera_;
};


class EdgePoseGraph : public g2o::BaseBinaryEdge<6, SE3, VertexPose, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void computeError() override {
        SE3 v1 = (static_cast<VertexPose *> (_vertices[0]))->estimate();
        SE3 v2 = (static_cast<VertexPose *> (_vertices[1]))->estimate();
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
    }

    virtual void linearizeOplus() override {
        SE3 v1 = (static_cast<VertexPose *> (_vertices[0]))->estimate();
        SE3 v2 = (static_cast<VertexPose *> (_vertices[1]))->estimate();
        Mat66 J = JRInv(SE3::exp(_error));
        // 尝试把J近似为I？
        _jacobianOplusXi = -J * v2.inverse().Adj();
        _jacobianOplusXj = J * v2.inverse().Adj();
    }

    Mat66 JRInv(const SE3 &e) {
        Mat66 J;
        J.block(0, 0, 3, 3) = SO3::hat(e.so3().log());
        J.block(0, 3, 3, 3) = SO3::hat(e.translation());
        J.block(3, 0, 3, 3) = Mat33::Zero(3, 3);
        J.block(3, 3, 3, 3) = SO3::hat(e.so3().log());
        J = J * 0.5 + Mat66::Identity();
        //J = Mat66::Identity();    // try Identity if you want
        return J;
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true;}
};


#endif