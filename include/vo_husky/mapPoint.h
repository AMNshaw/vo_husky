#pragma once
#ifndef VOHUSKY_MAPPOINT_H
#define VOHUSKY_MAPPOINT_H

#include "vo_husky/common_include.h"

namespace vo_husky {

struct Frame;

struct Feature;

/**
 * 路标点类
 * 特征点在三角化之后形成路标点
 */
struct MapPoint {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<MapPoint> Ptr;

    unsigned long id_ = 0;  // ID
    bool is_outlier_ = false;
    Vec3 position_ = Vec3::Zero();  // Position in world
    std::mutex data_mutex_;
    int observed_times_ = 0;  // being observed by feature matching algo.
    std::list<std::weak_ptr<Feature>> observations_;

    MapPoint() {}

    MapPoint(long id, Vec3 position);

    Vec3 Position(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return position_;
    }

    void SetPosition(const Vec3 &pos){
        std::unique_lock<std::mutex> lck(data_mutex_);
        position_ = pos;
    };

    void AddObservation(std::shared_ptr<Feature> feature);
    void RemoveObservation(std::shared_ptr<Feature> feat);

    std::list<std::weak_ptr<Feature>> GetObservations();

    // factory function
    static MapPoint::Ptr CreateNewMappoint();
};
}  // namespace myslam

#endif  // MYSLAM_MAPPOINT_H
