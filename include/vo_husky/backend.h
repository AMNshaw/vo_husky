//
// Created by gaoxiang on 19-5-2.
//

#ifndef VOHUSKY_BACKEND_H
#define VOHUSKY_BACKEND_H

#include "vo_husky/common_include.h"
#include "vo_husky/frame.h"
#include "vo_husky/map.h"

namespace vo_husky {
class Map;

/**
 * 后端
 * 有单独优化线程，在Map更新时启动优化
 * Map更新由前端触发
 */ 
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Backend> Ptr;

    /// 构造函数中启动优化线程并挂起
    Backend();

    // 设置左右目的相机，用于获得内外参
    void SetCamera(Camera::Ptr cam) { camera_ = cam;}

    /// 设置地图
    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    /// 触发地图更新，启动优化
    void UpdateMap();

    void UpdatePoseGraph();

    /// 关闭后端线程
    void Stop();

   private:
    /// 后端线程
    void Backendloop_BA();
    void Backendloop_PGO();

    /// 对给定关键帧和路标点进行优化
    void Optimize_BA(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);
    void Optimize_PoseGraph(Map::KeyframesType& keyframes);

    std::shared_ptr<Map> map_;
    std::thread pose_mapPoint_thread_;
    std::thread pose_graph_thread_;
    std::mutex data_mutex_;

    std::condition_variable BA_update_;
    std::condition_variable poseGraph_update;
    std::atomic<bool> BA_running_;
    std::atomic<bool> PGO_running_;

    Camera::Ptr camera_ = nullptr;
};

}  // namespace vo_husky

#endif  // MYSLAM_BACKEND_H