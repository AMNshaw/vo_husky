#pragma once
#ifndef VOHUSKY_BACKEND_H
#define VOHUSKY_BACKEND_H

#include "vo_husky/common_include.h"
#include "vo_husky/frame.h"
#include "vo_husky/map.h"

namespace vo_husky {

/**
 * Backend
 * 
 * Processes optimization in a separate thread, triggered when the map is updated.
 * The frontend signals a map update, which starts the optimization routines.
 */
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Backend> Ptr;

    // Start the backend thread loop in the constructor.
    Backend();

    // Set the camera with its intrinsic and extrinsic parameters.
    void SetCamera(Camera::Ptr cam) { camera_ = cam; }

    // Set the map.
    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    // Trigger a map update to start optimization.
    void UpdateMap();

    // Trigger an update of the pose graph.
    void UpdatePoseGraph();

    // Stop the backend thread.
    void Stop();

   private:
    // Backend thread loop for bundle adjustment (BA).
    void Backendloop_BA();

    // Backend thread loop for pose graph optimization (PGO).
    void Backendloop_PGO();

    // Optimize active keyframes and landmarks using bundle adjustment.
    void Optimize_BA(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    // Optimize the pose graph using active keyframes.
    void Optimize_PoseGraph(Map::KeyframesType& keyframes);

    std::shared_ptr<Map> map_;                // Map containing keyframes and landmarks
    std::thread pose_mapPoint_thread_;        // Thread for BA optimization
    std::thread pose_graph_thread_;           // Thread for pose graph optimization
    std::mutex data_mutex_;                   // Mutex to protect shared data

    std::condition_variable BA_update_;       // Condition variable for BA updates
    std::condition_variable poseGraph_update_;  // Condition variable for pose graph updates
    std::atomic<bool> BA_running_;            // Flag indicating BA is running
    std::atomic<bool> PGO_running_;           // Flag indicating PGO is running

    Camera::Ptr camera_ = nullptr;            // Camera used for optimization
};

}  // namespace vo_husky

#endif  // VOHUSKY_BACKEND_H
