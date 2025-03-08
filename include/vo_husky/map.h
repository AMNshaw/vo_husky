#pragma once
#ifndef VOHUSKY_MAP_H
#define VOHUSKY_MAP_H

#include "vo_husky/common_include.h"
#include "vo_husky/frame.h"
#include "vo_husky/mapPoint.h"

namespace vo_husky {

/**
 * Map
 * 
 * Manages all keyframes and map points in the SLAM system. This class maintains both global
 * and active sets of keyframes and map points, and uses a mutex to ensure thread-safe access.
 */
class Map {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Map> Ptr;
    
    // Because we want to randomly access the landmarks and keyframes, we store them in a hashmap.
    // The Map actually owns the keyframes and the landmarks, so we store them using shared pointers.
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType; 
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map() {}

    // Insert a keyframe into the map.
    void InsertKeyFrame(Frame::Ptr frame);
    
    // Insert a map point (landmark) into the map.
    void InsertMapPoint(MapPoint::Ptr map_point);

    // Retrieve a copy of all map points.
    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    
    // Retrieve a copy of all keyframes.
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    // Retrieve a copy of active map points.
    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    // Retrieve a copy of active keyframes.
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    // Remove map points that have zero observations.
    void CleanMap();

   private:
    // Mark old keyframes as inactive if the number of active keyframes exceeds a threshold.
    void RemoveOldKeyframe();

    std::mutex data_mutex_;           // Protects access to the map data

    LandmarksType landmarks_;         // All map points (landmarks)
    LandmarksType active_landmarks_;  // Active map points
    KeyframesType keyframes_;         // All keyframes
    KeyframesType active_keyframes_;  // Active keyframes

    Frame::Ptr current_frame_ = nullptr;  // Current frame reference

    int num_active_keyframes_ = 7;    // Maximum number of active keyframes
};

}  // namespace vo_husky

#endif  // VOHUSKY_MAP_H
