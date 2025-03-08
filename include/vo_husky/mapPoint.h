#pragma once
#ifndef VOHUSKY_MAPPOINT_H
#define VOHUSKY_MAPPOINT_H

#include "vo_husky/common_include.h"

namespace vo_husky {

struct Frame;
struct Feature;

/**
 * MapPoint 
 * 
 * Each MapPoint represents a 3D point in the world coordinate system and can be observed by
 * multiple Features from different Frames.
 * Since MapPoints can be computed in the frontend and optimized in the backend, a mutex is used
 * to ensure thread-safe data access.
*/
struct MapPoint {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<MapPoint> Ptr;

    Vec3 position_ = Vec3::Zero();  // Position of the MapPoint in the world coordinate system
    std::list<std::weak_ptr<Feature>> observations_; // Features that observe this MapPoint (using weak_ptr to avoid circular references)

    unsigned long id_ = 0;          // Unique identifier for the MapPoint
    int observed_times_ = 0;        // Number of times this MapPoint has been tracked by different Frames
    bool is_outlier_ = false;       // Flag indicating whether this MapPoint is considered an outlier

    std::mutex data_mutex_;         // Mutex to protect MapPoint data during concurrent access

   public:
    MapPoint() {}

    MapPoint(long id, Vec3 position);

    // Retrieves the position of the MapPoint in a thread-safe manner
    Vec3 Position(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return position_;
    }

    // Sets the position of the MapPoint in a thread-safe manner
    void SetPosition(const Vec3 &pos){
        std::unique_lock<std::mutex> lck(data_mutex_);
        position_ = pos;
    };

    // Adds a Feature observation to this MapPoint
    // Typically called after feature matching
    void AddObservation(std::shared_ptr<Feature> feature);

    // Removes a Feature observation from this MapPoint
    void RemoveObservation(std::shared_ptr<Feature> feat);

    // Retrieves all Features that observe this MapPoint
    std::list<std::weak_ptr<Feature>> GetObservations();

    // Factory function: Creates a new MapPoint instance
    static MapPoint::Ptr CreateNewMappoint();
};

}  // namespace vo_husky

#endif  // VOHUSKY_MAPPOINT_H
