#include "vo_husky/mapPoint.h"
#include "vo_husky/feature.h"

namespace vo_husky {

MapPoint::MapPoint(long id, Vec3 position) : id_(id), position_(position) {}

/** 
* Add, Get, and Remove observations can be done concurrently, 
* so a mutex lock is used to ensure thread-safe access and prevent 
* data races when modifying the observation list.
*/

void MapPoint::AddObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    observations_.push_back(feature);
    observed_times_++;
}

std::list<std::weak_ptr<Feature>> MapPoint::GetObservations() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return observations_;
}

/**
 * Remove a feature observation from this map point.
 * If the feature is found in the list of observations, it is removed.
 * Also resets the feature's reference to this map point to avoid dangling references.
 * 
 * @param feat The feature to be removed.
 */
void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    for (auto iter = observations_.begin(); iter != observations_.end();iter++){
        if (iter->lock() == feat) {
            observations_.erase(iter);
            feat->map_point_.reset();
            observed_times_--;
            break;
        }
    }
}

/**
 * Create a new MapPoint.
 * Each MapPoint is assigned a unique ID. The backend optimization 
 * process will later refine its position based on observations.
 *
 * @return A shared pointer to the newly created MapPoint.
 */
MapPoint::Ptr MapPoint::CreateNewMappoint() {
    static long factory_id = 0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}


}  // namespace vo_husky
