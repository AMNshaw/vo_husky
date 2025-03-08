/*
 * Map Implementation
 * 
 * Manages keyframes and landmarks in the SLAM system. Handles inserting keyframes 
 * and map points, removing old keyframes, and cleaning up unused landmarks.
 */

#include "vo_husky/map.h"
#include "vo_husky/feature.h"

namespace vo_husky {

/**
 * Insert a keyframe into the map.
 * If the keyframe already exists, it is reinserted.
 * If the number of active keyframes exceeds the limit, the oldest keyframe is removed.
 *
 * @param frame The keyframe to insert.
 */
void Map::InsertKeyFrame(Frame::Ptr frame) {
    current_frame_ = frame;
    if (keyframes_.count(frame->keyframe_id_)) {
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    } else {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if (active_keyframes_.size() > num_active_keyframes_) {
        RemoveOldKeyframe();
    }
}

/**
 * Insert a map point into the map.
 * If the map point already exists, it is reinserted.
 *
 * @param map_point The map point to insert.
 */
void Map::InsertMapPoint(MapPoint::Ptr map_point) {
    if (landmarks_.count(map_point->id_)) {
        landmarks_.insert(make_pair(map_point->id_, map_point));
        active_landmarks_.insert(make_pair(map_point->id_, map_point));
    } else {
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }
}

/**
 * Remove an old keyframe if the number of active keyframes exceeds the limit.
 * The function selects the keyframe to remove based on its distance to the current frame:
 * - If a keyframe is too close to the current frame (below a threshold), it is removed.
 * - Otherwise, the farthest keyframe is removed.
 */
void Map::RemoveOldKeyframe() {
    if (current_frame_ == nullptr) return;

    // Find the closest and farthest keyframes
    double max_dis = 0, min_dis = INT16_MAX;
    unsigned long max_kf_id = 0, min_kf_id = 0;
    auto T_w2b_current = current_frame_->Pose_EST().inverse();
    for (auto& kf : active_keyframes_) {
        if (kf.second == current_frame_) continue;  // Skip the current frame
        auto T_b2w_past = kf.second->Pose_EST();
        auto dis = (T_b2w_past * T_w2b_current).log().norm();
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = 0.2;  // Threshold for minimum distance
    Frame::Ptr frame_to_remove = nullptr;

    // Remove the keyframe that is too close, otherwise remove the farthest one
    if (min_dis < min_dis_th) {
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "Removing keyframe " << frame_to_remove->keyframe_id_;

    // Remove keyframe and its associated landmark observations
    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    for (auto feat : frame_to_remove->features_) {
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }

    CleanMap();
}

/**
 * Clean up inactive map points.
 * If a map point is not observed by any active keyframe, it is removed.
 */
void Map::CleanMap() {
    int cnt_landmark_removed = 0;
    for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();) {
        if (iter->second->observed_times_ == 0) {
            iter = active_landmarks_.erase(iter);
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }
    LOG(INFO) << "Removed " << cnt_landmark_removed << " inactive landmarks";
}

}  // namespace vo_husky
