/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "vo_husky/frame.h"

namespace vo_husky {

Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &img_color, const Mat &img_depth)
        : id_(id), time_stamp_(time_stamp), pose_est_(pose), img_color_(img_color), img_depth_(img_depth) {}

SE3 Frame::Pose_EST() {
    std::unique_lock<std::mutex> lck(mutex_pose_);
    return pose_est_;
}

SE3 Frame::Pose_GT() {
    return pose_gt_;
}

void Frame::SetEstimatePose(const SE3 &pose_est) {
    std::unique_lock<std::mutex> lck(mutex_pose_);
    pose_est_ = pose_est;
}

void Frame::SetGroundTruthPose(const SE3 &pose_gt){
    pose_gt_ = pose_gt;
}

void Frame::ShowCurrPose(std::string prefix){
    if(prefix == "GT"){
        LOG(INFO) << "Ground truth pose:\n" 
                << "q: " << pose_gt_.unit_quaternion().x()
                << " " << pose_gt_.unit_quaternion().y() 
                << " " << pose_gt_.unit_quaternion().z() 
                << " " << pose_gt_.unit_quaternion().w()
                << "\nR:\n" << pose_gt_.rotationMatrix() << "\n"    
                << " t: " << pose_gt_.translation().x()
                << " " << pose_gt_.translation().y()
                << " " << pose_gt_.translation().z() << "\n";
        return;
    }

    LOG(INFO) << prefix << " pose:\n" 
            << "q: " << pose_est_.unit_quaternion().x()
            << " " << pose_est_.unit_quaternion().y() 
            << " " << pose_est_.unit_quaternion().z() 
            << " " << pose_est_.unit_quaternion().w()  
            << "\nR:\n" << pose_est_.rotationMatrix() << "\n"    
            << " t: " << pose_est_.translation().x()
            << " " << pose_est_.translation().y()
            << " " << pose_est_.translation().z() << "\n";
    
}

Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++;
    return new_frame;
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
}

}
