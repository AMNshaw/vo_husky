#include "vo_husky/visual_odometry.h"
#include <chrono>

namespace vo_husky {

VisualOdometry::VisualOdometry(Camera::Ptr camera) : camera_(camera){}

bool VisualOdometry::Init() {

    // create components and links
    
    frontend_ = std::make_shared<Frontend>();
    backend_ = std::make_shared<Backend>();
    map_ = std::make_shared<Map>();
    viewer_ = std::make_shared<Viewer>();

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCamera(camera_);

    backend_->SetMap(map_);
    backend_->SetCamera(camera_);

    viewer_->SetMap(map_);
    viewer_->SetCamera(camera_);

    return true;
}

bool VisualOdometry::step(Frame::Ptr newFrame){
    // Next frame
    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(newFrame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}


void VisualOdometry::End() {
    // Close the threads of backend and viwer
    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

}  // namespace vo_husky
