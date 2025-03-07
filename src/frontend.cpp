#include "vo_husky/feature.h"
#include "vo_husky/frontend.h"
#include "vo_husky/map.h"
#include "vo_husky/ceres_types.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace vo_husky{

Frontend::Frontend() {
    gftt_ = cv::GFTTDetector::create(num_features_, 0.01, 20);
    num_features_init_ = 100;
    num_features_ = 50;
}

bool Frontend::AddFrame(Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING: {
            LOG(INFO) << "Initializing RGBD camera...\n";
            bool initialized = RGBdInit();
            if(initialized){
                LOG(INFO) << "Frontend initialized\n";
            }
            else{
                LOG(ERROR) << "Fail to initialize frontEnd, try again...\n";
                return false;
            }
            break;
        }
            
        case FrontendStatus::TRACKING: {
            int num_track_lastFrame = TrackLastFrame();
            if (num_track_lastFrame == 0) {
                LOG(ERROR) << "Fail to track last frame \n";
                status_ = FrontendStatus::LOST;
            }
            LOG(INFO) << "Estimating current pose with EPNP...\n"; 
            tracking_inliers_ = EstimatePose();

            if (tracking_inliers_ == 0){
                LOG(ERROR) << "Fail to estimate pose \n";
                return false;
            }
            else{
                // current_frame_->ShowCurrPose("PnP");
            }

            if(tracking_inliers_ < num_features_tracking_){
                LOG(INFO) << "Tracking inliers is not enough, using pose only BA...\n";
                tracking_inliers_ = PoseOnlyBA();
                // current_frame_->ShowCurrPose("Pose Only BA");

                // if(tracking_inliers_ < num_features_tracking_bad_)
                //     status_ = FrontendStatus::LOST;
                
                InsertKeyframe();
                
            }else
                status_ = FrontendStatus::TRACKING;

            // current_frame_->ShowCurrPose("GT");

            break;
        }
            
        case FrontendStatus::LOST: {
            // Reset();
            return false;
        }
            
    }
    if (last_frame_) {
        relative_motion_ = current_frame_->Pose_EST() * last_frame_->Pose_EST().inverse();
    } else {
        relative_motion_ = SE3();
    }
    last_frame_ = current_frame_;
    if (viewer_) viewer_->AddCurrentFrame(current_frame_);

    return true;
}

bool Frontend::RGBdInit(){
    int num_features = DetectFeatures();
    if (num_features < num_features_init_){
        LOG(ERROR) << "Detected features are fewer than the amount needed for initialization";
        return false;
    }
    LOG(INFO) << "Initial features detected";
    bool build_map_success = BuildInitMap();
    if(build_map_success){
        LOG(INFO) << "Initial map was built successfully";
        status_ = FrontendStatus::TRACKING;
        last_frame_ = current_frame_;
        if(viewer_){
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    LOG(ERROR) << "Failed to build initial map";
    return false;
}

int Frontend::DetectFeatures(){
    cv::Mat mask(current_frame_->img_grey_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_) {
        cv::rectangle(mask, feat->kp_.pt - cv::Point2f(10, 10),
                      feat->kp_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->img_grey_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

bool Frontend::BuildInitMap() {
    size_t cnt_init_landmarks = 0;
    for (auto &feat : current_frame_->features_) {

        int x = static_cast<int>(feat->kp_.pt.x);
        int y = static_cast<int>(feat->kp_.pt.y);
        if(x < 0 || x >= current_frame_->img_depth_.cols ||
           y < 0 || y >= current_frame_->img_depth_.rows )
           continue;

        float depth = current_frame_->img_depth_.at<float>(y, x);  
        if(depth <= 0.1f || depth >= 20.0f) continue;
        
        Vec3 pWorld = camera_->pixel2world(Vec2(x, y), depth, SE3());
        
        auto new_map_point = MapPoint::CreateNewMappoint();
        new_map_point->SetPosition(pWorld);
        new_map_point->AddObservation(feat);
        feat->map_point_ = new_map_point;
        cnt_init_landmarks++;
        map_->InsertMapPoint(new_map_point);
    }

    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

int Frontend::TrackLastFrame() {
    if (last_frame_) {
        current_frame_->SetEstimatePose(relative_motion_ * last_frame_->Pose_EST());
    } // offers a prediction of the pixel of map points in this frame
    else {
        LOG(INFO) << "No last frame to track\n";
        return 0;
    }
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &ft : last_frame_->features_) {
        auto mp = ft->map_point_.lock();
        if (mp) {
            auto pixel_curr = camera_->world2pixel(mp->Position(), current_frame_->Pose_EST());
            kps_last.push_back(ft->kp_.pt);
            kps_current.push_back(cv::Point2f(pixel_curr[0], pixel_curr[1]));
        } else {
            kps_last.push_back(ft->kp_.pt);
            kps_current.push_back(ft->kp_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->img_grey_,
        current_frame_->img_grey_,
        kps_last,
        kps_current,
        status, 
        error, 
        cv::Size(11, 11), 
        3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW); //

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            Feature::Ptr new_feat = std::make_shared<Feature>(
                current_frame_, 
                cv::KeyPoint(kps_current[i], 7)
            );
            auto mp = last_frame_->features_[i]->map_point_.lock();
            if(!mp) continue;
            new_feat->map_point_ = mp;
            current_frame_->features_.push_back(new_feat);
                                
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " tracked points in the last image.";

    return num_good_pts;
}

int Frontend::EstimatePose(){
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    std::vector<Feature::Ptr> availible_features;
    for(auto &ft : current_frame_->features_){
        auto mp = ft->map_point_.lock();
        if (mp == nullptr) continue;

        pts_3d.push_back(cv::Point3f( 
            mp->Position().x(), mp->Position().y(),mp->Position().z())
        );
        pts_2d.push_back(ft->kp_.pt);
        availible_features.push_back(ft);
    }
    Sophus::SE3d T_w2c = current_frame_->Pose_EST().inverse() * camera_->Ext();
    Mat33 R_eigen = T_w2c.rotationMatrix();
    Vec3 t_eigen = T_w2c.translation();


    cv::Mat R_cv; cv::eigen2cv(R_eigen, R_cv);
    cv::Mat rvec; cv::Rodrigues(R_cv, rvec); 
    cv::Mat tvec; cv::eigen2cv(t_eigen, tvec);
    cv::Mat K; cv::eigen2cv(camera_->K(), K);

    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat inliers;

    bool success = cv::solvePnPRansac(
        pts_3d, 
        pts_2d,
        K,   // 假設 camera_.K() 回傳 3x3 (CV_64F)
        distCoeffs,
        rvec, 
        tvec,
        true,          // use initial guess
        100,           // RANSAC iterations
        8.0,           // reprojection error threshold
        0.99,          // confidence
        inliers,
        cv::SOLVEPNP_EPNP
    );

    if (!success) {
        LOG(ERROR) << "PnP failed!" << std::endl;
        return 0;
    }
    LOG(INFO) << "PnP Inliers: " << inliers.total() << "/" << pts_3d.size() << std::endl;

    inlier_features_pnp_.clear();

    for (int i = 0; i < inliers.rows; i++) {
        int idx = inliers.at<int>(i, 0);  // 第 i 行的索引
        inlier_features_pnp_.push_back(availible_features[idx]);
    }
    
    cv::Rodrigues(rvec, R_cv);

    cv::cv2eigen(R_cv, R_eigen);
    cv::cv2eigen(tvec, t_eigen);

    T_w2c = SE3(R_eigen, t_eigen);

    SE3 T_b2w = (camera_->Ext().inverse() * T_w2c).inverse();

    current_frame_->SetEstimatePose(T_b2w);

    return inliers.total();
}

int Frontend::PoseOnlyBA(){

    Vec6 xi = current_frame_->Pose_EST().log();
     
    double pose[6];
    pose[0] = xi(0);
    pose[1] = xi(1);
    pose[2] = xi(2);
    pose[3] = xi(3);
    pose[4] = xi(4);
    pose[5] = xi(5);

    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    int prev_cnt_outlier = 0;

    LOG(INFO) << "Pose only BA optimizing...\n";
    for (int iteration = 0; iteration < 4; ++iteration) {

        ceres::Problem problem;

        problem.AddParameterBlock(pose, 6);
        problem.SetManifold(pose, new SophusSE3Manifold());


        for (auto &feat : inlier_features_pnp_) {
            auto mp = feat->map_point_.lock();
            if(!mp) continue;
            Vec3 Pw = mp->Position();
            Vec2 uv(feat->kp_.pt.x, feat->kp_.pt.y);

            std::unique_ptr<ceres::CostFunction> cost_func = 
                            std::make_unique<ReprojectionErrCeres>(
                                Pw, uv, camera_);

            std::unique_ptr<ceres::LossFunction> loss;
            if (iteration != 2) {
                loss = std::make_unique<ceres::HuberLoss>(1.0);
            }

            problem.AddResidualBlock(cost_func.release(), loss.release(), pose);            
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.max_num_iterations = 100;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // std::cout << summary.BriefReport() << std::endl;

        cnt_outlier = 0;
        // count outliers

        for (auto &feat : inlier_features_pnp_){
            auto mp = feat->map_point_.lock();
            if(mp == nullptr) continue;
            Vec3 Pw = mp->Position();
            Vec2 uv(feat->kp_.pt.x, feat->kp_.pt.y);
            
            double err_square = ReprojectionErrCeres::ComputeError(
                pose, 
                Pw, 
                uv,
                camera_);
            if(err_square >= chi2_th){
                feat->is_outlier_ = true;
                cnt_outlier++;
            } else {
                feat->is_outlier_ = false;
            };
        }

        if (cnt_outlier == prev_cnt_outlier) break;

        if (cnt_outlier == 0) break;
        
        prev_cnt_outlier = cnt_outlier;

    }

    LOG(INFO) << "Outlier/Inlier in pose only BA estimation: " << cnt_outlier << "/"
              << inlier_features_pnp_.size() - cnt_outlier;
    
    xi << pose[0], pose[1], pose[2], pose[3], pose[4], pose[5];
    SE3 T_b2w_optimized = SE3::exp(xi);

    current_frame_->SetEstimatePose(T_b2w_optimized);

    for (auto &feat : inlier_features_pnp_) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }

    return inlier_features_pnp_.size() - cnt_outlier;;
}

bool Frontend::InsertKeyframe() {
    static int cnt_keyframe = 0;
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // still have enough features, don't insert keyframe`
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();
    DetectFeatures();  // detect new features

    // Calculate map points
    CalculateNewMapPoints();
    // update backend because we have a new keyframe
    backend_->UpdateMap();
    cnt_keyframe ++;

    if(cnt_keyframe == 7){
        // backend_->UpdatePoseGraph();
        cnt_keyframe = 0;
    }

    if (viewer_) viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->features_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}

int Frontend::CalculateNewMapPoints(){
    int cnt_mapPoints = 0;
    for (auto &feat : current_frame_->features_) {
        
        int x = static_cast<int>(feat->kp_.pt.x);
        int y = static_cast<int>(feat->kp_.pt.y);
        if(x < 0 || x >= current_frame_->img_depth_.cols ||
           y < 0 || y >= current_frame_->img_depth_.rows )
           continue;
        float depth = current_frame_->img_depth_.at<float>(y, x);  
        if(depth <= 0.01f || depth >= 20.0f) continue;
        
        Vec3 pWorld = camera_->pixel2world(Vec2(x, y), depth, current_frame_->Pose_EST());

        auto new_map_point = MapPoint::CreateNewMappoint();
        new_map_point->SetPosition(pWorld);
        new_map_point->AddObservation(feat);
        feat->map_point_ = new_map_point;
        map_->InsertMapPoint(new_map_point);

        cnt_mapPoints++;
    }
    return cnt_mapPoints;
}

}