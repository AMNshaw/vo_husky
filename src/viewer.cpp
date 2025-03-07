#include "vo_husky/viewer.h"
#include "vo_husky/feature.h"
#include "vo_husky/frame.h"
#include "vo_husky/common_include.h"

namespace vo_husky {

Viewer::Viewer() {
    viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));

    canvas_ = cv::Mat::zeros(600, 600, CV_8UC3);
}

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframes_ = map_->GetActiveKeyFrames();
    active_landmarks_ = map_->GetActiveMapPoints();
    map_updated_ = true;
}

void Viewer::ThreadLoop() {

    pangolin::CreateWindowAndBind("3D Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
         pangolin::ProjectionMatrix(1024, 768, 420, 420, 512, 389, 0.1, 1000),
         pangolin::ModelViewLookAt(-5, 0, 5, 0, 0, 0, pangolin::AxisX)
    );

    pangolin::View& vis_display = pangolin::CreateDisplay()
         .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
         .SetHandler(new pangolin::Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while (viewer_running_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if (!current_frame_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        Draw3DPose(current_frame_);
        FollowCurrentFrame(vis_camera);

        cv::Mat frame_with_keypoints = PlotFrameImage();
        cv::imshow("Frame with Keypoints", frame_with_keypoints);
        
        PlotPoseComparison(current_frame_->Pose_GT(), current_frame_->Pose_EST());
        cv::imshow("Pose Comparison", canvas_);
        
        cv::waitKey(1);

        if (map_) {
            DrawMapPoints();
        }

        pangolin::FinishFrame();
        usleep(5000);
        // std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
}

cv::Mat Viewer::PlotFrameImage() {
    cv::Mat img_out;
    // 若影像為灰階，轉成 BGR 彩色影像
    if (current_frame_->img_color_.channels() == 1)
        cv::cvtColor(current_frame_->img_color_, img_out, cv::COLOR_GRAY2BGR);
    else
        img_out = current_frame_->img_color_.clone();

    // 遍歷每個特徵點，繪製小圓點
    for (const auto& feat : current_frame_->features_) {
        cv::circle(img_out, feat->kp_.pt, 3, cv::Scalar(0, 255, 0), -1);
    }
    return img_out;
}

void Viewer::PlotPoseComparison(const SE3& groundtruth, const SE3& estimate) {
    // 建立一個黑色畫布
    // 設定縮放因子，將位移值轉換到畫布像素座標（這裡使用 100 像素/單位，可依需求調整）
    double scale = 30.0;
    // 以畫布中心 (300,300) 為原點
    int origin_x = 300, origin_y = 300;

    // 取出位姿的平移向量（這裡假設使用 x 與 z 軸，若需要其他軸向請做相應調整）
    Eigen::Vector3d gt_pos = groundtruth.translation();
    Eigen::Vector3d est_pos = estimate.translation();

    cv::Point gt_point(origin_x + int(gt_pos[0] * scale), origin_y - int(gt_pos[1] * scale));
    cv::Point est_point(origin_x + int(est_pos[0] * scale), origin_y - int(est_pos[1] * scale));

    // 繪製 groundtruth 位姿（綠色）
    cv::circle(canvas_, gt_point, 2, cv::Scalar(0, 255, 0), -1);
    // 繪製估計位姿（紅色）
    if(current_frame_->is_keyframe_)
        cv::circle(canvas_, est_point, 5, cv::Scalar(0, 0, 255), -1);
    else
        cv::circle(canvas_, est_point, 2, cv::Scalar(255, 255, 255), -1);

}

void Viewer::Draw3DPose(Frame::Ptr frame){
    SE3 T_b2w = frame->Pose_EST();

    glPushMatrix();
    Sophus::Matrix4f m = T_b2w.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    // 定義車體尺寸 (以 x 軸表示車長)
    float L = 1.6f;    // 車長
    float W = 0.8f;    // 車寬
    float H = 0.3f;    // 車高

    glLineWidth(2);
    glColor3f(1.0f, 1.0f, 1.0f); // 黑色線框
    glBegin(GL_LINES);
    // 後方矩形 (車尾)
    glVertex3f(-L, -W/2, 0); glVertex3f(-L, W/2, 0);
    glVertex3f(-L, W/2, 0); glVertex3f(-L, W/2, H);
    glVertex3f(-L, W/2, H); glVertex3f(-L, -W/2, H);
    glVertex3f(-L, -W/2, H); glVertex3f(-L, -W/2, 0);

    // 前方矩形 (車頭)
    glVertex3f(0, -W/2, 0); glVertex3f(0, W/2, 0);
    glVertex3f(0, W/2, 0); glVertex3f(0, W/2, H);
    glVertex3f(0, W/2, H); glVertex3f(0, -W/2, H);
    glVertex3f(0, -W/2, H); glVertex3f(0, -W/2, 0);

    // 連接前後對應角點
    glVertex3f(0, -W/2, 0); glVertex3f(-L, -W/2, 0);
    glVertex3f(0, W/2, 0);  glVertex3f(-L, W/2, 0);
    glVertex3f(0, W/2, H);  glVertex3f(-L, W/2, H);
    glVertex3f(0, -W/2, H); glVertex3f(-L, -W/2, H);
    glEnd();

    // 畫一條車頭指向的紅色箭頭 (從車頭中心延伸)
    glLineWidth(3);
    glColor3f(1.0f, 0.0f, 0.0f); // 紅色
    glBegin(GL_LINES);
    glVertex3f(0, 0, H/2);
    glVertex3f(0 + 0.5f, 0, H/2);
    glEnd();

    glPopMatrix();
}

void Viewer::DrawMapPoints() {
    const float color[3] = {0.0, 1.0, 0.0};
    // for (auto& kf : active_keyframes_) {
    //     Draw3DPose(kf.second);
    // }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& landmark : active_landmarks_) {
        auto pos = landmark.second->Position();
        glColor3f(color[0], color[1], color[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    SE3 T_b2w = current_frame_->Pose_EST();

    pangolin::OpenGlMatrix m(T_b2w.matrix());
    vis_camera.Follow(m, true);
}

}  // namespace vo_husky
