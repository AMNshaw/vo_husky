#include "vo_husky/backend.h"
#include "vo_husky/feature.h"
#include "vo_husky/g2o_types.h"
#include "vo_husky/map.h"
#include "vo_husky/mapPoint.h"
#include "vo_husky/common_include.h"

namespace vo_husky {

Backend::Backend() {
    LOG(INFO) << "Creating backend spin threads...\n";
    BA_running_.store(true);
    PGO_running_.store(true);
    pose_mapPoint_thread_ = std::thread(std::bind(&Backend::Backendloop_BA, this));
    pose_graph_thread_ = std::thread(std::bind(&Backend::Backendloop_PGO, this));

    LOG(INFO) << "Threads initialized\n";
}

void Backend::UpdateMap() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    BA_update_.notify_one();
}

void Backend::UpdatePoseGraph() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    poseGraph_update_.notify_one();
}

void Backend::Stop() {
    BA_running_.store(false);
    PGO_running_.store(false);

    BA_update_.notify_one();
    poseGraph_update_.notify_one();

    pose_mapPoint_thread_.join();
    pose_graph_thread_.join();
}

void Backend::Backendloop_BA() {
    while (BA_running_.load()) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        BA_update_.wait(lock);  // 或使用 predicate 的形式
        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
        LOG(INFO) << "Backend BA optimizing...\n";
        Optimize_BA(active_kfs, active_landmarks);
        LOG(INFO) << "Backend BA optimized\n";
    }
}

void Backend::Backendloop_PGO() {
    while (PGO_running_.load()) {
        Map::KeyframesType all_kfs;
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            poseGraph_update_.wait(lock);

            all_kfs = map_->GetAllKeyFrames();
        }
        // Optimize_PoseGraph(all_kfs);
    }
}

void Backend::Optimize_BA(Map::KeyframesType &keyframes,
                       Map::LandmarksType &landmarks) {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(
        std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(true);

    // pose 顶点，使用Keyframe id
    std::map<unsigned long, VertexPose *> vertices_pose;
    std::map<unsigned long, Vertex3dMapPoint *> vertices_landmarks;

    std::map<unsigned long, std::pair<SE3, SE3>> pose_diff;


    // vertices of poses
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes) {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose_EST());
        optimizer.addVertex(vertex_pose);
        if (kf->keyframe_id_ > max_kf_id) {
            max_kf_id = kf->keyframe_id_;
        }

        vertices_pose.insert({kf->keyframe_id_, vertex_pose});

        // For debug
        pose_diff[kf->keyframe_id_].first = kf->Pose_EST();
    }

    // vertices of landmarks

    for(auto &landmark : landmarks){
        if (landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;

        if (!vertices_landmarks.count(landmark_id)) {
            Vertex3dMapPoint *vertex_mappoint = new Vertex3dMapPoint;
            vertex_mappoint->setEstimate(landmark.second->Position());
            vertex_mappoint->setId(landmark_id + max_kf_id + 1);
            vertex_mappoint->setMarginalized(true);
            vertices_landmarks.insert({landmark_id, vertex_mappoint});
            optimizer.addVertex(vertex_mappoint);
        }
    }


    // edges
    int index = 1;
    double chi2_th = 5.991;  // robust kernel 阈值
    std::map<EdgeReprojection *, Feature::Ptr> edges_and_features;


    for (auto &landmark : landmarks) {
        if (landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;
        auto observations = landmark.second->GetObservations();

        for (auto &obs : observations) {
            auto feat = obs.lock();
            if (feat == nullptr || 
                feat->is_outlier_) continue;

            auto frame = feat->frame_.lock();
            if (feat->frame_.lock() == nullptr) continue;
            
            if (vertices_pose.count(frame->keyframe_id_)&& 
                vertices_landmarks.count(landmark_id)) {
                    
                EdgeReprojection *edge = new EdgeReprojection(camera_);
                edge->setId(index);
                edge->setVertex(0, vertices_pose.at(frame->keyframe_id_));    // pose
                edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                edge->setMeasurement(Vec2(feat->kp_.pt.x, feat->kp_.pt.y));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});
                optimizer.addEdge(edge);
                index++;
            }
        }
    }
    

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    
    for(auto &v_p : vertices_pose){
        pose_diff[v_p.first].second = v_p.second->estimate();
    }

    // for(auto &p : pose_diff){
    //     std::cout << "Pose " << p.first << "\n: " 
    //             << p.second.first.translation()(0) << " " << p.second.second.translation()(0) << "\n"
    //             << p.second.first.translation()(1) << " " << p.second.second.translation()(1) << "\n"
    //             << p.second.first.translation()(2) << " " << p.second.second.translation()(2) << "\n";
    // }
    

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while (iteration < 5) {
        cnt_outlier = 0;
        cnt_inlier = 0;
        // determine if we want to adjust the outlier threshold
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                cnt_outlier++;
            } else {
                cnt_inlier++;
            }
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration++;
        }
    }

    for (auto &ef : edges_and_features) {
        if (ef.first->chi2() > chi2_th) {
            ef.second->is_outlier_ = true;
            auto mp = ef.second->map_point_.lock();
            if (mp) {
                mp->RemoveObservation(ef.second);
            }
        } else {
            ef.second->is_outlier_ = false;
        }
    }

    LOG(INFO) << "Inlier/Total in backend BA optimization: " << cnt_inlier << "/"
              << cnt_inlier + cnt_outlier;

    // Set pose and lanrmark position
    for (auto &v : vertices_pose) {
        keyframes.at(v.first)->SetEstimatePose(v.second->estimate());
    }
    for (auto &v : vertices_landmarks) {
        landmarks.at(v.first)->SetPosition(v.second->estimate());
    }
    optimizer.clear();
}

// void Backend::Optimize_PoseGraph(Map::KeyframesType& keyframes){
//     typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
//     typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
//     auto solver = new g2o::OptimizationAlgorithmLevenberg(
//         std::make_unique<BlockSolverType>(
//         std::make_unique<LinearSolverType>()));
//     g2o::SparseOptimizer optimizer;     // 图模型
//     optimizer.setAlgorithm(solver);   // 设置求解器
//     optimizer.setVerbose(true);       // 打开调试输出

//     int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量

//     std::vector<VertexPose *> vertices_pose;
//     std::vector<EdgePoseGraph *> edges;

//     for(auto it = keyframes.begin(); it != keyframes.end(); ++it){
//         VertexPose* v = new VertexPose();
//         v->setId(it->second->keyframe_id_);
//         v->setEstimate(it->second->Pose_EST());
//         optimizer.addVertex(v);
//         vertexCnt++;
//         vertices_pose.push_back(v);
//         if(it == keyframes.begin())
//             v->setFixed(true);
//     }
//     // 添加邊：利用 std::next 取得下一個元素
//     for(auto it = keyframes.begin(); it != keyframes.end(); ){
//         auto it_next = std::next(it);
//         if(it_next == keyframes.end())
//             break;
        
//         EdgePoseGraph* e = new EdgePoseGraph();
//         SE3 T_ij = it->second->Pose_EST().inverse() * it_next->second->Pose_EST();
//         e->setMeasurement(T_ij);
//         e->setInformation(Mat66::Identity());
//         e->setId(edgeCnt++);
//         e->setVertex(0, optimizer.vertices()[it->second->keyframe_id_]);
//         e->setVertex(1, optimizer.vertices()[it_next->second->keyframe_id_]);
//         optimizer.addEdge(e);
//         edges.push_back(e);
        
//         ++it;
//     }

//     std::cout << "read total " << vertexCnt << " vertices_pose, " << edgeCnt << " edges." << std::endl;

//     std::cout << "optimizing ...\n";
//     optimizer.initializeOptimization();
//     optimizer.optimize(30);

//     for(auto &kf : keyframes){
//         int id = kf.second->keyframe_id_;
//         auto v = optimizer.vertices()[id];
//         if(v)
//             kf.second->SetEstimatePose(
//                 static_cast<VertexPose*>(v)->estimate());
//     }
// }

}  // namespace vo_husky