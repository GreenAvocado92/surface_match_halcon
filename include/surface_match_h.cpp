#include "./surface_match_h.h"
#include <halconcpp/HalconCpp.h>
#include <halconcpp/HDevThread.h>
#include <Eigen/Dense>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
using namespace HalconCpp;

bool GenerateModel(const std::string& modelfile, const std::string& outfile, const float& sampledistance) {
    std::cout << "======== create surface model begin ========" << std::endl;
    HTuple hv_ObjectModel3DModel, hv_SFM, hv_Status;
    ReadObjectModel3d(modelfile.c_str(), "m", HTuple(), HTuple(), &hv_ObjectModel3DModel, &hv_Status);
    SurfaceNormalsObjectModel3d(hv_ObjectModel3DModel, "mls", HTuple(), HTuple(), &hv_ObjectModel3DModel);
    CreateSurfaceModel(hv_ObjectModel3DModel, sampledistance, HTuple(), HTuple(), &hv_SFM);
    std::string outfile_sfm = outfile + ".sfm";
    std::string outfile_model = outfile + ".ply";
    WriteSurfaceModel (hv_SFM, "./model.sfm");
    WriteObjectModel3d(hv_ObjectModel3DModel, "ply", "./model.ply", HTuple(), HTuple());
    std::cout << "======== create surface model success ========" << std::endl;
    return true;
}

std::vector<Eigen::Matrix4f> 
Matching(const HTuple& hv_SFM, 
         const HTuple& hv_ObjectModel3DScene,
         const float& sampledistance = 0.3,
         const float& min_score = 0.3) {
    // Local control variables
    HTuple  hv_ObjectModel3DModel, hv_Status;
    HTuple  hv_Pose, hv_Score, hv_ObjectModel3DSceneReduced;
    HTuple  hv_SurfaceMatchingResultID, hv_ObjectModel3DResult;
    HTuple  hv_SampledScene, hv_ResultValue, hv_TransformedModel3D;
    HTuple  hv_KeyPoints, hv_HomMat3DIn;
    HomMat3dIdentity(&hv_HomMat3DIn);

    SurfaceNormalsObjectModel3d(hv_ObjectModel3DScene, "mls", HTuple(), HTuple(), &hv_ObjectModel3DSceneReduced);
    FindSurfaceModel(hv_SFM, hv_ObjectModel3DSceneReduced, 0.05, sampledistance, min_score, "true", 
        "num_matches", 10, &hv_Pose, &hv_Score, &hv_SurfaceMatchingResultID);
    for (int i = 0; i < hv_Score.Length(); i++)
        std::cout << "hv_Score = " << hv_Score[i].D() << std::endl;
    std::vector<Eigen::Matrix4f> vec;
    if (hv_Pose.Length() != 0) {
        for (int num = 0; num < hv_Pose.Length() / 7; num++) {
            GetSurfaceMatchingResult(hv_SurfaceMatchingResultID, "pose", num, &hv_ResultValue);
            // RigidTransObjectModel3d(hv_ObjectModel3DModel, hv_ResultValue, &hv_TransformedModel3D);
            // WriteObjectModel3d(hv_TransformedModel3D, "ply", "./TransformedModel3D.ply", HTuple(), HTuple());
            PoseToHomMat3d(hv_ResultValue, &hv_HomMat3DIn);
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            
            pose(0, 0) = hv_HomMat3DIn[0].D();
            pose(0, 1) = hv_HomMat3DIn[1].D();
            pose(0, 2) = hv_HomMat3DIn[2].D();
            pose(0, 3) = hv_HomMat3DIn[3].D();
            pose(1, 0) = hv_HomMat3DIn[4].D();
            pose(1, 1) = hv_HomMat3DIn[5].D();
            pose(1, 2) = hv_HomMat3DIn[6].D();
            pose(1, 3) = hv_HomMat3DIn[7].D();
            pose(2, 0) = hv_HomMat3DIn[8].D();
            pose(2, 1) = hv_HomMat3DIn[9].D();
            pose(2, 2) = hv_HomMat3DIn[10].D();
            pose(2, 3) = hv_HomMat3DIn[11].D();
            vec.push_back(pose);
        }
    }
    return vec;
}

std::vector<Eigen::Matrix4f> 
Matching(const std::string& model_name_sfm, 
         pcl::PointCloud<pcl::PointXYZ>::Ptr scene,
         const float& sampledistance,
         const float& min_score) {
    
    auto t1 = clock();
    // Local control variables
    HTuple  hv_ObjectModel3DSceneReduced, hv_SFM;

    HTuple tx, ty, tz;
    for (int i = 0; i < scene->points.size(); i++) {
        tx.Append(scene->points[i].x);
        ty.Append(scene->points[i].y);
        tz.Append(scene->points[i].z);
    }
    GenObjectModel3dFromPoints(tx, ty, tz, &hv_ObjectModel3DSceneReduced);
    auto t2 = clock();
    std::cout << "trans point cloud cost time: " << t2 - t1 << std::endl;

    t1 = clock();
    ReadSurfaceModel(model_name_sfm.c_str(), &hv_SFM);
    t2 = clock();
    std::cout << "read sfm file cost time: " << t2 - t1 << std::endl;

    t1 = clock();
    auto res = Matching(hv_SFM, hv_ObjectModel3DSceneReduced, sampledistance, min_score);
    t2 = clock();
    std::cout << "Match core cost time: " << t2 - t1 << std::endl;

    return res;
}

std::vector<Eigen::Matrix4f> Matching(const std::string& model_name_sfm, 
                             const std::string& scene_name,
                             const float& sampledistance,
                             const float& min_score) {
    // Local control variables
    HTuple  hv_ObjectModel3DSceneReduced, hv_SFM, hv_Status;

    ReadSurfaceModel(model_name_sfm.c_str(), &hv_SFM);

    ReadObjectModel3d(scene_name.c_str(), "m", HTuple(), HTuple(), &hv_ObjectModel3DSceneReduced, &hv_Status);

    return Matching(hv_SFM, hv_ObjectModel3DSceneReduced, sampledistance, min_score);
}
