#include <iostream>
#include <ios>
#include <vector>
#include <Eigen/Dense>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

bool GenerateModel(const std::string& modelfile, const std::string& outfile = "./model", const float& sampledistance = 0.03);

std::vector<Eigen::Matrix4f> 
Matching(const std::string& model_name_sfm, 
         pcl::PointCloud<pcl::PointXYZ>::Ptr scene,
         const float& sampledistance = 0.3,
         const float& min_score = 0.3);
         
std::vector<Eigen::Matrix4f> Matching(const std::string& model_name_sfm, 
         const std::string& scene_name = "G:/halcon_data/scene03.ply",
         const float& sampledistance = 0.3,
         const float& min_score = 0.3)