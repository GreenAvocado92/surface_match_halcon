#include <iostream>
#include <string>
#include <ctime>
#include <string>

#include "../include/surface_match_halcon.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <boost/program_options.hpp>
namespace  bpo = boost::program_options;

int main(int argc, char const *argv[])
{   
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help", "显示帮助信息")
        ("mode", bpo::value<std::string>(), "模式")
        ("model", bpo::value<std::string>(), "输入点云路径")
        ("scene", bpo::value<std::string>(), "输入场景点云")
        ("sfm", bpo::value<std::string>(), "模板信息");

    // 创建一个存储解析后的命令行参数的变量
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 0;
    }

    if (vm.count("mode")) {
        // std::cout << "输入文件: " << vm["mode"].as<std::string>() << "\n";
        if (vm["mode"].as<std::string>() == "train") {
            if (!vm.count("model")) {
                std::cout << "参数不全，请校验" << std::endl;
                return 0;
            }
            GenerateModel(vm["model"].as<std::string>());
        } else if (vm["mode"].as<std::string>() == "det") {
            if (!vm.count("sfm") || !vm.count("scene")) {
                std::cout << "参数不全，请校验" << std::endl;
                return 0;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr scene_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPLYFile(vm["scene"].as<std::string>(), *scene_points);
            // 将 m 转换成 mm
            for(int i = 0; i < scene_points->size(); i++) {
                scene_points->points[i].x = scene_points->points[i].x * 1000;
                scene_points->points[i].y = scene_points->points[i].y * 1000;
                scene_points->points[i].z = scene_points->points[i].z * 1000;
            }
            pcl::io::savePLYFile("./current_scene.ply", *scene_points);

            auto t1 = clock();
            std::vector<Eigen::Matrix4f> res = 
                Matching(vm["sfm"].as<std::string>(), 
                         scene_points, 
                         0.3, 0.1);
            auto t2 = clock();
            std::cout << "cost time : " << t2 - t1 << "/ms" << std::endl;
            std::cout << "result size = " << res.size() << std::endl;

            if (vm.count("model")) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::io::loadPLYFile(vm["model"].as<std::string>(), *cloud);
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_source(new pcl::PointCloud<pcl::PointXYZ>);
                for (int i = 0; i < res.size(); i++) {
                    std::cout << "rt = \n" << res[i] << std::endl;
                    pcl::transformPointCloud(*cloud, *tmp_source, res[i]);      
                    pcl::io::savePLYFile("./res-" + std::to_string(i) + ".ply", *tmp_source);
                }
            }
            std::cout << "======== successful ========" << std::endl;
        }
    } else {
        std::cout << "please input mode type" << std::endl;
    }

    system("pause");
    return 0;
}
