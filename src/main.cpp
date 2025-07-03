#include <iostream>
#include <string>
#include <ctime>
#include <string>
#include <fstream>

#include "../include/surface_match_halcon.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <boost/program_options.hpp>
namespace  bpo = boost::program_options;

bool save_to_asc(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::ofstream out(filename);
    for (const auto& it : cloud->points) {
        out << it.x << " " << it.y << " " << it.z << std::endl;
    }
    out.close();
}

int main(int argc, char const *argv[])
{   
    std::locale::global(std::locale(""));

    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help", "show help information")
        ("mode", bpo::value<std::string>(), "mode, train or det")
        ("model", bpo::value<std::string>(), "input model file path")
        ("scene", bpo::value<std::string>(), "input scene file path")
        ("sfm", bpo::value<std::string>(), "set SFM file path")
        ("sd", bpo::value<float>(), "sample distance");

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
                std::cout << "please input model file path, this file should be a clear pointcloud file" << std::endl;
                return 0;
            }
            GenerateModel(vm["model"].as<std::string>(), "./model", vm["sd"].as<float>());

        } else if (vm["mode"].as<std::string>() == "det") {
            if (!vm.count("sfm") || !vm.count("scene")) {
                std::cout << "Missing required file: sfm or scene path" << std::endl;
                return 0;
            }
            // pcl::PointCloud<pcl::PointXYZ>::Ptr scene_points(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::io::loadPLYFile(vm["scene"].as<std::string>(), *scene_points);
            
            auto t1 = clock();
            std::vector<Eigen::Matrix4f> res = 
                Matching(vm["sfm"].as<std::string>(), 
                         vm["scene"].as<std::string>(), 
                         vm["sd"].as<float>(), 0.3);

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
                    save_to_asc("./res-" + std::to_string(i) + ".asc", tmp_source);
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
