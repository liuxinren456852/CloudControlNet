#include "dataio.hpp"
#include "filter.hpp"
#include "map_viewer.hpp"
#include "common_reg.hpp"
#include "find_constraint.h"
#include "utility.h"
#include <glog/logging.h>

using namespace std;
using namespace ccn;

typedef pcl::PointNormal Point_T;

int main(int argc, char **argv)
{
    google::InitGoogleLogging("Mylog");
    google::SetLogDestination(google::GLOG_INFO, "./log/test/MyLogInfo");
    LOG(INFO) << "Launch the program!";

    //Import configuration
    //Data path (4 formats are available: *.pcd, *.las, *.ply, *.txt)
    std::string filenameT = argv[1]; //Target pointcloud file path
    std::string filenameS = argv[2]; //Source pointcloud file path
    std::string filenameR = argv[3]; //Registered source pointcloud file path

    float vf_downsample_resolution_source = atof(argv[4]); //0.2
    float vf_downsample_resolution_target = atof(argv[5]); //0.2
    float gf_grid_resolution = atof(argv[6]);              //2.0
    float gf_max_grid_height_diff = atof(argv[7]);         //0.3
    float gf_neighbor_height_diff = atof(argv[8]);         //2.5
    float neighbor_search_radius = atof(argv[9]);          //1.0

    //Import the data
    DataIo<Point_T> dataio;
    pcl::PointCloud<Point_T>::Ptr pointCloudT(new pcl::PointCloud<Point_T>()), pointCloudS(new pcl::PointCloud<Point_T>());
    dataio.readCloudFile(filenameT, pointCloudT);
    dataio.readCloudFile(filenameS, pointCloudS);

    //Downsampling
    CFilter<Point_T> cfilter;
    pcl::PointCloud<Point_T>::Ptr pointCloudT_down(new pcl::PointCloud<Point_T>()), pointCloudS_down(new pcl::PointCloud<Point_T>());
    cfilter.voxelfilter(pointCloudT, pointCloudT_down, vf_downsample_resolution_target);
    cfilter.voxelfilter(pointCloudS, pointCloudS_down, vf_downsample_resolution_source);

    pcl::PointCloud<Point_T>::Ptr pointCloudT_down_ground(new pcl::PointCloud<Point_T>()), pointCloudT_ground(new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr pointCloudT_down_edge(new pcl::PointCloud<Point_T>()), pointCloudT_edge(new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr pointCloudT_down_facade(new pcl::PointCloud<Point_T>()), pointCloudT_facade(new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr pointCloudT_down_cluster(new pcl::PointCloud<Point_T>()), pointCloudT_cluster(new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr pointCloudS_down_ground(new pcl::PointCloud<Point_T>()), pointCloudS_ground(new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr pointCloudS_down_edge(new pcl::PointCloud<Point_T>()), pointCloudS_edge(new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr pointCloudS_down_facade(new pcl::PointCloud<Point_T>()), pointCloudS_facade(new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr pointCloudS_down_cluster(new pcl::PointCloud<Point_T>()), pointCloudS_cluster(new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr pointCloudT_unground(new pcl::PointCloud<Point_T>()), pointCloudS_unground(new pcl::PointCloud<Point_T>());

    int gf_min_grid_num = 20;
    float gf_max_ground_height = 500.0;
    int gf_downsample_rate_nonground = 4;
    int gf_downsample_rate_ground_first = 10;  //For reference ground points (more point number)
    int gf_downsample_rate_ground_second = 40; //For search ground points (less point number)

    int feature_min_neighbor_point_num = 15;
    float edge_point_thre_low = 0.65;     //For reference edge points (more point number)
    float planar_point_thre_low = 0.65;   //For reference planar points (more point number)
    float sphere_point_thre_low = 0.65;   //For reference sphere points (more point number)
    float edge_point_thre_high = 0.75;    //For search edge points (less point number)
    float planar_point_thre_high = 0.75;  //For search planar points (less point number)
    float sphere_point_thre_high = 0.75;  //For search sphere points (less point number)

    float dis_thre_ground = 1.0;
    float dis_thre_edge = 1.0;
    float dis_thre_facade = 1.0;
    float dis_thre_cluster = 1.0;

    cfilter.FastGroundFilter(pointCloudT_down, pointCloudT_ground, pointCloudT_down_ground, pointCloudT_unground, gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff, gf_neighbor_height_diff,
                             gf_max_ground_height, gf_downsample_rate_ground_first, gf_downsample_rate_ground_second, gf_downsample_rate_nonground);

    cfilter.FastGroundFilter(pointCloudS_down, pointCloudS_ground, pointCloudS_down_ground, pointCloudS_unground, gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff, gf_neighbor_height_diff,
                             gf_max_ground_height, gf_downsample_rate_ground_first, gf_downsample_rate_ground_second, gf_downsample_rate_nonground);

    cfilter.RoughClassify(pointCloudT_unground, pointCloudT_edge, pointCloudT_facade, pointCloudT_cluster, pointCloudT_down_edge, pointCloudT_down_facade, pointCloudT_down_cluster,
                          neighbor_search_radius, feature_min_neighbor_point_num, edge_point_thre_low, planar_point_thre_low, sphere_point_thre_low, edge_point_thre_high, planar_point_thre_high, sphere_point_thre_high);

    cfilter.RoughClassify(pointCloudS_unground, pointCloudS_edge, pointCloudS_facade, pointCloudS_cluster, pointCloudS_down_edge, pointCloudS_down_facade, pointCloudS_down_cluster,
                          neighbor_search_radius, feature_min_neighbor_point_num, edge_point_thre_low, planar_point_thre_low, sphere_point_thre_low, edge_point_thre_high, planar_point_thre_high, sphere_point_thre_high);

    Eigen::Matrix4f transformationS2T;
    Eigen::Matrix<float, 6, 6> information_matrix;
    CRegistration<pcl::PointNormal> creg;

    creg.feature_pts_registration(pointCloudS_down_ground, pointCloudS_down_edge, pointCloudS_down_facade, pointCloudS_down_cluster,
                                  pointCloudT_ground, pointCloudT_edge, pointCloudT_facade, pointCloudT_cluster,
                                  transformationS2T, information_matrix,
                                  20, dis_thre_ground, dis_thre_edge, dis_thre_facade, dis_thre_cluster);

    pcl::PointCloud<Point_T>::Ptr pointCloudS_reg(new pcl::PointCloud<Point_T>());

    pcl::transformPointCloud(*pointCloudS, *pointCloudS_reg, transformationS2T);
    dataio.writeCloudFile(filenameR, pointCloudS_reg); //Write out the transformed Source Point Cloud

    MapViewer<Point_T> viewer;
    viewer.Dispaly2Cloud(pointCloudS_reg, pointCloudT, "Registration Result", 5); //Show the registration result

    return 1;
}