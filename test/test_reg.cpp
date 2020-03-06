#include "dataio.hpp"
#include "filter.hpp"
#include "map_viewer.hpp"
#include "common_reg.hpp"
#include "find_constraint.h"
#include "utility.h"

#include <glog/logging.h>
#include <gflags/gflags.h>

using namespace std;
using namespace ccn;

//GFLAG Template: DEFINE_TYPE(Flag_variable_name, default_value, "Comments")
DEFINE_string(target_point_cloud_path, "", "Target pointcloud file path"); 
DEFINE_string(source_point_cloud_path, "", "Source pointcloud file path");
DEFINE_string(output_point_cloud_path, "", "Registered source pointcloud file path");

DEFINE_double(target_down_res, 0.1, "voxel size(m) of downsample for target point cloud");
DEFINE_double(source_down_res, 0.1, "voxel size(m) of downsample for source point cloud");
DEFINE_double(gf_grid_size, 2.0, "grid size(m) of ground segmentation");
DEFINE_double(gf_in_grid_h_thre, 0.3, "height threshold(m) above the lowest point in a grid for ground segmentation");
DEFINE_double(gf_neigh_grid_h_thre, 2.2, "height threshold(m) among neighbor grids for ground segmentation");
DEFINE_double(gf_max_h, 25.0, "max height(m) for a ground point");
DEFINE_double(target_pca_neigh_radius, 2.5, "pca neighborhood searching radius(m) for target point cloud");
DEFINE_double(source_pca_neigh_radius, 2.5, "pca neighborhood searching radius(m) for source point cloud");
DEFINE_double(linearity_thre, 0.65, "pca linearity threshold");
DEFINE_double(planarity_thre, 0.6, "pca planarity threshold");
DEFINE_double(spherity_thre, 0.55, "pca spherity threshold");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging("Mylog_testreg");
    LOG(INFO) << "Launch the program!";
    LOG(INFO) << "Logging is written to " << FLAGS_log_dir;

    // CHECK(FLAGS_target_point_cloud_path != "") << "Need to specify the target point cloud.";
    // CHECK(FLAGS_source_point_cloud_path != "") << "Need to specify the source point cloud.";
    // CHECK(FLAGS_output_point_cloud_path != "") << "Need to specify where to save the registered point cloud.";

    //Import configuration
    //Data path (4 formats are available: *.pcd, *.las, *.ply, *.txt)
    std::string filenameT = FLAGS_target_point_cloud_path; //Target pointcloud file path
    std::string filenameS = FLAGS_source_point_cloud_path; //Source pointcloud file path
    std::string filenameR = FLAGS_output_point_cloud_path; //Registered source pointcloud file path

    float vf_downsample_resolution_source = FLAGS_target_down_res; //0.2
    float vf_downsample_resolution_target = FLAGS_source_down_res; //0.2
    float gf_grid_resolution = FLAGS_gf_grid_size;                 //2.0
    float gf_max_grid_height_diff = FLAGS_gf_in_grid_h_thre;       //0.3
    float gf_neighbor_height_diff = FLAGS_gf_neigh_grid_h_thre;    //2.5
    float gf_max_height = FLAGS_gf_max_h;
    float pca_neigh_radius_target = FLAGS_target_pca_neigh_radius; //1.0
    float pca_neigh_radius_source = FLAGS_source_pca_neigh_radius;
    float pca_linearity_thre = FLAGS_linearity_thre;
    float pca_planarity_thre = FLAGS_planarity_thre;
    float pca_spherity_thre = FLAGS_spherity_thre;

    cloudblock_t cblock_T, cblock_S;
    cblock_T.filename = filenameT;
    cblock_S.filename = filenameS;

    //Import the data
    DataIo<Point_T> dataio;
    dataio.readCloudBlockPointCloud(cblock_T);
    dataio.readCloudBlockPointCloud(cblock_S);

    //Filter
    CFilter<Point_T> cfilter;
    cfilter.ExtractSparseGeoFeaturePoints(cblock_T, vf_downsample_resolution_target, gf_grid_resolution, gf_max_grid_height_diff,
                                          gf_neighbor_height_diff, gf_max_height, pca_neigh_radius_target,
                                          pca_linearity_thre, pca_planarity_thre, pca_spherity_thre, 0);

    cfilter.ExtractSparseGeoFeaturePoints(cblock_S, vf_downsample_resolution_source, gf_grid_resolution, gf_max_grid_height_diff,
                                          gf_neighbor_height_diff, gf_max_height, pca_neigh_radius_source,
                                          pca_linearity_thre, pca_planarity_thre, pca_spherity_thre, 0);

    MapViewer<Point_T> viewer;

    LOG(WARNING) << "Ground - Silver, Pillar - Green, Beam - Yellow, Facade - Blue, Roof - Red, Vertex - Purple";
    viewer.DispalyFeaturePoint(cblock_T, 0, "Target Point Cloud (dense)");
    viewer.DispalyFeaturePoint(cblock_S, 1, "Source Point Cloud (sparse)");

    //Registration
    constraint_t reg_cons;
    reg_cons.block1 = cblock_T;
    reg_cons.block2 = cblock_S;

    CRegistration<pcl::PointNormal> creg;
    creg.feature_pts_lls_icp(reg_cons);

    pcl::PointCloud<Point_T>::Ptr pointCloudS_reg(new pcl::PointCloud<Point_T>());

    pcl::transformPointCloud(*reg_cons.block2.pc_raw, *pointCloudS_reg, reg_cons.Trans1_2);
    dataio.writeCloudFile(filenameR, pointCloudS_reg); //Write out the transformed Source Point Cloud

    viewer.Dispaly2Cloud(pointCloudS_reg, reg_cons.block1.pc_raw, "Registration Result", 5); //Show the registration result

    //pcl::PointCloud<Point_T>::Ptr pointCloudT(new pcl::PointCloud<Point_T>()), pointCloudS(new pcl::PointCloud<Point_T>());
    //dataio.readCloudFile(filenameT, pointCloudT);
    //dataio.readCloudFile(filenameS, pointCloudS);

    // //Downsampling
    // CFilter<Point_T> cfilter;
    // pcl::PointCloud<Point_T>::Ptr pointCloudT_down(new pcl::PointCloud<Point_T>()), pointCloudS_down(new pcl::PointCloud<Point_T>());
    // cfilter.voxelfilter(pointCloudT, pointCloudT_down, vf_downsample_resolution_target);
    // cfilter.voxelfilter(pointCloudS, pointCloudS_down, vf_downsample_resolution_source);

    // pcl::PointCloud<Point_T>::Ptr pointCloudT_down_ground(new pcl::PointCloud<Point_T>()), pointCloudT_ground(new pcl::PointCloud<Point_T>());
    // pcl::PointCloud<Point_T>::Ptr pointCloudT_down_edge(new pcl::PointCloud<Point_T>()), pointCloudT_edge(new pcl::PointCloud<Point_T>());
    // pcl::PointCloud<Point_T>::Ptr pointCloudT_down_facade(new pcl::PointCloud<Point_T>()), pointCloudT_facade(new pcl::PointCloud<Point_T>());
    // pcl::PointCloud<Point_T>::Ptr pointCloudT_down_cluster(new pcl::PointCloud<Point_T>()), pointCloudT_cluster(new pcl::PointCloud<Point_T>());
    // pcl::PointCloud<Point_T>::Ptr pointCloudS_down_ground(new pcl::PointCloud<Point_T>()), pointCloudS_ground(new pcl::PointCloud<Point_T>());
    // pcl::PointCloud<Point_T>::Ptr pointCloudS_down_edge(new pcl::PointCloud<Point_T>()), pointCloudS_edge(new pcl::PointCloud<Point_T>());
    // pcl::PointCloud<Point_T>::Ptr pointCloudS_down_facade(new pcl::PointCloud<Point_T>()), pointCloudS_facade(new pcl::PointCloud<Point_T>());
    // pcl::PointCloud<Point_T>::Ptr pointCloudS_down_cluster(new pcl::PointCloud<Point_T>()), pointCloudS_cluster(new pcl::PointCloud<Point_T>());
    // pcl::PointCloud<Point_T>::Ptr pointCloudT_unground(new pcl::PointCloud<Point_T>()), pointCloudS_unground(new pcl::PointCloud<Point_T>());

    // int gf_min_grid_num = 20;
    // float gf_max_ground_height = 500.0;
    // int gf_downsample_rate_nonground = 4;
    // int gf_downsample_rate_ground_first = 10;  //For reference ground points (more point number)
    // int gf_downsample_rate_ground_second = 40; //For search ground points (less point number)

    // int feature_min_neighbor_point_num = 15;
    // float edge_point_thre_low = 0.65;     //For reference edge points (more point number)
    // float planar_point_thre_low = 0.65;   //For reference planar points (more point number)
    // float sphere_point_thre_low = 0.65;   //For reference sphere points (more point number)
    // float edge_point_thre_high = 0.75;    //For search edge points (less point number)
    // float planar_point_thre_high = 0.75;  //For search planar points (less point number)
    // float sphere_point_thre_high = 0.75;  //For search sphere points (less point number)

    // float dis_thre_ground = dis_thre_corre;
    // float dis_thre_edge = dis_thre_corre;
    // float dis_thre_facade = dis_thre_corre;
    // float dis_thre_cluster = dis_thre_corre;

    // cfilter.FastGroundFilter(pointCloudT_down, pointCloudT_ground, pointCloudT_down_ground, pointCloudT_unground, gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff, gf_neighbor_height_diff,
    //                          gf_max_ground_height, gf_downsample_rate_ground_first, gf_downsample_rate_ground_second, gf_downsample_rate_nonground);

    // cfilter.FastGroundFilter(pointCloudS_down, pointCloudS_ground, pointCloudS_down_ground, pointCloudS_unground, gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff, gf_neighbor_height_diff,
    //                          gf_max_ground_height, gf_downsample_rate_ground_first, gf_downsample_rate_ground_second, gf_downsample_rate_nonground);

    // cfilter.RoughClassify(pointCloudT_unground, pointCloudT_edge, pointCloudT_facade, pointCloudT_cluster, pointCloudT_down_edge, pointCloudT_down_facade, pointCloudT_down_cluster,
    //                       neighbor_search_radius, feature_min_neighbor_point_num, edge_point_thre_low, planar_point_thre_low, sphere_point_thre_low, edge_point_thre_high, planar_point_thre_high, sphere_point_thre_high);

    // cfilter.RoughClassify(pointCloudS_unground, pointCloudS_edge, pointCloudS_facade, pointCloudS_cluster, pointCloudS_down_edge, pointCloudS_down_facade, pointCloudS_down_cluster,
    //                       neighbor_search_radius, feature_min_neighbor_point_num, edge_point_thre_low, planar_point_thre_low, sphere_point_thre_low, edge_point_thre_high, planar_point_thre_high, sphere_point_thre_high);

    // Eigen::Matrix4f transformationS2T;
    // Eigen::Matrix<float, 6, 6> information_matrix;
    // CRegistration<pcl::PointNormal> creg;

    // creg.feature_pts_registration(pointCloudS_down_ground, pointCloudS_down_edge, pointCloudS_down_facade, pointCloudS_down_cluster,
    //                               pointCloudT_ground, pointCloudT_edge, pointCloudT_facade, pointCloudT_cluster,
    //                               transformationS2T, information_matrix,
    //                               20, dis_thre_ground, dis_thre_edge, dis_thre_facade, dis_thre_cluster);

    // pcl::PointCloud<Point_T>::Ptr pointCloudS_reg(new pcl::PointCloud<Point_T>());

    // pcl::transformPointCloud(*pointCloudS, *pointCloudS_reg, transformationS2T);
    // dataio.writeCloudFile(filenameR, pointCloudS_reg); //Write out the transformed Source Point Cloud

    // viewer.Dispaly2Cloud(pointCloudS_reg, pointCloudT, "Registration Result", 5); //Show the registration result

    return 1;
}