//
// This file is used for the Voxel Downsampling of Point Cloud.
// Dependent 3rd Libs: PCL (>1.7)  
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#ifndef CLOUD_F
#define CLOUD_F

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/normal_space.h>

#include <vector>
#include <limits>
#include <iostream>

#include "utility.h"

using namespace utility;

template<typename PointT>
class CFilter : public CloudUtility<PointT>
{
public:
    
	CFilter(){;} //Constructor

    //~CFilter(); //Destructor

    struct IDPair
	{
		IDPair() :idx(0), voxel_idx(0) {}

		unsigned long long voxel_idx;
		unsigned int idx;

		bool operator<(const IDPair& pair) { return voxel_idx < pair.voxel_idx; }
	};

	struct Grid
	{
		vector<int> point_id;
		float min_z;
		float max_z;
		float dertaz;
		float min_z_x;  //X of Lowest Point in the Voxel;
		float min_z_y;  //Y of Lowest Point in the Voxel;
		float NeighborMin_z;
		int PointsNumber;
		float mean_z;
		Grid()
		{
			min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
			PointsNumber = 0;
			dertaz = 0.0;
		}
	};
    

	struct SimplifiedVoxel
	{
		vector<int>point_id;
		float max_curvature;
		int max_curvature_point_id;
		bool has_keypoint;
		SimplifiedVoxel()
		{
			has_keypoint = false;
		}
	};
    
    bool VoxelDownsample(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out, float voxel_size);
    
	bool NormalDownsample(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out,float neighbor_radius, float downsample_ratio);
    
    bool NormalDownsample(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out,int K,float downsample_ratio);

	bool RandomDownsample(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out, float downsample_ratio);

	bool SORFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, int MeanK, double std);
    
	// Classify the non-ground point cloud into three different types
	bool RoughClassify(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_edge, typename pcl::PointCloud<PointT>::Ptr & cloud_planar, typename pcl::PointCloud<PointT>::Ptr & cloud_sphere,
                                    typename pcl::PointCloud<PointT>::Ptr & cloud_edge_down, typename pcl::PointCloud<PointT>::Ptr & cloud_planar_down, typename pcl::PointCloud<PointT>::Ptr & cloud_sphere_down,
                                    float neighbor_radius, int neigh_num_thre, float edge_thre, float planar_thre, float sphere_thre, float edge_thre_down, float planar_thre_down, float sphere_thre_down);
    
	bool DisFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, double xy_dis_max, double z_min, double z_max);  

    bool ActiveObjectFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, std::vector<Bounds> & active_bbxs);
    
	//Reference: Two-step adaptive extraction method for ground points and breaklines from lidar point clouds, Bisheng Yang, Ronggang Huang, et al. ISPRS Journal of Photogrammetry and Remote Sensing
	bool GroundFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_ground, typename pcl::PointCloud<PointT>::Ptr & cloud_unground, float grid_resolution, float max_height_difference); // 0.5 , 0.2 
	
	// Classify all the point into ground and non-ground points
	bool FastGroundFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_ground,
                                       typename pcl::PointCloud<PointT>::Ptr & cloud_ground_down, typename pcl::PointCloud<PointT>::Ptr & cloud_unground,
                                       int min_grid_num, float grid_resolution,float max_height_difference, float neighbor_height_diff, float max_ground_height, 
	                                   int ground_random_downsample_rate_first, int ground_random_downsample_rate_second,  int nonground_random_downsample_rate);

private:
    void preprocessing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, Bounds & bounds,
			int row, int col, int num_voxel, Grid* grid, float grid_resolution); //Construct the grid

	void processing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in,
			typename pcl::PointCloud<PointT>::Ptr & cloud_ground,
			typename pcl::PointCloud<PointT>::Ptr & cloud_unground,
			Grid* grid, int num_voxel, float grid_resolution, float max_height_difference); //Two threshold ground filtering

	void postprocessing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in,
			typename pcl::PointCloud<PointT>::Ptr & cloud_ground,
			typename pcl::PointCloud<PointT>::Ptr & cloud_unground); // Some of the ground points would be selected and push into the non-ground points

};


#endif //CLOUD_F