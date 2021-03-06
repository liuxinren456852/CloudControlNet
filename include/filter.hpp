//
// This file is used for the filtering of Point Cloud.
// Dependent 3rd Libs: PCL (>1.7)
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#ifndef _INCLUDE_FILTER_HPP
#define _INCLUDE_FILTER_HPP

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/normal_space.h>

#include <vector>
#include <iostream>

#include "utility.h"
#include "pca.h"

namespace ccn
{
template <typename PointT>
class CFilter : public CloudUtility<PointT>
{
  public:
	struct IDPair
	{
		IDPair() : idx(0), voxel_idx(0) {}

		unsigned long long voxel_idx;
		unsigned int idx;

		bool operator<(const IDPair &pair) { return voxel_idx < pair.voxel_idx; }
	};

	struct Grid
	{
		vector<int> point_id;
		float min_z;
		float max_z;
		float dertaz;
		float min_z_x; //X of Lowest Point in the Voxel;
		float min_z_y; //Y of Lowest Point in the Voxel;
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
		vector<int> point_id;
		float max_curvature;
		int max_curvature_point_id;
		bool has_keypoint;
		SimplifiedVoxel()
		{
			has_keypoint = false;
		}
	};

	bool voxelfilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, float voxel_size)
	{
		float inverse_voxel_size = 1.0f / voxel_size;

		Eigen::Vector4f min_p, max_p;
		pcl::getMinMax3D(*cloud_in, min_p, max_p);

		Eigen::Vector4f gap_p; //boundingbox gap;
		gap_p = max_p - min_p;

		unsigned long long max_vx = ceil(gap_p.coeff(0) * inverse_voxel_size) + 1;
		unsigned long long max_vy = ceil(gap_p.coeff(1) * inverse_voxel_size) + 1;
		unsigned long long max_vz = ceil(gap_p.coeff(2) * inverse_voxel_size) + 1;

		if (max_vx * max_vy * max_vz >= std::numeric_limits<unsigned long long>::max())
		{
			std::cout << "Filtering Failed: The number of box exceed the limit." << std::endl;
			return 0;
		}

		unsigned long long mul_vx = max_vy * max_vz;
		unsigned long long mul_vy = max_vz;
		unsigned long long mul_vz = 1;

		std::vector<IDPair> id_pairs(cloud_in->size());
		unsigned int idx = 0;
		for (typename pcl::PointCloud<PointT>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++)
		{

			unsigned long long vx = floor((it->x - min_p.coeff(0)) * inverse_voxel_size);
			unsigned long long vy = floor((it->y - min_p.coeff(1)) * inverse_voxel_size);
			unsigned long long vz = floor((it->z - min_p.coeff(2)) * inverse_voxel_size);

			unsigned long long voxel_idx = vx * mul_vx + vy * mul_vy + vz * mul_vz;

			IDPair pair;
			pair.idx = idx;
			pair.voxel_idx = voxel_idx;
			id_pairs.push_back(pair);
			idx++;
		}

		//Do sorting
		std::sort(id_pairs.begin(), id_pairs.end());

		unsigned int begin_id = 0;

		while (begin_id < id_pairs.size())
		{
			cloud_out->push_back(cloud_in->points[id_pairs[begin_id].idx]);

			unsigned int compare_id = begin_id + 1;
			while (compare_id < id_pairs.size() && id_pairs[begin_id].voxel_idx == id_pairs[compare_id].voxel_idx)
				compare_id++;
			begin_id = compare_id;
		}

		std::cout << "Downsample done (" << cloud_out->points.size() << " points)" << std::endl;
		return 1;
	}

	//Normal Space Downsampling with radius neighbor search
	bool NormalDownsample(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, float neighbor_radius, float downsample_ratio)
	{
		clock_t t0, t1;
		t0 = clock();

		pcl::PointCloud<pcl::PointXYZ>::Ptr CloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
		copyPointCloud(*cloud_in, *CloudXYZ);
		// In this case, The Cloud's Normal hasn't been calculated yet.

		pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal(new pcl::PointCloud<pcl::PointNormal>());
		pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal_out(new pcl::PointCloud<pcl::PointNormal>());

		//Estimate Normal Multi-thread
		PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator;

		//Radius search
		pca_estimator.CalculatePointCloudWithNormal_Radius(CloudXYZ, neighbor_radius, CloudNormal);

		pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> nss;
		nss.setInputCloud(CloudNormal);
		nss.setNormals(CloudNormal);
		nss.setBins(4, 4, 4);
		nss.setSeed(0);
		nss.setSample(static_cast<unsigned int>(downsample_ratio * CloudNormal->size()));

		nss.filter(*CloudNormal_out);
		copyPointCloud(*CloudNormal_out, *cloud_out);

		t1 = clock();
		cout << "Normal space downsampling done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl;
		return 1;
	}

	//Normal Space Downsampling with KNN neighbor search
	bool NormalDownsample(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, int K, float downsample_ratio)
	{
		clock_t t0, t1;
		t0 = clock();

		pcl::PointCloud<pcl::PointXYZ>::Ptr CloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
		copyPointCloud(*cloud_in, *CloudXYZ);
		// In this case, The Cloud's Normal hasn't been calculated yet.

		pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal(new pcl::PointCloud<pcl::PointNormal>());
		pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal_out(new pcl::PointCloud<pcl::PointNormal>());

		//Estimate Normal Multi-thread
		PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator;

		//K search
		pca_estimator.CalculatePointCloudWithNormal_KNN(CloudXYZ, K, CloudNormal);

		pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> nss;
		nss.setInputCloud(CloudNormal);
		nss.setNormals(CloudNormal);
		nss.setBins(4, 4, 4);
		nss.setSeed(0);
		nss.setSample(static_cast<unsigned int>(downsample_ratio * CloudNormal->size()));

		nss.filter(*CloudNormal_out);
		copyPointCloud(*CloudNormal_out, *cloud_out);

		t1 = clock();
		cout << "Normal space downsampling done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl;
		return 1;
	}

	//SOR (Statisics Outliers Remover);
	bool SORFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, int MeanK, double std)
	{
		// Create the filtering object
		pcl::StatisticalOutlierRemoval<PointT> sor;

		sor.setInputCloud(cloud_in);
		sor.setMeanK(MeanK);		 //50
		sor.setStddevMulThresh(std); //2.0
		sor.filter(*cloud_out);

		return 1;
	}

	bool RandomDownsample(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
						  typename pcl::PointCloud<PointT>::Ptr &cloud_out, int downsample_ratio)
	{
		cloud_out->points.clear();

		for (size_t i = 0; i < cloud_in->points.size(); i++)
		{

			if (i % downsample_ratio == 0)
				cloud_out->points.push_back(cloud_in->points[i]);
		}
	}

	//Filter the point cloud according to the horizontal and vertical distance to the lidar center
	bool DisFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, double xy_dis_max, double z_min, double z_max)
	{
		double dis_square;
		for (int i = 0; i < cloud_in->points.size(); i++)
		{
			dis_square = cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y + cloud_in->points[i].y;
			if (dis_square < xy_dis_max * xy_dis_max && cloud_in->points[i].z < z_max && cloud_in->points[i].z > z_min)
			{
				cloud_out->points.push_back(cloud_in->points[i]);
			}
		}
		return 1;
	}

	bool ActiveObjectFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, std::vector<bounds_t> &active_bbxs)
	{
		std::vector<bool> is_static(cloud_in->points.size(), 1);
		for (int i = 0; i < cloud_in->points.size(); i++)
		{
			for (int j = 0; j < active_bbxs.size(); j++)
			{
				//In the bounding box
				if (cloud_in->points[i].x > active_bbxs[j].min_x && cloud_in->points[i].x < active_bbxs[j].max_x &&
					cloud_in->points[i].y > active_bbxs[j].min_y && cloud_in->points[i].y < active_bbxs[j].max_y &&
					cloud_in->points[i].z > active_bbxs[j].min_z && cloud_in->points[i].z < active_bbxs[j].max_z)
				{
					is_static[i] = 0;
					break;
				}
			}
			if (is_static[i])
				cloud_out->points.push_back(cloud_in->points[i]);
		}

		return 1;
	}

	// Two threshold Fast Ground filter
	// 1.Construct 2D grid
	// 2.Calculate the Minimum Z value in each grid
	// 3.For each grid, if its 8 neighbor grids' Minimum Z value is less than current grid's Minimum Z minus threshold1, then all the points in current grid would be seen as unground points
	// 4.Or, points whose Z value is larger than grid's Minimum Z plus threshold2 would be regarded as unground points. The rest points in the grid would be ground points.
	// (Estimate Ground Points' normal at the same time)
	bool FastGroundFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_ground,
						  typename pcl::PointCloud<PointT>::Ptr &cloud_ground_down, typename pcl::PointCloud<PointT>::Ptr &cloud_unground,
						  int min_grid_num, float grid_resolution, float max_height_difference, float neighbor_height_diff, float max_ground_height,
						  int ground_random_downsample_rate_first, int ground_random_downsample_rate_second, int nonground_random_downsample_rate)
	{
		clock_t t0, t1, t2;
		t0 = clock();

		PrincipleComponentAnalysis<PointT> pca_estimator;

		bounds_t bounds;
		centerpoint_t center_pt;
		this->getBoundAndCenter(*cloud_in, bounds, center_pt); //Inherited from its parent class, use this->

		//Construct Grid
		int row, col, num_grid;
		row = ceil((bounds.max_y - bounds.min_y) / grid_resolution);
		col = ceil((bounds.max_x - bounds.min_x) / grid_resolution);
		num_grid = row * col;

		Grid *grid = new Grid[num_grid];

		//Each grid
		for (int i = 0; i < num_grid; i++)
		{
			grid[i].min_z = FLT_MAX;
			grid[i].NeighborMin_z = FLT_MAX;
		}

		//Each point
		for (int j = 0; j < cloud_in->points.size(); j++)
		{
			int temp_row, temp_col, temp_id;
			temp_col = floor((cloud_in->points[j].x - bounds.min_x) / grid_resolution);
			temp_row = floor((cloud_in->points[j].y - bounds.min_y) / grid_resolution);
			temp_id = temp_row * col + temp_col;
			if (temp_id >= 0 && temp_id < num_grid)
			{
				grid[temp_id].PointsNumber++;
				if (cloud_in->points[j].z > max_ground_height)
				{
					cloud_unground->points.push_back(cloud_in->points[j]);
				}
				else
				{
					grid[temp_id].point_id.push_back(j);
					if (cloud_in->points[j].z < grid[temp_id].min_z)
					{
						grid[temp_id].min_z = cloud_in->points[j].z;
						grid[temp_id].NeighborMin_z = cloud_in->points[j].z;
					}
				}
			}
		}

		//Each grid
		for (int i = 0; i < num_grid; i++)
		{
			int temp_row, temp_col;
			temp_row = i / col;
			temp_col = i % col;
			if (temp_row >= 1 && temp_row <= row - 2 && temp_col >= 1 && temp_col <= col - 2)
			{
				for (int j = -1; j <= 1; j++) //row
				{
					for (int k = -1; k <= 1; k++) //col
					{
						if (grid[i].NeighborMin_z > grid[i + j * col + k].min_z)
							grid[i].NeighborMin_z = grid[i + j * col + k].min_z;
					}
				}
			}
		}
		//Each grid
		for (int i = 0; i < num_grid; i++)
		{
			//Filtering some grids with too little points
			if (grid[i].PointsNumber >= min_grid_num)
			{
				if (grid[i].min_z - grid[i].NeighborMin_z < neighbor_height_diff)
				{
					for (int j = 0; j < grid[i].point_id.size(); j++)
					{
						if (cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_difference)
						{
							if (j % ground_random_downsample_rate_first == 0) // for example 10
							{

								//  cloud_in->points[grid[i].point_id[j]].normal_x=0.0;
								//  cloud_in->points[grid[i].point_id[j]].normal_y=0.0;
								//  cloud_in->points[grid[i].point_id[j]].normal_z=1.0;

								cloud_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to ground points
								if (j % ground_random_downsample_rate_second == 0)					   // for example 40
								{
									cloud_ground_down->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to down ground points
								}
							}
						}
						else
						{
							if (j % nonground_random_downsample_rate == 0)
								cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
						}
					}
				}
				else
				{
					for (int j = 0; j < grid[i].point_id.size(); j++)
					{
						if (j % nonground_random_downsample_rate == 0)
							cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
					} //Add to nonground points
				}
			}
		}

		delete[] grid;

		t1 = clock();

		pcl::PointCloud<pcl::Normal>::Ptr ground_normal(new pcl::PointCloud<pcl::Normal>);
		pca_estimator.CalculateNormalVector_KNN(cloud_ground, 8, ground_normal);
		for (int i = 0; i < cloud_ground->points.size(); i++)
		{
			cloud_ground->points[i].normal_x = ground_normal->points[i].normal_x;
			cloud_ground->points[i].normal_y = ground_normal->points[i].normal_y;
			cloud_ground->points[i].normal_z = ground_normal->points[i].normal_z;
			//if (i%500==0) printf("normal: %lf, %lf, %lf\n", cloud_ground->points[i].normal_x, cloud_ground->points[i].normal_y, cloud_ground->points[i].normal_z );
		}

		t2 = clock();

		LOG(INFO) << "Ground: [" << cloud_ground->points.size() << " | " << cloud_ground_down->points.size() << "] Unground: [" << cloud_unground->points.size() << "].";
		LOG(INFO) << "Ground segmentation done in " << (float(t1 - t0) / CLOCKS_PER_SEC);
		LOG(INFO) << "Ground Normal Estimation done in " << (float(t2 - t1) / CLOCKS_PER_SEC);

		return 1;
	}

	bool RoughClassifyNG(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
						 typename pcl::PointCloud<PointT>::Ptr &cloud_pillar, typename pcl::PointCloud<PointT>::Ptr &cloud_beam,
						 typename pcl::PointCloud<PointT>::Ptr &cloud_facade, typename pcl::PointCloud<PointT>::Ptr &cloud_roof,
						 typename pcl::PointCloud<PointT>::Ptr &cloud_pillar_down, typename pcl::PointCloud<PointT>::Ptr &cloud_beam_down,
						 typename pcl::PointCloud<PointT>::Ptr &cloud_facade_down, typename pcl::PointCloud<PointT>::Ptr &cloud_roof_down,
						 typename pcl::PointCloud<PointT>::Ptr &cloud_vertex,
						 float neighbor_radius, int neigh_num_thre,
						 float edge_thre, float planar_thre, float edge_thre_down, float planar_thre_down,
						 float sphere_thre, float vertex_feature_ratio_thre,
						 float linear_vertical_cosine_high_thre, float linear_vertical_cosine_low_thre,
						 float planar_vertical_cosine_high_thre, float planar_vertical_cosine_low_thre)
	{
		clock_t t0, t1;
		t0 = clock();

		//Estimate Normal Multi-thread
		PrincipleComponentAnalysis<PointT> pca_estimator;
		vector<pcaFeature> cloud_features;
		pca_estimator.CalculatePcaFeaturesOfPointCloud(cloud_in, cloud_features, neighbor_radius);

		vector<int> index_with_feature(cloud_in->points.size(), 0); // 0 - default, 1 - pillar, 2 - beam, 3 - facade, 4 - roof

		for (int i = 0; i < cloud_in->points.size(); i++)
		{
			if (cloud_features[i].ptNum > neigh_num_thre)
			{

				if (cloud_features[i].linear_2 > edge_thre)
				{
					if (cloud_features[i].vectors.principalDirection.z() > linear_vertical_cosine_high_thre)
					{
						cloud_pillar->points.push_back(cloud_in->points[i]);
						index_with_feature[i] = 1;
					}
					else if (cloud_features[i].vectors.principalDirection.z() < linear_vertical_cosine_low_thre)
					{
						cloud_beam->points.push_back(cloud_in->points[i]);
						index_with_feature[i] = 2;
					}

					else
					{
						;
					}

					if (cloud_features[i].linear_2 > edge_thre_down)
					{
						if (cloud_features[i].vectors.principalDirection.z() > linear_vertical_cosine_high_thre)
							cloud_pillar_down->points.push_back(cloud_in->points[i]);
						else if (cloud_features[i].vectors.principalDirection.z() < linear_vertical_cosine_low_thre)
							cloud_beam_down->points.push_back(cloud_in->points[i]);
						else
						{
							;
						}
					}
				}

				else if (cloud_features[i].planar_2 > planar_thre)
				{
					if (cloud_features[i].vectors.normalDirection.z() > planar_vertical_cosine_high_thre)
					{
						cloud_roof->points.push_back(cloud_in->points[i]);
						index_with_feature[i] = 4;
					}

					else if (cloud_features[i].vectors.normalDirection.z() < planar_vertical_cosine_low_thre)
					{
						cloud_facade->points.push_back(cloud_in->points[i]);
						index_with_feature[i] = 3;
					}

					else
					{
						;
					}

					if (cloud_features[i].planar_2 > planar_thre_down)
					{
						if (cloud_features[i].vectors.normalDirection.z() > planar_vertical_cosine_high_thre)
							cloud_roof_down->points.push_back(cloud_in->points[i]);
						else if (cloud_features[i].vectors.normalDirection.z() < planar_vertical_cosine_low_thre)
							cloud_facade_down->points.push_back(cloud_in->points[i]);
						else
						{
							;
						}
					}
				}
			}
		}

		for (int i = 0; i < cloud_in->points.size(); i++)
		{
			if (cloud_features[i].ptNum > neigh_num_thre && cloud_features[i].spherical_2 > sphere_thre)
			{
				int geo_feature_point_count = 0;
				for (int j = 0; j < cloud_features[i].neighbor_indices.size(); j++)
				{
					if (index_with_feature[cloud_features[i].neighbor_indices[j]])
						geo_feature_point_count++;
				}
				if (1.0 * geo_feature_point_count / cloud_features[i].ptNum > vertex_feature_ratio_thre)
					cloud_vertex->points.push_back(cloud_in->points[i]);
			}
		}

		t1 = clock();
		LOG(INFO) << "Pillar: [" << cloud_pillar->points.size() << " | " << cloud_pillar_down->points.size() << "] Beam: [" << cloud_beam->points.size() << " | " << cloud_beam_down->points.size() << "] Facade: [" << cloud_facade->points.size() << " | " << cloud_facade_down->points.size() << "] Roof: [" << cloud_roof->points.size() << " | "
				  << cloud_roof_down->points.size() << "] Vertex: [" << cloud_vertex->points.size() << "].";
		LOG(INFO) << "Feature points extracted done in " << (float(t1 - t0) / CLOCKS_PER_SEC);

		return 1;
	}

	bool ExtractSparseGeoFeaturePoints(cloudblock_t &in_block,
									   float vf_downsample_resolution, float gf_grid_resolution, float gf_max_grid_height_diff, float gf_neighbor_height_diff, int gf_downsample_rate_ground_first,
									   float pca_neighbor_radius, float edge_thre, float planar_thre, float sphere_thre,
									   bool clear_raw_cloud = 1,
									   int gf_grid_pt_num_thre = 15, float gf_max_ground_height = 100.0, int gf_downsample_rate_nonground = 3,
									   int neigh_pt_num_thre = 10, float vertex_feature_ratio_thre = 0.5,
									   float linear_vertical_cosine_high_thre = 0.9, float linear_vertical_cosine_low_thre = 0.4,
									   float planar_vertical_cosine_high_thre = 0.9, float planar_vertical_cosine_low_thre = 0.4) //65 degree, 25 degree
	{

		voxelfilter(in_block.pc_raw, in_block.pc_down, vf_downsample_resolution);

		int gf_downsample_rate_ground_second = 3 * gf_downsample_rate_ground_first;

		FastGroundFilter(in_block.pc_down, in_block.pc_ground, in_block.pc_ground_down, in_block.pc_unground, gf_grid_pt_num_thre, gf_grid_resolution, gf_max_grid_height_diff, gf_neighbor_height_diff,
						 gf_max_ground_height, gf_downsample_rate_ground_first, gf_downsample_rate_ground_second, gf_downsample_rate_nonground);

		float edge_thre_down = edge_thre + 0.05;
		float planar_thre_down = planar_thre + 0.05;

		RoughClassifyNG(in_block.pc_unground, in_block.pc_pillar, in_block.pc_beam, in_block.pc_facade, in_block.pc_roof,
						in_block.pc_pillar_down, in_block.pc_beam_down, in_block.pc_facade_down, in_block.pc_roof_down, in_block.pc_vertex,
						pca_neighbor_radius, neigh_pt_num_thre,
						edge_thre, planar_thre, edge_thre_down, planar_thre_down, sphere_thre, vertex_feature_ratio_thre,
						linear_vertical_cosine_high_thre, linear_vertical_cosine_low_thre,
						planar_vertical_cosine_high_thre, planar_vertical_cosine_low_thre);

		if (clear_raw_cloud)
			in_block.free_raw_cloud();
	}

	bool batchdownsamplepair(const constraint_t &this_con,
							 typename pcl::PointCloud<PointT>::Ptr &cloud1,
							 typename pcl::PointCloud<PointT>::Ptr &cloud2,
							 typename pcl::PointCloud<PointT>::Ptr &subcloud1,
							 typename pcl::PointCloud<PointT>::Ptr &subcloud2,
							 float ALS_radius, float TLS_radius, float MLS_radius, float BPLS_radius)
	{
		//Down-sampling
		switch (this_con.block1.data_type)
		{
		case ALS: //ALS
			voxelfilter(cloud1, subcloud1, ALS_radius);
			break;
		case TLS: //TLS
			voxelfilter(cloud1, subcloud1, TLS_radius);
			break;
		case MLS: //MLS
			voxelfilter(cloud1, subcloud1, MLS_radius);
			break;
		case BPLS: //BPLS
			voxelfilter(cloud1, subcloud1, BPLS_radius);
			break;
		default:
			break;
		}
		switch (this_con.block2.data_type)
		{
		case ALS: //ALS
			voxelfilter(cloud2, subcloud2, ALS_radius);
			break;
		case TLS: //TLS
			voxelfilter(cloud2, subcloud2, TLS_radius);
			break;
		case MLS: //MLS
			voxelfilter(cloud2, subcloud2, MLS_radius);
			break;
		case BPLS: //BPLS
			voxelfilter(cloud2, subcloud2, BPLS_radius);
			break;
		default:
			break;
		}

		LOG(INFO) << "Down-sampled point cloud number: [ " << subcloud1->size() << "  ,  " << subcloud2->size() << "  ]" << endl;
		return 1;
	}
};
} // namespace ccn

#endif //_INCLUDE_FILTER_HPP