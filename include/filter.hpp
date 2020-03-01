//
// This file is used for the filtering of Point Cloud.
// Dependent 3rd Libs: PCL (>1.7)
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#ifndef _INCLUDE_FILTER_HPP
#define _INCLUDE_FILTER_HPP

#include <pcl/filters/statistical_outlier_removal.h>

#include <vector>
#include <iostream>

#include "utility.h"


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

	bool ActiveObjectFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, std::vector<Bounds> &active_bbxs)
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