//
// This file is used for the Voxel Downsampling of Point Cloud.
// Dependent 3rd Libs: PCL (>1.7)  
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#ifndef VOXEL_F
#define VOXEL_F

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <vector>
#include <limits>
#include <iostream>

template<typename PointT>
class VoxelFilter
{
public:
	
	typedef typename pcl::PointCloud<PointT>		PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr	PointCloudPtr;
	
	float V_boundingbox;    //bounding box ���;
	float _voxel_size;		//������С;

	VoxelFilter(float voxel_size):_voxel_size(voxel_size) {}

	struct IDPair
	{
		IDPair() :idx(0), voxel_idx(0) {}

		unsigned long long voxel_idx;
		unsigned int idx;

		//���رȽϺ���;
		bool operator<(const IDPair& pair) { return voxel_idx < pair.voxel_idx; }
	};

	//���г�ϡ;
	PointCloudPtr filter(const PointCloudPtr& cloud_ptr)
	{
		//voxel��С�ĵ���;
		float inverse_voxel_size = 1.0f / _voxel_size;

		//��ȡ�����С;
		Eigen::Vector4f min_p, max_p;
		pcl::getMinMax3D(*cloud_ptr, min_p, max_p);

		//�����ܹ��ĸ�������;
		Eigen::Vector4f gap_p;  //boundingbox gap;
		gap_p = max_p - min_p;
		
		unsigned long long max_vx = ceil(gap_p.coeff(0)*inverse_voxel_size)+1;
		unsigned long long max_vy = ceil(gap_p.coeff(1)*inverse_voxel_size)+1;
		unsigned long long max_vz = ceil(gap_p.coeff(2)*inverse_voxel_size)+1;
		
		//�ж����������Ƿ񳬹�����ֵ;
		if (max_vx*max_vy*max_vz >= std::numeric_limits<unsigned long long>::max())
		{
			std::cout << "Filtering Failed: The number of box exceed the limit."<<endl;
		}

		//�������;
		unsigned long long mul_vx = max_vy*max_vz;
		unsigned long long mul_vy = max_vz;
		unsigned long long mul_vz = 1;

		//�������е��λ��;
		std::vector<IDPair> id_pairs(cloud_ptr->size());
		unsigned int idx = 0;
		for (typename PointCloud::iterator it = cloud_ptr->begin(); it != cloud_ptr->end(); it++)
		{
			//������;
			unsigned long long vx = floor((it->x - min_p.coeff(0))*inverse_voxel_size);
			unsigned long long vy = floor((it->y - min_p.coeff(1))*inverse_voxel_size);
			unsigned long long vz = floor((it->z - min_p.coeff(2))*inverse_voxel_size);

			//������ӱ��;
			unsigned long long voxel_idx = vx*mul_vx + vy*mul_vy + vz*mul_vz;

			IDPair pair;
			pair.idx = idx;
			pair.voxel_idx = voxel_idx;
			id_pairs.push_back(pair);
			idx++;
		}

		//��������;
		std::sort(id_pairs.begin(), id_pairs.end());

		//����ÿ�������е�һ����;
		unsigned int begin_id = 0;
		PointCloudPtr result_ptr(new PointCloud);
		while (begin_id < id_pairs.size())
		{
			//������һ����;
			result_ptr->push_back(cloud_ptr->points[id_pairs[begin_id].idx]);

			//������ͬ���ӵĵ㶼������;
			unsigned int compare_id = begin_id + 1;
			while (compare_id < id_pairs.size() && id_pairs[begin_id].voxel_idx == id_pairs[compare_id].voxel_idx)
				compare_id++;
			begin_id = compare_id;
		}
		//����boundingbox ���;
		//V_boundingbox = gap_p[0] * gap_p[1] * gap_p[2];

		return result_ptr;
	}
};

#endif //VOXEL_F