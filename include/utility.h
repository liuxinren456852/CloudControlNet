//
// This file is a General Definition and Tool for Point Cloud Processing based on PCL.
// Dependent 3rd Libs: PCL (>1.7)
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//
#ifndef _INCLUDE_UTILITY_H
#define _INCLUDE_UTILITY_H

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/point_representation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <vector>
#include <list>

#include <glog/logging.h>

#define max_(a, b) (((a) > (b)) ? (a) : (b))
#define min_(a, b) (((a) < (b)) ? (a) : (b))

using namespace std;

//TypeDef
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pcXYZIPtr;
typedef pcl::PointCloud<pcl::PointXYZI> pcXYZI;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZ> pcXYZ;

typedef pcl::PointCloud<pcl::PointXY>::Ptr pcXYPtr;
typedef pcl::PointCloud<pcl::PointXY> pcXY;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> pcXYZRGB;

typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcXYZRGBAPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBA> pcXYZRGBA;

typedef pcl::PointCloud<pcl::Normal>::Ptr NormalsPtr;
typedef pcl::PointCloud<pcl::Normal> Normals;

typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcXYZINPtr;
typedef pcl::PointCloud<pcl::PointXYZINormal> pcXYZIN;

typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeaturePtr;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

namespace ccn
{

struct CenterPoint
{
	double x;
	double y;
	double z;
	CenterPoint(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z)
	{
		z = 0.0;
		x = y = 0.0;
	}
};

struct Bounds
{
	double min_x;
	double min_y;
	double min_z;
	double max_x;
	double max_y;
	double max_z;
	Bounds()
	{
		min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
	}
};

struct Grid
{
	bool is_empty;
	Grid()
	{
		is_empty = true;
	}
};

struct Voxel
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
	Voxel()
	{
		min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
		PointsNumber = 1;
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

enum DataType
{
	ALS,
	TLS,
	MLS,
	BPLS,
	RGBD
};

enum ConstraintType
{
	REGISTRATION,
	ADJACENT,
	None
};

struct cloudblock_t
{
	int unique_index;				//Unique ID of the index
	int strip_num;					//Strip ID
	int num_in_strip;				//ID in the strip
	DataType data_type;				//Datatype
	Bounds bound;					//Bounding Box
	CenterPoint cp;					//Center Point
	Eigen::Matrix4f optimized_pose; //Transformation Matrix after optimization

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


typedef std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> strip;
typedef std::vector<strip> strips;

struct constraint_t
{
	cloudblock_t block1, block2; //Two block
	ConstraintType con_type;	 //ConstraintType
	Eigen::Matrix4f Trans1_2 = Eigen::Matrix4f::Identity(4, 4);
	float confidence;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::vector<constraint_t, Eigen::aligned_allocator<constraint_t>> constraints;

template <typename PointT>
class CloudUtility
{
  public:
	//Get Center of a Point Cloud
	void getCloudCenterPoint(const typename pcl::PointCloud<PointT> &cloud, CenterPoint &centerPoint)
	{
		double cx = 0, cy = 0, cz = 0;

		for (int i = 0; i < cloud.size(); i++)
		{
			cx += cloud.points[i].x / cloud.size();
			cy += cloud.points[i].y / cloud.size();
			cz += cloud.points[i].z / cloud.size();
		}
		centerPoint.x = cx;
		centerPoint.y = cy;
		centerPoint.z = cz;
	}

	//Get Bound of a Point Cloud
	void getCloudBound(const typename pcl::PointCloud<PointT> &cloud, Bounds &bound)
	{
		double min_x = cloud[0].x;
		double min_y = cloud[0].y;
		double min_z = cloud[0].z;
		double max_x = cloud[0].x;
		double max_y = cloud[0].y;
		double max_z = cloud[0].z;

		for (int i = 0; i < cloud.size(); i++)
		{
			if (min_x > cloud.points[i].x)
				min_x = cloud.points[i].x;
			if (min_y > cloud.points[i].y)
				min_y = cloud.points[i].y;
			if (min_z > cloud.points[i].z)
				min_z = cloud.points[i].z;
			if (max_x < cloud.points[i].x)
				max_x = cloud.points[i].x;
			if (max_y < cloud.points[i].y)
				max_y = cloud.points[i].y;
			if (max_z < cloud.points[i].z)
				max_z = cloud.points[i].z;
		}
		bound.min_x = min_x;
		bound.max_x = max_x;
		bound.min_y = min_y;
		bound.max_y = max_y;
		bound.min_z = min_z;
		bound.max_z = max_z;
	}

	//Get Bound and Center of a Point Cloud
	void getBoundAndCenter(const typename pcl::PointCloud<PointT> &cloud, Bounds &bound, CenterPoint &centerPoint)
	{
		getCloudCenterPoint(cloud, centerPoint);
		getCloudBound(cloud, bound);
	}

	//Get Bound of Subsets of a Point Cloud
	void GetSubsetBoundary(typename pcl::PointCloud<PointT>::Ptr &cloud, vector<int> &index, Bounds &bound)
	{
		typename pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
		for (int i = 0; i < index.size(); i++)
		{
			temp_cloud->push_back(cloud->points[index[i]]);
		}
		getCloudBound(*temp_cloud, bound);
	}

  protected:
  private:
};
} // namespace ccn

#endif //_INCLUDE_UTILITY_H