//
// This file is a General Definition and Tool for Point Cloud Processing based on PCL.
// Dependent 3rd Libs: PCL (>1.7)  
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//
#ifndef UTILITY_H
#define UTILITY_H

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/fpfh.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/impl/pcl_base.hpp>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <vector>
#include <list>
#include <fstream>

using namespace std;

typedef pcl::PointXYZINormal Point_T; 

//Max and Min
#define max_(a,b) (((a) > (b)) ? (a) : (b))
#define min_(a,b) (((a) < (b)) ? (a) : (b))

//TypeDef 
typedef  pcl::PointCloud<pcl::PointXYZI>::Ptr        pcXYZIPtr;
typedef  pcl::PointCloud<pcl::PointXYZI>             pcXYZI;

typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr         pcXYZPtr;
typedef  pcl::PointCloud<pcl::PointXYZ>              pcXYZ;

typedef  pcl::PointCloud<pcl::PointXY>::Ptr          pcXYPtr;
typedef  pcl::PointCloud<pcl::PointXY>               pcXY;

typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr      pcXYZRGBPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGB>           pcXYZRGB;

typedef  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr     pcXYZRGBAPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGBA>          pcXYZRGBA;

typedef  pcl::PointCloud<pcl::Normal>::Ptr           NormalsPtr;
typedef  pcl::PointCloud<pcl::Normal>                Normals;

typedef  pcl::PointCloud<pcl::PointXYZINormal>::Ptr  pcXYZINPtr;
typedef  pcl::PointCloud<pcl::PointXYZINormal>       pcXYZIN;

typedef  pcl::PointCloud<pcl::FPFHSignature33>::Ptr  fpfhFeaturePtr;
typedef  pcl::PointCloud<pcl::FPFHSignature33>       fpfhFeature;

namespace utility
{
	struct CenterPoint 
	{
		double x;
		double y;
		double z;
		CenterPoint(double x = 0, double y = 0, double z = 0) :
			x(x), y(y), z(z)
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
    
	enum color_type{ INTENSITY, HEIGHT, SINGLE, FRAME};

	enum frame_type{ HDL64, VLP16, VLP32};
    
    enum edge_type{ REVISIT, ADJACENT };
    
	enum point_type {GROUND, EDGE, PLANAR, UNKNOWN};

    struct IMU_data 
	{
	    float ax; //Unit: m/s^2
		float ay; //Unit: m/s^2
		float az; //Unit: m/s^2
		float wx; //Unit: rad/s
		float wy; //Unit: rad/s
		float wz; //Unit: rad/s	

		timeval time_stamp;
	};

	struct Frame //Cloud Frame Node
	{
        frame_type type;

		int unique_id;
		int transaction_id;
        int id_in_transaction;
		int submap_id;
		int id_in_submap;

		string pcd_file_name;
		timeval time_stamp;
		
		Eigen::Vector3f oxts_position;
		Eigen::Matrix4f oxts_pose;
		Eigen::Matrix4f oxts_noise_pose;
		Eigen::Matrix4f odom_pose;  
		Eigen::Matrix4f optimized_pose;  
		
        vector<IMU_data> imu_datas;

		Bounds boundingbox;
		CenterPoint centerpoint;
	   
		//Lidar Station Center
		pcl::PointCloud<Point_T>::Ptr pointcloud_lidar;       
		pcl::PointCloud<Point_T>::Ptr pointcloud_lidar_down;
        
		//Feature points (Lidar Station Center)
		pcl::PointCloud<Point_T>::Ptr pointcloud_ground;
		pcl::PointCloud<Point_T>::Ptr pointcloud_ground_down;
		pcl::PointCloud<Point_T>::Ptr pointcloud_edge;
        pcl::PointCloud<Point_T>::Ptr pointcloud_edge_down;
		pcl::PointCloud<Point_T>::Ptr pointcloud_planar;
        pcl::PointCloud<Point_T>::Ptr pointcloud_planar_down;
        pcl::PointCloud<Point_T>::Ptr pointcloud_sphere;
		pcl::PointCloud<Point_T>::Ptr pointcloud_sphere_down;

        //Submap's Reference station Center
		pcl::PointCloud<Point_T>::Ptr pointcloud_submap;
		pcl::PointCloud<Point_T>::Ptr pointcloud_submap_down;
        
        //Lidar Odometry prediction result in World System
		pcl::PointCloud<Point_T>::Ptr pointcloud_odom;
		pcl::PointCloud<Point_T>::Ptr pointcloud_odom_down;

		//OXTS prediction result in World System
		pcl::PointCloud<Point_T>::Ptr pointcloud_oxts;
		pcl::PointCloud<Point_T>::Ptr pointcloud_oxts_down;
        
		//Optimized prediction result in World System
		// pcl::PointCloud<Point_T>::Ptr pointcloud_optimized;
		// pcl::PointCloud<Point_T>::Ptr pointcloud_optimized_down;
	};

    
    struct SubMap //Submap Node
	{
		frame_type type;
		
		int unique_id;
		int transaction_id;
		int id_in_transaction;
        
		int frame_number;

        //Of first frame
	    Eigen::Vector3f oxts_position;
		Eigen::Matrix4f oxts_pose;
		Eigen::Matrix4f oxts_noise_pose;
		Eigen::Matrix4f odom_pose;
		Eigen::Matrix4f optimized_pose;

		timeval time_stamp;

        Bounds boundingbox;
		CenterPoint centerpoint;
        
		vector<Frame> frames;
		
		float submap_mae;

		pcl::PointCloud<Point_T>::Ptr pointcloud_lidar;
		pcl::PointCloud<Point_T>::Ptr pointcloud_lidar_down;
		pcl::PointCloud<Point_T>::Ptr pointcloud_oxts;
		pcl::PointCloud<Point_T>::Ptr pointcloud_oxts_down;
		pcl::PointCloud<Point_T>::Ptr pointcloud_odom;
		pcl::PointCloud<Point_T>::Ptr pointcloud_odom_down;
		pcl::PointCloud<Point_T>::Ptr pointcloud_optimized;
		pcl::PointCloud<Point_T>::Ptr pointcloud_optimized_down;
	};
    

	struct Edge_between_2Frames //Registration edge between two cloud frame nodes
	{
		int unique_id;
		edge_type type;
     
		Frame frame1,frame2;
		Eigen::Matrix4f Trans1_2 = Eigen::Matrix4f::Identity(4,4);  //Transformation from frame 2 to frame 1
		float confidence;    
	};
    
	
	struct Edge_between_2Submaps //Registration edge between two submap nodes
	{
		int unique_id;
		edge_type type;
     
		SubMap submap1,submap2;
		Eigen::Matrix4f Trans1_2 = Eigen::Matrix4f::Identity(4,4);  //Transformation from submap 2 to submap 1
		float confidence;    
	};

    struct Transaction //Transaction Node
	{
        frame_type type;
		
		int unique_id;
        
		int frame_number;
		int submap_number;
        
		Eigen::Matrix4f calibration_matrix;
		Eigen::Matrix4f calibration_compensate_matrix;
		Eigen::Matrix3f bore_sight;
		Eigen::Vector3f lever_arm;

		timeval time_stamp;
        
		Bounds boundingbox;
		CenterPoint centerpoint;

        vector<Frame> frames;
		vector<SubMap> submaps;
        vector<Edge_between_2Submaps> submap_edges;
        
		float transaction_mae;

		pcl::PointCloud<Point_T>::Ptr pointcloud_lidar;
		pcl::PointCloud<Point_T>::Ptr pointcloud_lidar_down;
		pcl::PointCloud<Point_T>::Ptr pointcloud_oxts;
		pcl::PointCloud<Point_T>::Ptr pointcloud_oxts_down;
		pcl::PointCloud<Point_T>::Ptr pointcloud_odom;
		pcl::PointCloud<Point_T>::Ptr pointcloud_odom_down;
		pcl::PointCloud<Point_T>::Ptr pointcloud_optimized;
		pcl::PointCloud<Point_T>::Ptr pointcloud_optimized_down;

	};

	
	template <typename PointT>
	class CloudUtility
	{

	public:
		//Get Center of a Point Cloud
		void getCloudCenterPoint(const typename pcl::PointCloud<PointT> & cloud, CenterPoint & centerPoint)
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
		void getCloudBound(const typename pcl::PointCloud<PointT> & cloud, Bounds & bound)
		{
			double min_x = cloud.points[0].x;
			double min_y = cloud.points[0].y;
			double min_z = cloud.points[0].z;
			double max_x = cloud.points[0].x;
			double max_y = cloud.points[0].y;
			double max_z = cloud.points[0].z;

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
		void getBoundAndCenter(const typename pcl::PointCloud<PointT> & cloud, Bounds & bound, CenterPoint& centerPoint)
		{
			getCloudCenterPoint(cloud, centerPoint);
			getCloudBound(cloud, bound);
		}

		//Get Bound of Subsets of a Point Cloud
		void GetSubsetBoundary(typename pcl::PointCloud<PointT>::Ptr & cloud, vector<int> & index, Bounds & bound)
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
}

#endif //UTILITY_H