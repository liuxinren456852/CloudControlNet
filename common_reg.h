//
// This file is for the general implements of all kinds of registration methods
// Dependent 3rd Libs: PCL (>1.7)  
// Author: Yue Pan et al. @ WHU LIESMARS
//

#ifndef COMMON_REG_H
#define COMMON_REG_H

#include <Eigen/dense>

#include "utility.h"

using namespace utility;
using namespace std;
using namespace Eigen;

template <typename PointT>
class CRegistration
{
public:
	
	/**
	* \brief Point-to-Point metric ICP
	* \param[in]  SourceCloud : A pointer of the Source Point Cloud (Each point of it is used to find the nearest neighbor as correspondence)
	* \param[in]  TargetCloud : A pointer of the Target Point Cloud
	* \param[out] TransformedSource : A pointer of the Source Point Cloud after registration [ Transformed ]
	* \param[out] transformationS2T : The transformation matrix (4*4) of Source Point Cloud for this registration
	* \param[in]  max_iter : A parameter controls the max iteration number for the registration
	* \param[in]  use_reciprocal_correspondence : A parameter controls whether use reciprocal correspondence or not (bool)
	* \param[in]  use_trimmed_rejector : A parameter controls whether use trimmed correspondence rejector or not (bool)
	* \param[in]  thre_dis : A parameter used to estimate the approximate overlap ratio of Source Point Cloud. It acts as the search radius of overlapping estimation.
	*/
	void icp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
		const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
		typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
		Eigen::Matrix4f & transformationS2T,
		int max_iter,
		bool use_reciprocal_correspondence,
		bool use_trimmed_rejector,
		float thre_dis);
	

	/**
	* \brief Point-to-Plane metric ICP
	* \param[in]  SourceCloud : A pointer of the Source Point Cloud (Each point of it is used to find the nearest neighbor as correspondence)
	* \param[in]  TargetCloud : A pointer of the Target Point Cloud
	* \param[out] TransformedSource : A pointer of the Source Point Cloud after registration [ Transformed ]
	* \param[out] transformationS2T : The transformation matrix (4*4) of Source Point Cloud for this registration
	* \param[in]  max_iter : A parameter controls the max iteration number for the registration
	* \param[in]  use_reciprocal_correspondence : A parameter controls whether use reciprocal correspondence or not (bool)
	* \param[in]  use_trimmed_rejector : A parameter controls whether use trimmed correspondence rejector or not (bool)
	* \param[in]  thre_dis : A parameter used to estimate the approximate overlap ratio of Source Point Cloud. It acts as the search radius of overlapping estimation.
	*/
	// Tips: The Source and Target Point Cloud must have normal (You need to calculated it outside this method). In this case, PointT should be something like PointXYZ**Normal
	void ptplicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
		const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
		typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
		Eigen::Matrix4f & transformationS2T,
		int max_iter,
		bool use_reciprocal_correspondence,
		bool use_trimmed_rejector,
		float thre_dis);


	/**
	* \brief Generalized (Plane-to-Plane) metric ICP
	* \param[in]  SourceCloud : A pointer of the Source Point Cloud (Each point of it is used to find the nearest neighbor as correspondence)
	* \param[in]  TargetCloud : A pointer of the Target Point Cloud
	* \param[out] TransformedSource : A pointer of the Source Point Cloud after registration [ Transformed ]
	* \param[out] transformationS2T : The transformation matrix (4*4) of Source Point Cloud for this registration
	* \param[in]  k_neighbor_covariance : A parameter controls the number of points used to calculate the approximate covariance of points, which is used to accomplish the General ICP
	* \param[in]  max_iter : A parameter controls the max iteration number for the registration
	* \param[in]  use_reciprocal_correspondence : A parameter controls whether use reciprocal correspondence or not (bool)
	* \param[in]  use_trimmed_rejector : A parameter controls whether use trimmed correspondence rejector or not (bool)
	* \param[in]  thre_dis : A parameter used to estimate the approximate overlap ratio of Source Point Cloud. It acts as the search radius of overlapping estimation.
	*/
	// Attention please.
	// Problem : It is found that the G-ICP codes in pcl does not support the correspondence rejector because its computeTransformation method is completely different from classic icp's though gicp is inherited from icp.
	// In my opinion, this is the main reason for G-ICP's ill behavior. I am working on the G-ICP with trimmed property now.
	void gicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
		const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
		typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
		Eigen::Matrix4f & transformationS2T,
		int k_neighbor_covariance,
		int max_iter,
		bool use_reciprocal_correspondence,
		bool use_trimmed_rejector,
		float thre_dis);


	/**
	* \brief Estimated the approximate overlapping ratio for Cloud1 considering Cloud2
	* \param[in]  Cloud1 : A pointer of the Point Cloud used for overlap ratio calculation
	* \param[in]  Cloud2 : A pointer of the Point Cloud overlapped with Cloud1
	* \param[out] thre_dis : It acts as the search radius of overlapping estimation
	* \return : The estimated overlap ratio [from 0 to 1]
	*/
	float calOverlap(const typename pcl::PointCloud<PointT>::Ptr & Cloud1,
		const typename pcl::PointCloud<PointT>::Ptr & Cloud2,
		float thre_dis);


	/**
	* \brief Transform a Point Cloud using a given transformation matrix
	* \param[in]  Cloud : A pointer of the Point Cloud before transformation
	* \param[out] TransformedCloud : A pointer of the Point Cloud after transformation
	* \param[in]  transformation : A 4*4 transformation matrix
	*/
	void transformcloud(typename pcl::PointCloud<PointT>::Ptr & Cloud,
		typename pcl::PointCloud<PointT>::Ptr & TransformedCloud,
		Eigen::Matrix4f & transformation);


	/**
	* \brief Calculate the Normal of a given Point Cloud
	* \param[in]  cloud : A pointer of the Point Cloud
	* \param[out] cloud_normals : A pointer of the Point Cloud's Normal
	* \param[in]  search_radius : The search radius for neighborhood covariance calculation and PCA
	*/
	void calNormal(const typename pcl::PointCloud<PointT>::Ptr & cloud,
		typename pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
		float search_radius);


	/**
	* \brief Calculate the Covariance of a given Point Cloud
	* \param[in]  cloud : A pointer of the Point Cloud
	* \param[out] cloud_normals : A pointer of the Point Cloud's Normal
	* \param[out] covariances : The covariance of the Point Cloud
	* \param[in]  search_radius : The search radius for neighborhood covariance calculation and PCA
	*/
	void calCovariance(const typename pcl::PointCloud<PointT>::Ptr & cloud,
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
		std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> & covariances,
		float search_radius);


	/**
	* \brief Get the Inverse (Not Mathimatically) of a giving 4*4 Transformation Matrix
	* \param[in]  transformation : A 4*4 transformation matrix ( from Cloud A to Cloud A' )
	* \param[out] invtransformation : Inversed 4*4 transformation matrix ( from Cloud A' to Cloud A )
	*/
	void invTransform(const Eigen::Matrix4f & transformation,
		Eigen::Matrix4f & invtransformation);

	//brief: Compute fpfh_feature
	void compute_fpfh_feature(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
		fpfhFeaturePtr &cloud_fpfh,
		float search_radius);

	//brief: Accomplish Coarse registration using FPFH SAC
	void Coarsereg_FPFHSAC(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
		const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
		typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
		Eigen::Matrix4f & transformationS2T,
		float search_radius);

	
	//Brief: Using the Gauss-Newton Least Square Method to solve 4 Degree of Freedom Transformation from no less than 2 points
	//cp_number is the control points' number for LLS calculation, the rest of points are used to check the accuracy;
	bool LLS_4DOF(const std::vector <std::vector<double>> & coordinatesA, const std::vector <std::vector<double>> & coordinatesB, Matrix4d & TransMatrixA2B, int cp_number, double theta0_degree);

	bool SVD_6DOF(const std::vector <std::vector<double>> & coordinatesA, const std::vector <std::vector<double>> & coordinatesB, Matrix4d & TransMatrixA2B, int cp_number);

protected:

	void PointcloudwithNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::PointCloud<pcl::Normal>::Ptr & cloudnormal,
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloudwithnormal);

private:
};

#endif //COMMON_REG_H