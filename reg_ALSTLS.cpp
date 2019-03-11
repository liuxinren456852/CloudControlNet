//
// This file is for the general implements of ICP and its variants for fine registration
// Dependent 3rd Libs: PCL (>1.7)  
// Author: Yue Pan et al. @ WHU LIESMARS
//

#include <pcl/registration/icp.h> 
#include <pcl/registration/gicp.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/from_meshes.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>

#include <Eigen/dense>
#include "reg_ALSTLS.h"

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
template<typename PointT> 
void ICPs<PointT>::icp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	int max_iter,
	bool use_reciprocal_correspondence,
	bool use_trimmed_rejector,
	float thre_dis)
{
	clock_t t0, t1;
	t0=clock();
	
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	
	// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
	
	// Trimmed or not? [ Use a predefined overlap ratio to trim part of the correspondence with bigger distance ]
	if (use_trimmed_rejector){
		pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorTrimmed);
		trimmed_cr->setOverlapRatio(calOverlap(SourceCloud,TargetCloud,thre_dis));
		icp.addCorrespondenceRejector(trimmed_cr);
	}

	icp.setInputSource(SourceCloud);
	icp.setInputTarget(TargetCloud);

	// Converge criterion ( 'Or' Relation ) 
	// Set the maximum number of iterations [ n>x ] (criterion 1)
	icp.setMaximumIterations(max_iter);     //Most likely to happen
	// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	icp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	icp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen
	
	icp.align(*TransformedSource);  //Use closed-form SVD to estimate transformation for each iteration [You can switch to L-M Optimization]
	transformationS2T = icp.getFinalTransformation();
	
	t1=clock();

	// Commented these out if you don't want to output the registration log
	cout << "Point-to-Point ICP done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
	cout << "The fitness score of this registration is " <<icp.getFitnessScore()<<endl;
	cout << "-----------------------------------------------------------------------------" << endl;
}


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
template<typename PointT>
void ICPs<PointT>::ptplicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	int max_iter,
	bool use_reciprocal_correspondence,
	bool use_trimmed_rejector,
	float thre_dis)
{
	clock_t t0, t1;
	t0=clock();
	
	// In this case, The Cloud's Normal hasn't been calculated yet. 
	// pcl::PointCloud<pcl::Normal>::Ptr SourceNormal(new pcl::PointCloud<pcl::Normal>), TargetNormal(new pcl::PointCloud<pcl::Normal>);
	// pcl::PointCloud<pcl::PointXYZINormal>::Ptr SourceCloudwithNormal(new pcl::PointCloud<pcl::PointXYZINormal>), TargetCloudwithNormal(new pcl::PointCloud<pcl::PointXYZINormal>);
	// calNormal(SourceCloud, SourceNormal, 1.0);
	// calNormal(TargetCloud, TargetNormal, 1.0);
	// PointcloudwithNormal(SourceCloud, SourceNormal, SourceCloudwithNormal);
	// PointcloudwithNormal(TargetCloud, TargetNormal, TargetCloudwithNormal);

	pcl::IterativeClosestPointWithNormals<PointT, PointT> ptplicp;
	
	// ptplicp.setInputSource(SourceCloudwithNormal);
	// ptplicp.setInputTarget(TargetCloudwithNormal);
	
	// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	ptplicp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

	// Trimmed or not? [ Use a predefined overlap ratio to trim part of the correspondence with bigger distance ]
	if (use_trimmed_rejector){
		pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorTrimmed);
		trimmed_cr->setOverlapRatio(calOverlap(SourceCloud, TargetCloud, thre_dis));
		ptplicp.addCorrespondenceRejector(trimmed_cr);
	}

	ptplicp.setInputSource(SourceCloud);
	ptplicp.setInputTarget(TargetCloud);
	
	// Converge criterion ( 'Or' Relation ) 
	// Set the maximum number of iterations [ n>x ] (criterion 1)
	ptplicp.setMaximumIterations(max_iter);     //Most likely to happen
	// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	ptplicp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	ptplicp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen
	
	ptplicp.align(*TransformedSource);  //Use Linear Least Square Method to estimate transformation for each iteration

	transformationS2T = ptplicp.getFinalTransformation();

	t1=clock();

	// Commented these out if you don't want to output the registration log
	cout << "Point-to-Plane ICP done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
	cout << "The fitness score of this registration is " << ptplicp.getFitnessScore() << endl;
	cout << "-----------------------------------------------------------------------------" << endl;
}

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
template<typename PointT>
void ICPs<PointT>::gicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	int k_neighbor_covariance,
	int max_iter,
	bool use_reciprocal_correspondence,
	bool use_trimmed_rejector,
	float thre_dis)
{	
	clock_t t0, t1;
	t0=clock();
	
	pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
	
	// Set the number of points used to calculated the covariance of a point
	gicp.setCorrespondenceRandomness(k_neighbor_covariance);
	
	// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	gicp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

	// Trimmed or not? [ Use a predefined overlap ratio to trim part of the correspondence with bigger distance ]
	if (use_trimmed_rejector){
		pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorTrimmed);
		trimmed_cr->setOverlapRatio(calOverlap(SourceCloud, TargetCloud, thre_dis));
		gicp.addCorrespondenceRejector(trimmed_cr);
	}
	
	gicp.setMaxCorrespondenceDistance(1e6); //A large value

	gicp.setInputSource(SourceCloud);
	gicp.setInputTarget(TargetCloud);

	// Converge criterion ( 'Or' Relation ) 
	// According to gicp.hpp, the iteration terminates when one of the three happens
	// Set the maximum number of iterations  (criterion 1)
	gicp.setMaximumIterations(max_iter);     //Most likely to happen
	// Set the transformation difference threshold (criterion 2)
	gicp.setTransformationEpsilon(1e-8);     //Hard to happen
	// Set the rotation difference threshold (criterion 3) 
	gicp.setRotationEpsilon(1e-6);           //Hard to happen

	gicp.align(*TransformedSource);

	transformationS2T = gicp.getFinalTransformation();

	t1=clock();

	// Commented these out if you don't want to output the registration log
	cout << "GICP done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
	cout << "The fitness score of this registration is " << gicp.getFitnessScore() << endl;
	cout << "-----------------------------------------------------------------------------" << endl;
}


/**
* \brief Estimated the approximate overlapping ratio for Cloud1 considering Cloud2
* \param[in]  Cloud1 : A pointer of the Point Cloud used for overlap ratio calculation
* \param[in]  Cloud2 : A pointer of the Point Cloud overlapped with Cloud1
* \param[out] thre_dis : It acts as the search radius of overlapping estimation
* \return : The estimated overlap ratio [from 0 to 1]
*/
template<typename PointT>
float ICPs<PointT>::calOverlap(const typename pcl::PointCloud<PointT>::Ptr & Cloud1,
	const typename pcl::PointCloud<PointT>::Ptr & Cloud2,
	float thre_dis)
{
	int overlap_point_num=0;
	float overlap_ratio;

	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(Cloud2);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	
	for (int i = 0; i < Cloud1->size(); i++){
		if (kdtree.radiusSearch(Cloud1->points[i], thre_dis, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) overlap_point_num++;
	}
	
	overlap_ratio = (0.01 + overlap_point_num) / Cloud1->size();
	cout << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio << endl;

	return overlap_ratio;
}

/**
* \brief Transform a Point Cloud using a given transformation matrix
* \param[in]  Cloud : A pointer of the Point Cloud before transformation
* \param[out] TransformedCloud : A pointer of the Point Cloud after transformation
* \param[in]  transformation : A 4*4 transformation matrix
*/
template<typename PointT>
void ICPs<PointT>::transformcloud(typename pcl::PointCloud<PointT>::Ptr & Cloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedCloud,
	Eigen::Matrix4f & transformation)
{
	Eigen::Matrix4Xf PC;
	Eigen::Matrix4Xf TPC; 
	PC.resize(4, Cloud->size());
	TPC.resize(4, Cloud->size());
	for (int i = 0; i < Cloud->size(); i++)
	{
		PC(0,i)= Cloud->points[i].x;
		PC(1,i)= Cloud->points[i].y;
		PC(2,i)= Cloud->points[i].z;
		PC(3,i)= 1;
	}
	TPC = transformation * PC;
	for (int i = 0; i < Cloud->size(); i++)
	{
		PointT pt;
		pt.x = TPC(0, i);
		pt.y = TPC(1, i);
		pt.z = TPC(2, i);
		TransformedCloud->points.push_back(pt);
	}
	cout << "Transform done ..." << endl;
}

/**
* \brief Get the Inverse (Not Mathimatically) of a giving 4*4 Transformation Matrix
* \param[in]  transformation : A 4*4 transformation matrix ( from Cloud A to Cloud A' )
* \param[out] invtransformation : Inversed 4*4 transformation matrix ( from Cloud A' to Cloud A )
*/
template<typename PointT>
void ICPs<PointT>::invTransform(const Eigen::Matrix4f & transformation,
	Eigen::Matrix4f & invtransformation)
{
	invtransformation.block<3, 3>(0, 0) = (transformation.block<3, 3>(0, 0)).transpose();
	invtransformation(0, 3) = -transformation(0, 3);
	invtransformation(1, 3) = -transformation(1, 3);
	invtransformation(2, 3) = -transformation(2, 3);
	invtransformation(3, 0) = 0;
	invtransformation(3, 1) = 0;
	invtransformation(3, 2) = 0;
	invtransformation(3, 3) = 1;
}


/**
* \brief Calculate the Normal of a given Point Cloud
* \param[in]  cloud : A pointer of the Point Cloud 
* \param[out] cloud_normals : A pointer of the Point Cloud's Normal 
* \param[in]  search_radius : The search radius for neighborhood covariance calculation and PCA
*/
template<typename PointT>
void ICPs<PointT>::calNormal(const typename pcl::PointCloud<PointT>::Ptr & cloud,
	typename pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
	float search_radius)
{
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(search_radius);
	ne.compute(*cloud_normals);
}


/**
* \brief Calculate the Covariance of a given Point Cloud
* \param[in]  cloud : A pointer of the Point Cloud
* \param[out] cloud_normals : A pointer of the Point Cloud's Normal
* \param[out] covariances : The covariance of the Point Cloud
* \param[in]  search_radius : The search radius for neighborhood covariance calculation and PCA
*/
template<typename PointT>
void ICPs<PointT>::calCovariance(const typename pcl::PointCloud<PointT>::Ptr & cloud,
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
	std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> & covariances,
	float search_radius)
{
	calNormal(cloud, cloud_normals, search_radius);
	pcl::features::computeApproximateCovariances(*cloud, *cloud_normals, covariances);

}


template<typename PointT>
void ICPs<PointT>::PointcloudwithNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
	const pcl::PointCloud<pcl::Normal>::Ptr & cloudnormal,
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloudwithnormal)
{
	for (int i = 0; i < cloud->size(); i++)
	{
		cloudwithnormal->points[i].x = cloud->points[i].x;
		cloudwithnormal->points[i].y = cloud->points[i].y;
		cloudwithnormal->points[i].z = cloud->points[i].z;
		cloudwithnormal->points[i].intensity = cloud->points[i].intensity;
		cloudwithnormal->points[i].normal_x = cloudnormal->points[i].normal_x;
		cloudwithnormal->points[i].normal_y = cloudnormal->points[i].normal_y;
		cloudwithnormal->points[i].normal_z = cloudnormal->points[i].normal_z;
	}
}

