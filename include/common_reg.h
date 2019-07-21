//
// This file is for the general implements of all kinds of registration methods
// Dependent 3rd Libs: PCL (>1.7)  
// Author: Yue Pan et al. @ WHU LIESMARS
//

#ifndef COMMON_REG_H
#define COMMON_REG_H

#include <Eigen/Dense>
#include "utility.h"

using namespace utility;
using namespace std;
using namespace Eigen;

enum transform_estimator_type {SVD, LM, LLS}; 
enum correspondence_estimator_type {NN, NS};  //NN: Nearest Neighbor ; NS: Normal Shooting
enum metrics_type {Point2Point, Point2Plane, Plane2Plane};

template <typename PointT>
class CRegistration
{
public: 


    double icp_registration(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	               const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	               typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	               Eigen::Matrix4f & transformationS2T,
				   metrics_type metrics, correspondence_estimator_type ce, transform_estimator_type te,
				   bool use_reciprocal_correspondence, bool use_trimmed_rejector,
				   int max_iter, float thre_dis, float neighbor_radius);

    

    double icp_registration(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	               const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	               typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	               Eigen::Matrix4f & transformationS2T,
				   metrics_type metrics, correspondence_estimator_type ce, transform_estimator_type te,
				   bool use_reciprocal_correspondence, bool use_trimmed_rejector,
				   int max_iter, float thre_dis, int neighbor_K);
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
	* \return     mean absolute error (mae) fitness score of the registration
	*/
	double icp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
		const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
		typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
		Eigen::Matrix4f & transformationS2T,
		int max_iter,
		bool use_reciprocal_correspondence,
		bool use_trimmed_rejector,
		float thre_dis,
		transform_estimator_type te);
	
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
	* \param[in]  covariance_K : the number of neighbor points used to calculate the normal
	* \return     mean absolute error (mae) fitness score of the registration
	*/
	// Tips: The Source and Target Point Cloud must have normal (You need to calculated it outside this method). In this case, PointT should be something like PointXYZ**Normal
	double ptplicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
		const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
		typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
		Eigen::Matrix4f & transformationS2T,
		int max_iter,
		bool use_reciprocal_correspondence,
		bool use_trimmed_rejector,
		float thre_dis,
		float neighbor_radius, // You can switch to radius search
		transform_estimator_type te); 

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
	* \param[in]  covariance_K : the number of neighbor points used to calculate the normal
	* \return     mean absolute error (mae) fitness score of the registration
	*/
	// Attention please.
	// Problem : It is found that the G-ICP codes in pcl does not support the correspondence rejector because its computeTransformation method is completely different from classic icp's though gicp is inherited from icp.
	// In my opinion, this is the main reason for G-ICP's ill behavior. I am working on the G-ICP with trimmed property now.
	double gicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
		const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
		typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
		Eigen::Matrix4f & transformationS2T,
		int max_iter,
		bool use_reciprocal_correspondence,
		bool use_trimmed_rejector,
		float thre_dis,
		int covariance_K);

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
	bool LLS_4DOF(const std::vector <std::vector<double> > & coordinatesA, const std::vector <std::vector<double> > & coordinatesB, Matrix4d & TransMatrixA2B, int cp_number, double theta0_degree); //X Y Z yaw
	bool SVD_6DOF(const std::vector <std::vector<double> > & coordinatesA, const std::vector <std::vector<double> > & coordinatesB, Matrix4d & TransMatrixA2B, int cp_number);   //X Y Z roll pitch yaw

	bool CSTRAN_4DOF(const std::vector <std::vector<double> > & coordinatesA, const std::vector <std::vector<double> > & coordinatesB, std::vector<double> &transpara, int cp_number);  // X Y yaw scale
	bool CSTRAN_7DOF(const std::vector <std::vector<double> > & coordinatesA, const std::vector <std::vector<double> > & coordinatesB, std::vector<double> &transpara, int cp_number);  // X Y Z roll pitch yaw scale

	void Perturbation(const typename pcl::PointCloud<PointT>::Ptr & Cloud, typename pcl::PointCloud<PointT>::Ptr & CloudAfterPerturbation, float pertubate_value, std::vector<float> &pertubate_vector);
    
    //Registration Entrance
    bool feature_pts_registration(const typename pcl::PointCloud<PointT>::Ptr & Source_Ground, const typename pcl::PointCloud<PointT>::Ptr & Source_Edge, 
                                  const typename pcl::PointCloud<PointT>::Ptr & Source_Planar, const typename pcl::PointCloud<PointT>::Ptr & Source_Sphere,
								  const typename pcl::PointCloud<PointT>::Ptr & Target_Ground, const typename pcl::PointCloud<PointT>::Ptr & Target_Edge, 
                                  const typename pcl::PointCloud<PointT>::Ptr & Target_Planar, const typename pcl::PointCloud<PointT>::Ptr & Target_Sphere,                                                  
	                              Eigen::Matrix4f & transformationS2T, Eigen::Matrix<float, 6, 6> & information_matrix, 
								  int max_iter_num, float dis_thre_ground, float dis_thre_edge, float dis_thre_planar, float dis_thre_sphere );
    
	bool Multi_metrics_lls_estimation(const typename pcl::PointCloud<PointT>::Ptr & Source_Ground, const typename pcl::PointCloud<PointT>::Ptr & Target_Ground, boost::shared_ptr<pcl::Correspondences> &Corr_Ground,
                                      const typename pcl::PointCloud<PointT>::Ptr & Source_Edge,   const typename pcl::PointCloud<PointT>::Ptr & Target_Edge  , boost::shared_ptr<pcl::Correspondences> &Corr_Edge  ,
									  const typename pcl::PointCloud<PointT>::Ptr & Source_Planar, const typename pcl::PointCloud<PointT>::Ptr & Target_Planar, boost::shared_ptr<pcl::Correspondences> &Corr_Planar,
                                      const typename pcl::PointCloud<PointT>::Ptr & Source_Sphere, const typename pcl::PointCloud<PointT>::Ptr & Target_Sphere, boost::shared_ptr<pcl::Correspondences> &Corr_Sphere,                              
	                                  Eigen::Matrix4f & transformationS2T, Eigen::Matrix<float, 6, 6> & information_matrix);
    
	void constructTransformation (const float & tx, const float & ty, const float & tz, const float & alpha, const float & beta, const float & gamma, Eigen::Matrix4f &transformation_matrix); 

    //Add Registration edge in one line
	//bool add_registration_edge(const typename pcl::PointCloud<PointT>::Ptr & subcloud1, const typename pcl::PointCloud<PointT>::Ptr & subcloud2, Constraint &con, 
	//	float cloud_Registration_PerturbateValue, int cloud_Registration_MaxIterNumber, bool cloud_Registration_UseReciprocalCorres, bool cloud_Registration_UseTrimmedRejector, float cloud_Registration_OverlapSearchRadius, int covariance_K, float cloud_Registration_MinOverlapForReg);
    

protected:

private:
    float min_overlap_for_reg_=0.0;
};

#endif //COMMON_REG_H