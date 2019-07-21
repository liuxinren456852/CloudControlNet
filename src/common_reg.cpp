//
// This file is for the general implements of all kinds of registration methods
// Dependent 3rd Libs: PCL (>1.7)  
// Author: Yue Pan et al. @ WHU LIESMARS
//

#define _USE_MATH_DEFINES

#include <pcl/registration/icp.h> 
#include <pcl/registration/gicp.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h> 
#include <pcl/features/normal_3d_omp.h> 
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_features.h> 
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <glog/logging.h>

#include <Eigen/Dense>
#include <cmath>
#include "common_reg.h"
#include "pca.h"

// This program accomplish the fine registration between ALS block and TLS point clouds. [Use TLS as the control cloud (alignment target)]
// It also provides various methods to achieve the same goal. You can do comprehensive experiment based on this program.
// It is part of the ALS Refinement Project for Highway Expansion and Reconstruction Engineering

template<typename PointT>
double CRegistration<PointT>::icp_registration(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	               const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	               typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	               Eigen::Matrix4f & transformationS2T,
				   metrics_type metrics, correspondence_estimator_type ce, transform_estimator_type te,
				   bool use_reciprocal_correspondence, bool use_trimmed_rejector,
				   int max_iter, float thre_dis, float neighbor_radius)
{
    clock_t t0, t1, t2;
	t0=clock();
    
	double mae_nn;
	pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorVarTrimmed);
    

    switch (metrics)
	{
	case Point2Point:
    {
	    t1=clock();
	    pcl::IterativeClosestPoint<PointT, PointT> icp;
		 
        icp.setInputSource(SourceCloud);
	    icp.setInputTarget(TargetCloud);

		typename pcl::registration::TransformationEstimationSVD<PointT,PointT,float>::Ptr te_svd(new pcl::registration::TransformationEstimationSVD<PointT,PointT,float>);
        typename pcl::registration::TransformationEstimationLM<PointT,PointT,float>::Ptr te_lm(new pcl::registration::TransformationEstimationLM<PointT,PointT,float>);
		
	    switch (te)
	    {
	       case SVD:
	          icp.setTransformationEstimation(te_svd); //Use SVD 
		      break;
	       case LM:
	          icp.setTransformationEstimation(te_lm); //Use L-M Non-Linear Optimization
	          break;
	       default:  //Default svd
		      break;
	    }  
        
		// Use Reciprocal Correspondences or not? [a -> b && b -> a]
     	icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
     
	    // Trimmed or not? 
	    if (use_trimmed_rejector) icp.addCorrespondenceRejector(trimmed_cr);
	    else icp.setMaxCorrespondenceDistance(thre_dis);

        // Converge criterion ( 'Or' Relation ) 
	    // Set the maximum number of iterations [ n>x ] (criterion 1)
	    icp.setMaximumIterations(max_iter);     //Most likely to happen
	    // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	    icp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	    // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	    icp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen

	    icp.align(*TransformedSource); 

	    transformationS2T = icp.getFinalTransformation();
	
	    t2=clock();
    
	    if (use_trimmed_rejector) thre_dis=trimmed_cr->getTrimmedDistance();
	    printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);
	
	    mae_nn=icp.getFitnessScore(thre_dis);  //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

	    // Commented these out if you don't want to output the registration log
	    cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "<< TargetCloud->points.size()<< endl;
	    cout << "Point to Point ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
	    cout << "The fitness score of this registration is " << mae_nn << endl << transformationS2T << endl;

	    break;
	}
	case Point2Plane:
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr SourceCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	    pcl::PointCloud<pcl::PointXYZ>::Ptr TargetCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	    copyPointCloud(*SourceCloud, *SourceCloudXYZ);
	    copyPointCloud(*TargetCloud, *TargetCloudXYZ);
	    // In this case, The Cloud's Normal hasn't been calculated yet. 
    
	    pcl::PointCloud<pcl::PointNormal>::Ptr SourceNormal(new pcl::PointCloud<pcl::PointNormal>());
	    pcl::PointCloud<pcl::PointNormal>::Ptr TargetNormal(new pcl::PointCloud<pcl::PointNormal>());
	    pcl::PointCloud<pcl::PointNormal>::Ptr TransformedSourceN(new pcl::PointCloud<pcl::PointNormal>());
	    
	    //Estimate Normal Multi-thread
	    PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator; 
	
	    //Radius search
	    pca_estimator.CalculatePointCloudWithNormal_Radius(SourceCloudXYZ, neighbor_radius, SourceNormal); 
	    pca_estimator.CalculatePointCloudWithNormal_Radius(TargetCloudXYZ, neighbor_radius, TargetNormal);
	    //Or
	    //KNN search
	    //pca_estimator.CalculatePointCloudWithNormal_KNN(SourceCloud, covariance_K, SourceNormal);
	    //pca_estimator.CalculatePointCloudWithNormal_KNN(TargetCloud, covariance_K, TargetNormal);
        t1=clock();

		pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;   
	     
        icp.setInputSource(SourceNormal);
	    icp.setInputTarget(TargetNormal);
	    
		if (ce == NS) //Normal Shooting
		{
           pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal,pcl::PointNormal,pcl::PointNormal>::Ptr ns_est(new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal,pcl::PointNormal,pcl::PointNormal>);
	       ns_est->setInputSource(SourceNormal);
		   ns_est->setSourceNormals(SourceNormal);
		   ns_est->setInputTarget(TargetNormal);
           ns_est->setKSearch(5);
           icp.setCorrespondenceEstimation(ns_est);
		}
        
        pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal,pcl::PointNormal,float>::Ptr te_lmn(new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal,pcl::PointNormal,float>);
        pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal,pcl::PointNormal,float>::Ptr te_lls(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal,pcl::PointNormal,float>);
		pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<pcl::PointNormal,pcl::PointNormal,float>::Ptr te_lls_weight(new pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<pcl::PointNormal,pcl::PointNormal,float>);
        switch (te)
	    {
	       case LLS:
	           icp.setTransformationEstimation(te_lls); //Use Linear Least Square
		       break;
	       case LM:
	           icp.setTransformationEstimation(te_lmn); //Use L-M Non-Linear Optimization
	           break;
	       default:  //Default lls
		       break;
	    }

		// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	    icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
     
	    // Trimmed or not? 
	    if (use_trimmed_rejector) icp.addCorrespondenceRejector(trimmed_cr);
	    else icp.setMaxCorrespondenceDistance(thre_dis);

        // Converge criterion ( 'Or' Relation ) 
	    // Set the maximum number of iterations [ n>x ] (criterion 1)
	    icp.setMaximumIterations(max_iter);     //Most likely to happen
	    // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	    icp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	    // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	    icp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen
	
	    icp.align(*TransformedSourceN);

		transformationS2T = icp.getFinalTransformation();
	
	    t2=clock();
    
	    if (use_trimmed_rejector) thre_dis=trimmed_cr->getTrimmedDistance();
	    printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);
	
	    mae_nn=icp.getFitnessScore(thre_dis);  //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

	    // Commented these out if you don't want to output the registration log
	    cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "<< TargetCloud->points.size()<< endl;
	    cout << "Point-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
	    cout << "Normal Estimation in " << float(t1 - t0) / CLOCKS_PER_SEC << " s, " << "registration in " << float(t2 - t1) / CLOCKS_PER_SEC << " s."<<endl;
	    cout << "The fitness score of this registration is " << mae_nn << endl << transformationS2T << endl;
    
		copyPointCloud(*TransformedSourceN,*TransformedSource);
        pcl::PointCloud<pcl::PointNormal>().swap(*TransformedSourceN);    // Free the Memory
	    pcl::PointCloud<pcl::PointNormal>().swap(*SourceNormal);          // Free the Memory
	    pcl::PointCloud<pcl::PointNormal>().swap(*TargetNormal);          // Free the Memory
	    pcl::PointCloud<pcl::PointXYZ>().swap(*SourceCloudXYZ);           // Free the Memory
	    pcl::PointCloud<pcl::PointXYZ>().swap(*TargetCloudXYZ);           // Free the Memory
		
		break;	    
	}
	case Plane2Plane:
    {
        t1=clock();
	    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
	
	    // Set the number of points used to calculated the covariance of a point
	    // icp.setCorrespondenceRandomness(covariance_K);
		icp.setCorrespondenceRandomness(10);
         
        icp.setInputSource(SourceCloud);
	    icp.setInputTarget(TargetCloud);
        
		// Use Reciprocal Correspondences or not? [a -> b && b -> a]
  	    icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
     
	    // Trimmed or not? 
	    if (use_trimmed_rejector) icp.addCorrespondenceRejector(trimmed_cr);
	    else icp.setMaxCorrespondenceDistance(thre_dis);
        
		icp.setMaximumOptimizerIterations(10);

        // Converge criterion ( 'Or' Relation ) 
	    // Set the maximum number of iterations [ n>x ] (criterion 1)
	    icp.setMaximumIterations(max_iter);     //Most likely to happen
	    // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	    icp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	    // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	    icp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen
	
	    icp.align(*TransformedSource); 

	    transformationS2T = icp.getFinalTransformation();
	
	    t2=clock();

		if (use_trimmed_rejector) thre_dis=trimmed_cr->getTrimmedDistance();
	    printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);
	
	    mae_nn=icp.getFitnessScore(thre_dis);  //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

	    // Commented these out if you don't want to output the registration log
	    cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "<< TargetCloud->points.size()<< endl;
	    cout << "Plane-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
	    cout << "The fitness score of this registration is " << mae_nn << endl << transformationS2T << endl;
	    break;
	}
	default:
	    return -1;
		
	}
    
		
	return mae_nn;
}




template<typename PointT>
double CRegistration<PointT>::icp_registration(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	               const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	               typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	               Eigen::Matrix4f & transformationS2T,
				   metrics_type metrics, correspondence_estimator_type ce, transform_estimator_type te,
				   bool use_reciprocal_correspondence, bool use_trimmed_rejector,
				   int max_iter, float thre_dis, int neighbor_K)
{
    clock_t t0, t1, t2;
	t0=clock();
    
	double mae_nn;
	pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorVarTrimmed);
    

    switch (metrics)
	{
	case Point2Point:
    {
	    t1=clock();
	    pcl::IterativeClosestPoint<PointT, PointT> icp;
		 
        icp.setInputSource(SourceCloud);
	    icp.setInputTarget(TargetCloud);

		typename pcl::registration::TransformationEstimationSVD<PointT,PointT,float>::Ptr te_svd(new pcl::registration::TransformationEstimationSVD<PointT,PointT,float>);
        typename pcl::registration::TransformationEstimationLM<PointT,PointT,float>::Ptr te_lm(new pcl::registration::TransformationEstimationLM<PointT,PointT,float>);
		
	    switch (te)
	    {
	       case SVD:
	          icp.setTransformationEstimation(te_svd); //Use SVD 
		      break;
	       case LM:
	          icp.setTransformationEstimation(te_lm); //Use L-M Non-Linear Optimization
	          break;
	       default:  //Default svd
		      break;
	    }  
        
		// Use Reciprocal Correspondences or not? [a -> b && b -> a]
     	icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
     
	    // Trimmed or not? 
	    if (use_trimmed_rejector) icp.addCorrespondenceRejector(trimmed_cr);
	    else icp.setMaxCorrespondenceDistance(thre_dis);

        // Converge criterion ( 'Or' Relation ) 
	    // Set the maximum number of iterations [ n>x ] (criterion 1)
	    icp.setMaximumIterations(max_iter);     //Most likely to happen
	    // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	    icp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	    // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	    icp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen

	    icp.align(*TransformedSource); 

	    transformationS2T = icp.getFinalTransformation();
	
	    t2=clock();
    
	    if (use_trimmed_rejector) thre_dis=trimmed_cr->getTrimmedDistance();
	    printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);
	
	    mae_nn=icp.getFitnessScore(thre_dis);  //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

	    // Commented these out if you don't want to output the registration log
	    cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "<< TargetCloud->points.size()<< endl;
	    cout << "Point to Point ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
	    cout << "The fitness score of this registration is " << mae_nn << endl << transformationS2T << endl;

	    break;
	}
	case Point2Plane:
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr SourceCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	    pcl::PointCloud<pcl::PointXYZ>::Ptr TargetCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	    copyPointCloud(*SourceCloud, *SourceCloudXYZ);
	    copyPointCloud(*TargetCloud, *TargetCloudXYZ);
	    // In this case, The Cloud's Normal hasn't been calculated yet. 
    
	    pcl::PointCloud<pcl::PointNormal>::Ptr SourceNormal(new pcl::PointCloud<pcl::PointNormal>());
	    pcl::PointCloud<pcl::PointNormal>::Ptr TargetNormal(new pcl::PointCloud<pcl::PointNormal>());
	    pcl::PointCloud<pcl::PointNormal>::Ptr TransformedSourceN(new pcl::PointCloud<pcl::PointNormal>());
	
	    //Estimate Normal Multi-thread
	    PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator; 
	
	    //Radius search
	    // pca_estimator.CalculatePointCloudWithNormal_Radius(SourceCloudXYZ, neighbor_radius, SourceNormal); 
	    // pca_estimator.CalculatePointCloudWithNormal_Radius(TargetCloudXYZ, neighbor_radius, TargetNormal);
	    //Or
	    //KNN search
	    pca_estimator.CalculatePointCloudWithNormal_KNN(SourceCloudXYZ, neighbor_K, SourceNormal);
	    pca_estimator.CalculatePointCloudWithNormal_KNN(TargetCloudXYZ, neighbor_K, TargetNormal);

        t1=clock();

		pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;   
	     
        icp.setInputSource(SourceNormal);
	    icp.setInputTarget(TargetNormal);
	    
		if (ce == NS) //Normal Shooting
		{
           pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal,pcl::PointNormal,pcl::PointNormal>::Ptr ns_est(new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal,pcl::PointNormal,pcl::PointNormal>);
	       ns_est->setInputSource(SourceNormal);
		   ns_est->setSourceNormals(SourceNormal);
		   ns_est->setInputTarget(TargetNormal);
           ns_est->setKSearch(1);
           icp.setCorrespondenceEstimation(ns_est);
		}
        
        pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal,pcl::PointNormal,float>::Ptr te_lmn(new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal,pcl::PointNormal,float>);
        pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal,pcl::PointNormal,float>::Ptr te_lls(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal,pcl::PointNormal,float>);
        switch (te)
	    {
	       case LLS:
	           icp.setTransformationEstimation(te_lls); //Use Linear Least Square
		       break;
	       case LM:
	           icp.setTransformationEstimation(te_lmn); //Use L-M Non-Linear Optimization
	           break;
	       default:  //Default lls
		       break;
	    }

		// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	    icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
     
	    // Trimmed or not? 
	    if (use_trimmed_rejector) icp.addCorrespondenceRejector(trimmed_cr);
	    else icp.setMaxCorrespondenceDistance(thre_dis);

        // Converge criterion ( 'Or' Relation ) 
	    // Set the maximum number of iterations [ n>x ] (criterion 1)
	    icp.setMaximumIterations(max_iter);     //Most likely to happen
	    // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	    icp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	    // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	    icp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen
	
	    icp.align(*TransformedSourceN);

		transformationS2T = icp.getFinalTransformation();
	
	    t2=clock();
    
	    if (use_trimmed_rejector) thre_dis=trimmed_cr->getTrimmedDistance();
	    printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);
	
	    mae_nn=icp.getFitnessScore(thre_dis);  //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

	    // Commented these out if you don't want to output the registration log
	    cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "<< TargetCloud->points.size()<< endl;
	    cout << "Point-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
	    cout << "Normal Estimation in " << float(t1 - t0) / CLOCKS_PER_SEC << " s, " << "registration in " << float(t2 - t1) / CLOCKS_PER_SEC << " s."<<endl;
	    cout << "The fitness score of this registration is " << mae_nn << endl << transformationS2T << endl;
    
		copyPointCloud(*TransformedSourceN,*TransformedSource);
        pcl::PointCloud<pcl::PointNormal>().swap(*TransformedSourceN);    // Free the Memory
	    pcl::PointCloud<pcl::PointNormal>().swap(*SourceNormal);          // Free the Memory
	    pcl::PointCloud<pcl::PointNormal>().swap(*TargetNormal);          // Free the Memory
	    pcl::PointCloud<pcl::PointXYZ>().swap(*SourceCloudXYZ);           // Free the Memory
	    pcl::PointCloud<pcl::PointXYZ>().swap(*TargetCloudXYZ);           // Free the Memory
		
		break;	    
	}
	case Plane2Plane:
    {
        t1=clock();
	    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
	
	    // Set the number of points used to calculated the covariance of a point
	    // icp.setCorrespondenceRandomness(covariance_K);
		icp.setCorrespondenceRandomness(neighbor_K);
         
        icp.setInputSource(SourceCloud);
	    icp.setInputTarget(TargetCloud);
        
		// Use Reciprocal Correspondences or not? [a -> b && b -> a]
  	    icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
     
	    // Trimmed or not? 
	    if (use_trimmed_rejector) icp.addCorrespondenceRejector(trimmed_cr);
	    else icp.setMaxCorrespondenceDistance(thre_dis);
        
		icp.setMaximumOptimizerIterations(10);

        // Converge criterion ( 'Or' Relation ) 
	    // Set the maximum number of iterations [ n>x ] (criterion 1)
	    icp.setMaximumIterations(max_iter);     //Most likely to happen
	    // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	    icp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	    // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	    icp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen
	
	    icp.align(*TransformedSource); 

	    transformationS2T = icp.getFinalTransformation();
	
	    t2=clock();

		if (use_trimmed_rejector) thre_dis=trimmed_cr->getTrimmedDistance();
	    printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);
	
	    mae_nn=icp.getFitnessScore(thre_dis);  //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

	    // Commented these out if you don't want to output the registration log
	    cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "<< TargetCloud->points.size()<< endl;
	    cout << "Plane-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
	    cout << "The fitness score of this registration is " << mae_nn << endl << transformationS2T << endl;
	    break;
	}
	default:
	    return -1;
		
	}
    
		
	return mae_nn;
}

  
//Feature points Point-to-Plane ICP Estimated by LLS
template<typename PointT>
bool CRegistration<PointT>::feature_pts_registration(const typename pcl::PointCloud<PointT>::Ptr & Source_Ground, const typename pcl::PointCloud<PointT>::Ptr & Source_Edge, 
                                                     const typename pcl::PointCloud<PointT>::Ptr & Source_Planar, const typename pcl::PointCloud<PointT>::Ptr & Source_Sphere,
													 const typename pcl::PointCloud<PointT>::Ptr & Target_Ground, const typename pcl::PointCloud<PointT>::Ptr & Target_Edge, 
                                                     const typename pcl::PointCloud<PointT>::Ptr & Target_Planar, const typename pcl::PointCloud<PointT>::Ptr & Target_Sphere,                                                  
	                                                 Eigen::Matrix4f & transformationS2T, Eigen::Matrix<float, 6, 6> & information_matrix, 
													 int max_iter_num, float dis_thre_ground, float dis_thre_edge, float dis_thre_planar, float dis_thre_sphere )
{
	clock_t t0, t1, t2;
	
	int K_min=20;

	t0=clock();
	
	
	//Preprocessing
    // pcl::PointCloud<pcl::PointXYZ>::Ptr SourceCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	// pcl::PointCloud<pcl::PointXYZ>::Ptr TargetCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	// copyPointCloud(*SourceCloud, *SourceCloudXYZ);
	// copyPointCloud(*TargetCloud, *TargetCloudXYZ);
	
	t1=clock();

	transformationS2T=Eigen::Matrix4f::Identity(4,4);  

	//Iteration
	for (int i=0;i<max_iter_num;i++)
	{ 

		//Estimate Correspondence
		boost::shared_ptr<pcl::Correspondences> corrs_Ground(new pcl::Correspondences);
        boost::shared_ptr<pcl::Correspondences> corrs_Edge(new pcl::Correspondences);
		boost::shared_ptr<pcl::Correspondences> corrs_Planar(new pcl::Correspondences);
		boost::shared_ptr<pcl::Correspondences> corrs_Sphere(new pcl::Correspondences);
		boost::shared_ptr<pcl::Correspondences> corrs_Ground_f(new pcl::Correspondences);
        boost::shared_ptr<pcl::Correspondences> corrs_Edge_f(new pcl::Correspondences);
		boost::shared_ptr<pcl::Correspondences> corrs_Planar_f(new pcl::Correspondences);
		boost::shared_ptr<pcl::Correspondences> corrs_Sphere_f(new pcl::Correspondences);

        typename pcl::registration::CorrespondenceEstimation<PointT,PointT> corr_est;
		pcl::registration::CorrespondenceRejectorDistance corr_rej_dist;
        
		//For Ground points
		if (Source_Ground->points.size()>=K_min && Target_Ground->points.size()>=K_min)
		{
		    corr_est.setInputCloud(Source_Ground);
            corr_est.setInputTarget(Target_Ground);
	        corr_est.determineCorrespondences(*corrs_Ground);
            corr_rej_dist.setInputCorrespondences(corrs_Ground);
	        corr_rej_dist.setMaximumDistance(dis_thre_ground); 
		    corr_rej_dist.getCorrespondences(*corrs_Ground_f);
		}

		//For Edge points
		if (Source_Edge->points.size()>=K_min && Target_Edge->points.size()>=K_min)
		{
		    corr_est.setInputCloud(Source_Edge);
            corr_est.setInputTarget(Target_Edge);
	        corr_est.determineCorrespondences(*corrs_Edge);
            corr_rej_dist.setInputCorrespondences(corrs_Edge);
	        corr_rej_dist.setMaximumDistance(dis_thre_edge); 
		    corr_rej_dist.getCorrespondences(*corrs_Edge_f);
		}

		//For Planar points
		if (Source_Planar->points.size()>=K_min && Target_Planar->points.size()>=K_min)
		{
            corr_est.setInputCloud(Source_Planar);
            corr_est.setInputTarget(Target_Planar);
	        corr_est.determineCorrespondences(*corrs_Planar);
            corr_rej_dist.setInputCorrespondences(corrs_Planar);
	        corr_rej_dist.setMaximumDistance(dis_thre_planar); 
		    corr_rej_dist.getCorrespondences(*corrs_Planar_f);
		}
		
		//For Sphere points
		if (Source_Sphere->points.size()>=K_min && Target_Sphere->points.size()>=K_min)
		{
            corr_est.setInputCloud(Source_Sphere);
            corr_est.setInputTarget(Target_Sphere);
	        corr_est.determineCorrespondences(*corrs_Sphere);
            corr_rej_dist.setInputCorrespondences(corrs_Sphere);
	        corr_rej_dist.setMaximumDistance(dis_thre_sphere); 
		    corr_rej_dist.getCorrespondences(*corrs_Sphere_f);
		}
		
		dis_thre_ground /= 1.1;
        dis_thre_edge /= 1.1;
		dis_thre_planar /= 1.1;
		dis_thre_sphere /= 1.1;
		
        //if(i==max_iter_num-1) 
		printf("Found correspondences # [G:%d E:%d P:%d S:%d] \nAfter filtering # [G:%d E:%d P:%d S:%d] \n",
		 (*corrs_Ground).size(),(*corrs_Edge).size(), (*corrs_Planar).size(), (*corrs_Sphere).size(), 
		 (*corrs_Ground_f).size(), (*corrs_Edge_f).size(), (*corrs_Planar_f).size(), (*corrs_Sphere_f).size());

		//Estimate Transformation
	    Eigen::Matrix4f TempTran;
		Multi_metrics_lls_estimation(Source_Ground, Target_Ground, corrs_Ground_f,
		                             Source_Edge,   Target_Edge,   corrs_Edge_f,
		                             Source_Planar, Target_Planar, corrs_Planar_f,
		                             Source_Sphere, Target_Sphere, corrs_Sphere_f,
		                             TempTran, information_matrix);
		
		cout<<"Iter: # "<< i<<endl << TempTran <<endl;

		

		//Update the Source pointcloud
        pcl::transformPointCloudWithNormals(*Source_Ground,*Source_Ground,TempTran);
		pcl::transformPointCloudWithNormals(*Source_Edge  ,*Source_Edge  ,TempTran);
		pcl::transformPointCloudWithNormals(*Source_Planar,*Source_Planar,TempTran);
		pcl::transformPointCloudWithNormals(*Source_Sphere,*Source_Sphere,TempTran);

		//pcl::transformPointCloud(*SourceNormal, *SourceNormal, TempTran);
		
		transformationS2T=TempTran*transformationS2T;

	}

	t2=clock();
	
	printf("SCloud point [G:%d E:%d P:%d S:%d] TCloud point [G:%d E:%d P:%d S:%d]\n",Source_Ground->points.size(), Source_Edge->points.size(), Source_Planar->points.size(), Source_Sphere->points.size(),
	 Target_Ground->points.size(), Target_Edge->points.size(), Target_Planar->points.size(), Target_Sphere->points.size());
	//cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "<< TargetCloud->points.size()<< endl;
	cout << "Registration done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
	//cout << "Normal Estimation in " << float(t1 - t0) / CLOCKS_PER_SEC << " s, "<< "Alignment in " << float(t2 - t1) / CLOCKS_PER_SEC<< " s."<<endl;
	cout << transformationS2T << endl;
    
	return 1;
}


template<typename PointT>
bool CRegistration<PointT>::Multi_metrics_lls_estimation(  const typename pcl::PointCloud<PointT>::Ptr & Source_Ground, const typename pcl::PointCloud<PointT>::Ptr & Target_Ground, boost::shared_ptr<pcl::Correspondences> &Corr_Ground,
                                                           const typename pcl::PointCloud<PointT>::Ptr & Source_Edge,   const typename pcl::PointCloud<PointT>::Ptr & Target_Edge  , boost::shared_ptr<pcl::Correspondences> &Corr_Edge  ,
													       const typename pcl::PointCloud<PointT>::Ptr & Source_Planar, const typename pcl::PointCloud<PointT>::Ptr & Target_Planar, boost::shared_ptr<pcl::Correspondences> &Corr_Planar,
                                                           const typename pcl::PointCloud<PointT>::Ptr & Source_Sphere, const typename pcl::PointCloud<PointT>::Ptr & Target_Sphere, boost::shared_ptr<pcl::Correspondences> &Corr_Sphere,                              
	                                                       Eigen::Matrix4f & transformationS2T, Eigen::Matrix<float, 6, 6> & information_matrix)
{
	 typedef Eigen::Matrix<float, 6, 1> Vector6f;
     typedef Eigen::Matrix<float, 6, 6> Matrix6f;

     Matrix6f ATA;
     Vector6f ATb;
     ATA.setZero ();
     ATb.setZero ();
     
     float w_ground,w_planar,w_edge_x,w_edge_y,w_edge_z,w_sphere;
	 w_ground=3.0;
	 w_planar=2.4;
	//  w_edge_x=1.0;
	//  w_edge_y=1.0;
	//  w_edge_z=0.5;
	 w_sphere=0.25;
     
	 //Ground to Ground (Plane to Plane)
	 for (int i=0; i<(*Corr_Ground).size();i++)
	 {
        int s_index,t_index;
		s_index=(*Corr_Ground)[i].index_query;
		t_index=(*Corr_Ground)[i].index_match;
		
		if (t_index!=-1)
		{
			float px = Source_Ground->points[s_index].x;
			float py = Source_Ground->points[s_index].y;
			float pz = Source_Ground->points[s_index].z;
			float qx = Target_Ground->points[t_index].x;
			float qy = Target_Ground->points[t_index].y;
			float qz = Target_Ground->points[t_index].z;
			float ntx = Target_Ground->points[t_index].normal[0];
			float nty = Target_Ground->points[t_index].normal[1];
			float ntz = Target_Ground->points[t_index].normal[2];
			float w = w_ground;

            float a = ntz*py - nty*pz;
            float b = ntx*pz - ntz*px;
            float c = nty*px - ntx*py;
            
			 //    0  1  2  3  4  5
             //    6  7  8  9 10 11
             //   12 13 14 15 16 17
             //   18 19 20 21 22 23
             //   24 25 26 27 28 29
             //   30 31 32 33 34 35

			 ATA.coeffRef (0) += w * ntx * ntx;
             ATA.coeffRef (1) += w * ntx * nty;
             ATA.coeffRef (2) += w * ntx * ntz;
             ATA.coeffRef (3) += w * a * ntx;
             ATA.coeffRef (4) += w * b * ntx;
             ATA.coeffRef (5) += w * c * ntx;
             ATA.coeffRef (7) += w * nty * nty;
             ATA.coeffRef (8) += w * nty * ntz;
             ATA.coeffRef (9) += w * a * nty;
             ATA.coeffRef (10) += w * b * nty;
             ATA.coeffRef (11) += w * c * nty;
             ATA.coeffRef (14) += w * ntz * ntz;
             ATA.coeffRef (15) += w * a * ntz;
             ATA.coeffRef (16) += w * b * ntz;
             ATA.coeffRef (17) += w * c * ntz;
             ATA.coeffRef (21) += w * a * a;
             ATA.coeffRef (22) += w * a * b;
             ATA.coeffRef (23) += w * a * c;
             ATA.coeffRef (28) += w * b * b;
             ATA.coeffRef (29) += w * b * c;
             ATA.coeffRef (35) += w * c * c;

             float d = ntx*qx + nty*qy + ntz*qz - ntx*px - nty*py - ntz*pz;

             ATb.coeffRef (0) += w * d * ntx;
             ATb.coeffRef (1) += w * d * nty;
             ATb.coeffRef (2) += w * d * ntz;
             ATb.coeffRef (3) += w * d * a;
             ATb.coeffRef (4) += w * d * b;
             ATb.coeffRef (5) += w * d * c;
        
		}
	 }
     

     //Planar to Planar (Plane to Plane)
	 for (int i=0; i<(*Corr_Planar).size();i++)
	 {
        int s_index,t_index;
		s_index=(*Corr_Planar)[i].index_query;
		t_index=(*Corr_Planar)[i].index_match;
		
		if (t_index!=-1)
		{
			float px = Source_Planar->points[s_index].x;
			float py = Source_Planar->points[s_index].y;
			float pz = Source_Planar->points[s_index].z;
			float qx = Target_Planar->points[t_index].x;
			float qy = Target_Planar->points[t_index].y;
			float qz = Target_Planar->points[t_index].z;
			float ntx = Target_Planar->points[t_index].normal[0];
			float nty = Target_Planar->points[t_index].normal[1];
			float ntz = Target_Planar->points[t_index].normal[2];
			float w = w_planar;

            float a = ntz*py - nty*pz;
            float b = ntx*pz - ntz*px;
            float c = nty*px - ntx*py;
            
			 //    0  1  2  3  4  5
             //    6  7  8  9 10 11
             //   12 13 14 15 16 17
             //   18 19 20 21 22 23
             //   24 25 26 27 28 29
             //   30 31 32 33 34 35

			 ATA.coeffRef (0) += w * ntx * ntx;
             ATA.coeffRef (1) += w * ntx * nty;
             ATA.coeffRef (2) += w * ntx * ntz;
             ATA.coeffRef (3) += w * a * ntx;
             ATA.coeffRef (4) += w * b * ntx;
             ATA.coeffRef (5) += w * c * ntx;
             ATA.coeffRef (7) += w * nty * nty;
             ATA.coeffRef (8) += w * nty * ntz;
             ATA.coeffRef (9) += w * a * nty;
             ATA.coeffRef (10) += w * b * nty;
             ATA.coeffRef (11) += w * c * nty;
             ATA.coeffRef (14) += w * ntz * ntz;
             ATA.coeffRef (15) += w * a * ntz;
             ATA.coeffRef (16) += w * b * ntz;
             ATA.coeffRef (17) += w * c * ntz;
             ATA.coeffRef (21) += w * a * a;
             ATA.coeffRef (22) += w * a * b;
             ATA.coeffRef (23) += w * a * c;
             ATA.coeffRef (28) += w * b * b;
             ATA.coeffRef (29) += w * b * c;
             ATA.coeffRef (35) += w * c * c;

             float d = ntx*qx + nty*qy + ntz*qz - ntx*px - nty*py - ntz*pz;

             ATb.coeffRef (0) += w * d * ntx;
             ATb.coeffRef (1) += w * d * nty;
             ATb.coeffRef (2) += w * d * ntz;
             ATb.coeffRef (3) += w * d * a;
             ATb.coeffRef (4) += w * d * b;
             ATb.coeffRef (5) += w * d * c;
        
		}
	 }

	 //Edge to Edge (Line to Line)
	 for (int i=0; i<(*Corr_Edge).size();i++)
	 {
        int s_index,t_index;
		s_index=(*Corr_Edge)[i].index_query;
		t_index=(*Corr_Edge)[i].index_match;
		 

		if (t_index!=-1)
		{
			float px = Source_Edge->points[s_index].x;
			float py = Source_Edge->points[s_index].y;
			float pz = Source_Edge->points[s_index].z;
			float qx = Target_Edge->points[t_index].x;
			float qy = Target_Edge->points[t_index].y;
			float qz = Target_Edge->points[t_index].z;
			float ntx = Target_Edge->points[t_index].normal[0];
			float nty = Target_Edge->points[t_index].normal[1];
			float ntz = Target_Edge->points[t_index].normal[2];
			float nsx = Source_Edge->points[s_index].normal[0];
			float nsy = Source_Edge->points[s_index].normal[1];
			float nsz = Source_Edge->points[s_index].normal[2];
			
            float wx,wy,wz;
			
            
            float dx=px-qx;
			float dy=py-qy;
			float dz=pz-qz;

			float nx=ntx*nsz-ntz*nsy;
			float ny=ntz*nsx-ntx*nsz;
			float nz=ntx*nsy-nty*nsx;
			float nd=sqrt(nx*nx+ny*ny+nz*nz);
			nx/=nd;
			ny/=nd;
			nz/=nd;
  
            if (nz>0.7)
			{
				wx=1.0;
				wy=1.0;
				wz=0.25;
			} 
			else if (nz<0.3)
			{
				wx=0.5;
				wy=0.5;
				wz=0.75;
			}
			else
			{
				wx=0.1;
				wy=0.1;
				wz=0.1;
			}
			
            double nxy=nx*ny;
			double nxz=nx*nz;
			double nyz=ny*nz;
			double nx2=nx*nx;
			double ny2=ny*ny;
			double nz2=nz*nz;
            double px2=px*px;
			double py2=py*py;
			double pz2=pz*pz;
			double pxy=px*py;
			double pxz=px*pz;
			double pyz=py*pz;
             
			 //    0  1  2  3  4  5
             //    6  7  8  9 10 11
             //   12 13 14 15 16 17
             //   18 19 20 21 22 23
             //   24 25 26 27 28 29
             //   30 31 32 33 34 35

             ATA.coeffRef (0) += (wy*nz2+wz*ny2);
			 ATA.coeffRef (1) += (-wz*nxy);
			 ATA.coeffRef (2) += (-wy*nxz);
			 ATA.coeffRef (3) += (-wy*nxz*py+wz*nxy*pz);
			 ATA.coeffRef (4) += (wy*nxz*px+wy*nz2*pz+wz*ny2*pz);
			 ATA.coeffRef (5) += (-wy*nz2*py-wz*ny2*py+wz*nxy*px);
			 ATA.coeffRef (7) += (wx*nz2+wz*nx2);
			 ATA.coeffRef (8) += (-wx*nyz);
			 ATA.coeffRef (9) += (-wx*nz2*pz-wx*nyz*py-wz*nz2*pz);
			 ATA.coeffRef (10) += (wx*nyz*px-wz*nxy*pz);
			 ATA.coeffRef (11) += (wx*nz2*px+wz*nxy*py+wz*nx2*px);
			 ATA.coeffRef (14) += (wx*ny2+wy*nx2);
			 ATA.coeffRef (15) += (wx*nyz*pz+wx*ny2*py+wy*nx2*py);
			 ATA.coeffRef (16) += (-wx*ny2*px-wy*nx2*px-wy*nxz*pz);
			 ATA.coeffRef (17) += (-wx*nyz*px+wy*nxz*py);
			 ATA.coeffRef (21) += (wx*(nz2*pz2+ny2*py2+2*nyz*pyz)+wy*nx2*py2+wz*nx2*pz2);
			 ATA.coeffRef (22) += (-wx*(nyz*pxz+ny2*pxy)-wy*(nx2*pxy+nxz*pyz)+wz*nxy*pz2);
			 ATA.coeffRef (23) += (-wx*(nz2*pxz+nyz*pxy)+wy*nxz*py2-wz*(nxy*pyz+nx2*pxz));
			 ATA.coeffRef (28) += (wx*ny2*px2+wy*(nx2*px2+nz2*pz2+2*nxz*pxz)+wz*ny2*pz2);
			 ATA.coeffRef (29) += (wx*nyz*px2-wy*(nxz*pxy+nz2*pyz)-wz*(ny2*pyz+nxy*pxz)); 
			 ATA.coeffRef (35) += (wx*nz2*px2+wy*nz2*py2+wz*(ny2*py2+nx2*px2+2*nxy*pxy));

			 ATb.coeffRef (0) += (wy*(nxz*dz-nz2*dx)+wz*(-ny2*dx+nxy*dy));
			 ATb.coeffRef (1) += (wx*(-nz2*dy+nyz*dz)+wz*(nxy*dx-nx2*dy));
			 ATb.coeffRef (2) += (wx*(nyz*dy-ny2*dz)+wy*(-nx2*dz+nxz*dx));
			 ATb.coeffRef (3) += (wx*(nz*pz*ny*py)*(nz*dy-ny*dz)+wy*nx*py*(-nx*dz+nz*dx)+wz*nx*pz*(-ny*dx+nx*dy));
			 ATb.coeffRef (4) += (wx*ny*px*(-nz*dy+ny*dz)+wy*(nx*px+nz*pz)*(nx*dz-nz*dx)+wz*ny*pz*(-ny*dx+nx*dy));                                  
             ATb.coeffRef (5) += (wx*nz*px*(-nz*dy+ny*dz)+wy*nz*py*(-nx*dz+nz*dx)+wz*(ny*py+nx*px)*(ny*dx-nx*dy));
        
		}
	 }
            
	
     //Sphere to Sphere (Point to Point)
	 for (int i=0; i<(*Corr_Sphere).size();i++)
	 {
        int s_index,t_index;
		s_index=(*Corr_Sphere)[i].index_query;
		t_index=(*Corr_Sphere)[i].index_match;
		
		if (t_index!=-1)
		{
			float px = Source_Sphere->points[s_index].x;
			float py = Source_Sphere->points[s_index].y;
			float pz = Source_Sphere->points[s_index].z;
			float qx = Target_Sphere->points[t_index].x;
			float qy = Target_Sphere->points[t_index].y;
			float qz = Target_Sphere->points[t_index].z;
			float wx = w_sphere;
			float wy = w_sphere;
			float wz = w_sphere;

            float dx=px-qx;
			float dy=py-qy;
			float dz=pz-qz;
            
			 //    0  1  2  3  4  5
             //    6  7  8  9 10 11
             //   12 13 14 15 16 17
             //   18 19 20 21 22 23
             //   24 25 26 27 28 29
             //   30 31 32 33 34 35

            ATA.coeffRef (0) += wx;
            ATA.coeffRef (1) += 0;
            ATA.coeffRef (2) += 0;
            ATA.coeffRef (3) += 0;
            ATA.coeffRef (4) += wx * pz;
            ATA.coeffRef (5) += (-wx * py);
            ATA.coeffRef (7) += wy;
            ATA.coeffRef (8) += 0;
            ATA.coeffRef (9) += (-wy * pz);
            ATA.coeffRef (10) += 0;
            ATA.coeffRef (11) += wy * px;
			ATA.coeffRef (14) += wz;
            ATA.coeffRef (15) += wz * py;
            ATA.coeffRef (16) += (-wz * px);
            ATA.coeffRef (17) += 0;
            ATA.coeffRef (21) += wy* pz * pz + wz* py * py;
            ATA.coeffRef (22) += (-wz * px * py);
            ATA.coeffRef (23) += (-wy * px * pz);
            ATA.coeffRef (28) += wx* pz * pz + wz* px * px;
            ATA.coeffRef (29) += (-wx * py * pz); 
            ATA.coeffRef (35) += wx* py * py + wy* px * px; 

            ATb.coeffRef (0) += (-wx * dx);
            ATb.coeffRef (1) += (-wy * dy);
            ATb.coeffRef (2) += (-wz * dz);
            ATb.coeffRef (3) += wy * pz * dy - wz * py * dz;
            ATb.coeffRef (4) += wz * px * dz - wx * pz * dx;
            ATb.coeffRef (5) += wx * py * dx - wy * px * dy;
        
		}
	 }

	 ATA.coeffRef (6) = ATA.coeff (1);
     ATA.coeffRef (12) = ATA.coeff (2);
     ATA.coeffRef (13) = ATA.coeff (8);
     ATA.coeffRef (18) = ATA.coeff (3);
     ATA.coeffRef (19) = ATA.coeff (9);
     ATA.coeffRef (20) = ATA.coeff (15);
     ATA.coeffRef (24) = ATA.coeff (4);
     ATA.coeffRef (25) = ATA.coeff (10);
     ATA.coeffRef (26) = ATA.coeff (16);
     ATA.coeffRef (27) = ATA.coeff (22);
     ATA.coeffRef (30) = ATA.coeff (5);
     ATA.coeffRef (31) = ATA.coeff (11);
     ATA.coeffRef (32) = ATA.coeff (17);
     ATA.coeffRef (33) = ATA.coeff (23);
     ATA.coeffRef (34) = ATA.coeff (29);
 
     // Solve A*x = b  x= (ATA)^(-1)ATb  
	 // x: tx ty tz alpha beta gamma
     Vector6f x = static_cast<Vector6f> (ATA.inverse () * ATb);
     
	 information_matrix= ATA;
     
	 // Construct the transformation matrix from x
     constructTransformation(x (0), x (1), x (2), x (3), x (4), x (5), transformationS2T );
     
	 return 1;
}



template<typename PointT>
void CRegistration<PointT>::constructTransformation(const float & tx, const float & ty, const float & tz,
                                                    const float & alpha, const float & beta, const float & gamma,
                                                    Eigen::Matrix4f &transformation_matrix) 
{
  // Construct the transformation matrix from rotation and translation 
  transformation_matrix = Eigen::Matrix<float, 4, 4>::Zero ();
  // From euler angle to rotation matrix 
  transformation_matrix (0, 0) = static_cast<float> ( cos (gamma) * cos (beta));
  transformation_matrix (0, 1) = static_cast<float> (-sin (gamma) * cos (alpha) + cos (gamma) * sin (beta) * sin (alpha));
  transformation_matrix (0, 2) = static_cast<float> ( sin (gamma) * sin (alpha) + cos (gamma) * sin (beta) * cos (alpha));
  transformation_matrix (1, 0) = static_cast<float> ( sin (gamma) * cos (beta));
  transformation_matrix (1, 1) = static_cast<float> ( cos (gamma) * cos (alpha) + sin (gamma) * sin (beta) * sin (alpha));
  transformation_matrix (1, 2) = static_cast<float> (-cos (gamma) * sin (alpha) + sin (gamma) * sin (beta) * cos (alpha));
  transformation_matrix (2, 0) = static_cast<float> (-sin (beta));
  transformation_matrix (2, 1) = static_cast<float> ( cos (beta) * sin (alpha));
  transformation_matrix (2, 2) = static_cast<float> ( cos (beta) * cos (alpha));

  transformation_matrix (0, 3) = static_cast<float> (tx);
  transformation_matrix (1, 3) = static_cast<float> (ty);
  transformation_matrix (2, 3) = static_cast<float> (tz);
  transformation_matrix (3, 3) = static_cast<float> (1);
}



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
double CRegistration<PointT>::icp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	int max_iter,
	bool use_reciprocal_correspondence,
	bool use_trimmed_rejector,
	float thre_dis,
	transform_estimator_type te)
{
	clock_t t0, t1;
	t0=clock();
	
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	
	// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
	
	// Trimmed or not? [ Use a predefined overlap ratio to trim part of the correspondence with bigger distance ]
	if (use_trimmed_rejector){
		pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorTrimmed);
		float overlap_ratio = calOverlap(SourceCloud, TargetCloud, thre_dis);
		if (overlap_ratio < min_overlap_for_reg_) {
			LOG(WARNING) << "The overlap ratio is too small. This registration would not be done.";
			return -1;
		}
		else{
			trimmed_cr->setOverlapRatio(overlap_ratio);
			icp.addCorrespondenceRejector(trimmed_cr);
		}
	}
	else {icp.setMaxCorrespondenceDistance(thre_dis);}

	icp.setInputSource(SourceCloud);
	icp.setInputTarget(TargetCloud);
    
    typename pcl::registration::TransformationEstimationSVD<PointT,PointT,float>::Ptr te_svd(new pcl::registration::TransformationEstimationSVD<PointT,PointT,float>);

	typename pcl::registration::TransformationEstimationLM<PointT,PointT,float>::Ptr te_lm(new pcl::registration::TransformationEstimationLM<PointT,PointT,float>);
	
	switch (te)
	{
	case SVD:
	    icp.setTransformationEstimation(te_svd); //Use SVD 
		break;
	case LM:
	    icp.setTransformationEstimation(te_lm); //Use L-M Non-Linear Optimization
	    break;
	default:  //Default svd
		break;
	}
	
	//icp.setMaxCorrespondenceDistance(thre_dis);

	// Converge criterion ( 'Or' Relation ) 
	// Set the maximum number of iterations [ n>x ] (criterion 1)
	icp.setMaximumIterations(max_iter);     //Most likely to happen
	// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	icp.setTransformationEpsilon(1e-7);     //Quite hard to happen
	// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	icp.setEuclideanFitnessEpsilon(1e-4);   //Quite hard to happen
	
	icp.align(*TransformedSource);  //Use closed-form SVD to estimate transformation for each iteration [You can switch to L-M Optimization]
	transformationS2T = icp.getFinalTransformation();
	
	t1=clock();
    
	double mae_nn=icp.getFitnessScore(thre_dis);  //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

	// Commented these out if you don't want to output the registration log
	LOG(INFO) << "Point-to-Point ICP done in  " << float(t1 - t0) / CLOCKS_PER_SEC << "s";
	LOG(INFO) << transformationS2T;
	LOG(INFO) << "The fitness score of this registration is " << mae_nn;
	if (mae_nn > 1000) LOG(WARNING) << "The fitness score of this registration is a bit too large";
	LOG(INFO) << "-----------------------------------------------------------------------------";

	//
	cout << "Point-to-Point ICP done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
	cout << "The fitness score of this registration is " <<mae_nn <<endl;
	cout << "-----------------------------------------------------------------------------" << endl;
     
	return mae_nn;
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
double CRegistration<PointT>::ptplicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	int max_iter,
	bool use_reciprocal_correspondence,
	bool use_trimmed_rejector,
	float thre_dis,
	float neighbor_radius,
	transform_estimator_type te)
{
	clock_t t0, t1, t2;
	t0=clock();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr SourceCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr TargetCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	copyPointCloud(*SourceCloud, *SourceCloudXYZ);
	copyPointCloud(*TargetCloud, *TargetCloudXYZ);
	// In this case, The Cloud's Normal hasn't been calculated yet. 
    
	pcl::PointCloud<pcl::PointNormal>::Ptr SourceNormal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr TargetNormal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr TransformedSourceN(new pcl::PointCloud<pcl::PointNormal>());
	
	//Estimate Normal Multi-thread
	PrincipleComponentAnalysis<PointT> pca_estimator; 
	
	//Radius search
	pca_estimator.CalculatePointCloudWithNormal_Radius(SourceCloudXYZ, neighbor_radius, SourceNormal); 
	pca_estimator.CalculatePointCloudWithNormal_Radius(TargetCloudXYZ, neighbor_radius, TargetNormal);
	
	//KNN search
	//pca_estimator.CalculatePointCloudWithNormal_KNN(SourceCloud, covariance_K, SourceNormal);
	//pca_estimator.CalculatePointCloudWithNormal_KNN(TargetCloud, covariance_K, TargetNormal);

	t1=clock();

	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> ptplicp;
	
	ptplicp.setInputSource(SourceNormal);
	ptplicp.setInputTarget(TargetNormal);

	// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	ptplicp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
     
	// Trimmed or not? [ Use a predefined overlap ratio to trim part of the correspondence with bigger distance ]
	pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorVarTrimmed);
	if (use_trimmed_rejector) ptplicp.addCorrespondenceRejector(trimmed_cr);
	else {ptplicp.setMaxCorrespondenceDistance(thre_dis);}

    typename pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal,pcl::PointNormal,float>::Ptr te_lm(new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal,pcl::PointNormal,float>);
    typename pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal,pcl::PointNormal,float>::Ptr te_lls(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal,pcl::PointNormal,float>);

    switch (te)
	{
	case LLS:
	    ptplicp.setTransformationEstimation(te_lls); //Use Linear Least Square
		break;
	case LM:
	    ptplicp.setTransformationEstimation(te_lm); //Use L-M Non-Linear Optimization
	    break;
	default:  //Default lls
		break;
	}

	// Converge criterion ( 'Or' Relation ) 
	// Set the maximum number of iterations [ n>x ] (criterion 1)
	ptplicp.setMaximumIterations(max_iter);     //Most likely to happen
	// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	ptplicp.setTransformationEpsilon(1e-7);     //Quite hard to happen
	// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	ptplicp.setEuclideanFitnessEpsilon(1e-4);   //Quite hard to happen
	
	ptplicp.align(*TransformedSourceN);  //Use Linear Least Square Method to estimate transformation for each iteration
    
	if (use_trimmed_rejector) printf("Estimated trimmed distance threshold is %lf.\n", trimmed_cr->getTrimmedDistance());
	
	transformationS2T = ptplicp.getFinalTransformation();

	t2=clock();
    
	double mae_nn=ptplicp.getFitnessScore(thre_dis);  //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

	// Commented these out if you don't want to output the registration log
	LOG(INFO) << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "<< TargetCloud->points.size();
	LOG(INFO) << "Point-to-Plane ICP done in  " << float(t1 - t0) / CLOCKS_PER_SEC << "s";
	// LOG(INFO) << "Transform Matrix" << endl << transformationS2T;
	LOG(INFO) << "The fitness score of this registration is " << mae_nn;
	if (mae_nn > 1000) LOG(WARNING) << "The fitness score of this registration is a bit too large";
	// LOG(INFO) << "-----------------------------------------------------------------------------";

	// Commented these out if you don't want to output the registration log
	cout << "Point-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl
	<< "Normal Estimation in " << float(t1 - t0) / CLOCKS_PER_SEC << " s, " << "registration in " << float(t2 - t1) / CLOCKS_PER_SEC << " s."<<endl;
	cout << "The fitness score of this registration is " << mae_nn << endl << transformationS2T << endl;
	//cout << "-----------------------------------------------------------------------------" << endl;
    
	copyPointCloud(*TransformedSourceN,*TransformedSource);

    pcl::PointCloud<pcl::PointNormal>().swap(*TransformedSourceN);    // Free the Memory
	pcl::PointCloud<pcl::PointNormal>().swap(*SourceNormal);          // Free the Memory
	pcl::PointCloud<pcl::PointNormal>().swap(*TargetNormal);          // Free the Memory
	pcl::PointCloud<pcl::PointXYZ>().swap(*SourceCloudXYZ);           // Free the Memory
	pcl::PointCloud<pcl::PointXYZ>().swap(*TargetCloudXYZ);           // Free the Memory

	return mae_nn;
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
double CRegistration<PointT>::gicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	int max_iter,
	bool use_reciprocal_correspondence,
	bool use_trimmed_rejector,
	float thre_dis,
	int covariance_K)
{	
	clock_t t0, t1;
	t0=clock();
	
	pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
	
	// Set the number of points used to calculated the covariance of a point
	gicp.setCorrespondenceRandomness(covariance_K);
	
	// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	gicp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

	// Trimmed or not? [ Use a predefined overlap ratio to trim part of the correspondence with bigger distance ]
	if (use_trimmed_rejector){
		pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorTrimmed);
		float overlap_ratio = calOverlap(SourceCloud, TargetCloud, thre_dis);
		if (overlap_ratio < min_overlap_for_reg_) {
			LOG(WARNING) << "The overlap ratio is too small. This registration would not be done.";
			return -1;
		}
		else{
			trimmed_cr->setOverlapRatio(overlap_ratio);
			gicp.addCorrespondenceRejector(trimmed_cr);
		}
	}
	else {gicp.setMaxCorrespondenceDistance(thre_dis);}
	
	//gicp.setMaxCorrespondenceDistance(1e6); //A large value

	gicp.setInputSource(SourceCloud);
	gicp.setInputTarget(TargetCloud);

	// Converge criterion ( 'Or' Relation ) 
	// According to gicp.hpp, the iteration terminates when one of the three happens
	// Set the maximum number of iterations  (criterion 1)
	gicp.setMaximumIterations(max_iter);     //Most likely to happen
	// Set the transformation difference threshold (criterion 2)
	gicp.setTransformationEpsilon(1e-7);     //Hard to happen
	// Set the rotation difference threshold (criterion 3) 
	gicp.setRotationEpsilon(1e-4);           //Hard to happen

	gicp.align(*TransformedSource);

	transformationS2T = gicp.getFinalTransformation();

	t1=clock();
    
	double mae_nn=gicp.getFitnessScore(thre_dis);  //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

	// Commented these out if you don't want to output the registration log
	LOG(INFO) << "GICP done in  " << float(t1 - t0) / CLOCKS_PER_SEC << "s";
	LOG(INFO) << transformationS2T;
	LOG(INFO) << "The fitness score of this registration is " << mae_nn;
	if (mae_nn > 1000) LOG(WARNING) << "The fitness score of this registration is a bit too large";
	LOG(INFO) << "-----------------------------------------------------------------------------";

	// Commented these out if you don't want to output the registration log
	cout << "GICP done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
	cout << "The fitness score of this registration is " << mae_nn << endl;
	cout << "-----------------------------------------------------------------------------" << endl;

	return mae_nn;
}


/**
* \brief Estimated the approximate overlapping ratio for Cloud1 considering Cloud2
* \param[in]  Cloud1 : A pointer of the Point Cloud used for overlap ratio calculation
* \param[in]  Cloud2 : A pointer of the Point Cloud overlapped with Cloud1
* \param[out] thre_dis : It acts as the search radius of overlapping estimation
* \return : The estimated overlap ratio [from 0 to 1]
*/
template<typename PointT>
float CRegistration<PointT>::calOverlap(const typename pcl::PointCloud<PointT>::Ptr & Cloud1,
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
	//cout << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio << endl;
	LOG(INFO) << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio;

	return overlap_ratio;
}

/**
* \brief Transform a Point Cloud using a given transformation matrix
* \param[in]  Cloud : A pointer of the Point Cloud before transformation
* \param[out] TransformedCloud : A pointer of the Point Cloud after transformation
* \param[in]  transformation : A 4*4 transformation matrix
*/
template<typename PointT>
void CRegistration<PointT>::transformcloud(typename pcl::PointCloud<PointT>::Ptr & Cloud,
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
		
		//Comment it if there's no intensity
		pt.intensity =  Cloud->points[i].intensity;
		TransformedCloud->points.push_back(pt);
	}
	//cout << "Transform done ..." << endl;
}

/**
* \brief Get the Inverse (Not Mathimatically) of a giving 4*4 Transformation Matrix
* \param[in]  transformation : A 4*4 transformation matrix ( from Cloud A to Cloud A' )
* \param[out] invtransformation : Inversed 4*4 transformation matrix ( from Cloud A' to Cloud A )
*/
template<typename PointT>
void CRegistration<PointT>::invTransform(const Eigen::Matrix4f & transformation,
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

template<typename PointT>
void CRegistration<PointT>::compute_fpfh_feature(const typename pcl::PointCloud<PointT>::Ptr &input_cloud, fpfhFeaturePtr &cloud_fpfh, float search_radius){
	
	// Calculate the Point Normal
	NormalsPtr cloud_normal(new Normals);
	calNormal(input_cloud, cloud_normal, search_radius);

	// Estimate FPFH Feature
	pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4);
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(cloud_normal);
	typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	est_fpfh.setSearchMethod(tree);
	//est_fpfh.setKSearch(20);
	est_fpfh.setRadiusSearch(2.0*search_radius);
	est_fpfh.compute(*cloud_fpfh);
}

#if 0
template<typename PointT>
void CRegistration<PointT>::Coarsereg_FPFHSAC(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	float search_radius)
{
	clock_t t0, t1;
	t0 = clock();

	fpfhFeaturePtr source_fpfh(new fpfhFeature());
	fpfhFeaturePtr target_fpfh(new fpfhFeature());

	compute_fpfh_feature(SourceCloud, source_fpfh, search_radius);
	compute_fpfh_feature(TargetCloud, target_fpfh, search_radius);

	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(SourceCloud);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(TargetCloud);
	sac_ia.setTargetFeatures(target_fpfh);
	//sac_ia.setNumberOfSamples(20);       //����ÿ�ε���������ʹ�õ�������������ʡ��,�ɽ�ʡʱ��
	sac_ia.setCorrespondenceRandomness(15); //���ü���Э����ʱѡ����ٽ��ڵ㣬��ֵԽ��Э����Խ��ȷ�����Ǽ���Ч��Խ��.(��ʡ)
	sac_ia.align(*TransformedSource);
	transformationS2T = sac_ia.getFinalTransformation();
	
	t1 = clock();
	// Commented these out if you don't want to output the registration log
	cout << "FPFH-SAC done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
	cout << "The fitness score of this registration is " << sac_ia.getFitnessScore() << endl;
	cout << "-----------------------------------------------------------------------------" << endl;
}
#endif

template<typename PointT>
void CRegistration<PointT>::Perturbation(const typename pcl::PointCloud<PointT>::Ptr & Cloud, typename pcl::PointCloud<PointT>::Ptr & CloudAfterPerturbation, float pertubate_value, std::vector<float> &pertubate_vector)
{
	pertubate_vector.resize(3);
	pertubate_vector[0] = 0.5 * pertubate_value - pertubate_value * ((double)rand() / RAND_MAX);
	pertubate_vector[1] = 0.5 * pertubate_value - pertubate_value * ((double)rand() / RAND_MAX);
	pertubate_vector[2] = 0.5 * pertubate_value - pertubate_value * ((double)rand() / RAND_MAX);
	
	for (size_t i = 0; i < Cloud->size(); i++)
	{
		PointT pt;
		pt.x = Cloud->points[i].x + pertubate_vector[0];
		pt.y = Cloud->points[i].y + pertubate_vector[1];
		pt.z = Cloud->points[i].z + pertubate_vector[2];
		CloudAfterPerturbation->push_back(pt);
	}
	LOG(INFO) << "The perturbation vector is:  X " << pertubate_vector[0] << " , Y " << pertubate_vector[1] << " , Z " << pertubate_vector[2];
}


template<typename PointT>
bool CRegistration<PointT>::CSTRAN_4DOF(const std::vector <std::vector<double> > & coordinatesA, const std::vector <std::vector<double> > & coordinatesB, std::vector<double> &transpara, int cp_number)
{
	double tx,ty,a,b;                   // 4 parameters 
	double s, rot_rad, rot_degree;
	double RMSE_check, sum_squaredist;
	int pointnumberA, pointnumberB, pointnumbercheck;
	Vector4d transAB;
	transpara.resize(5);

	pointnumberA = coordinatesA.size();
	pointnumberB = coordinatesB.size();

	sum_squaredist = 0;

	if (cp_number < 3)
	{
		cout << "Error ! Not enough control point number ..." << endl;
		return 0;
	}

	MatrixXd A_;
	VectorXd b_;

	A_.resize(cp_number * 2, 4);
	b_.resize(cp_number * 2, 1);

	for (int j = 0; j < cp_number; j++)
	{
		// A Matrix
		A_(j * 2, 0) = 1;
		A_(j * 2, 1) = 0;
		A_(j * 2, 2) = coordinatesA[j][0];
		A_(j * 2, 3) = -coordinatesA[j][1];

		A_(j * 2 + 1, 0) = 0;
		A_(j * 2 + 1, 1) = 1;
		A_(j * 2 + 1, 2) = coordinatesA[j][1];
		A_(j * 2 + 1, 3) = coordinatesA[j][0];

		//b Vector
		b_(j * 2, 0) = coordinatesB[j][0];
		b_(j * 2 + 1, 0) = coordinatesB[j][1];
	}
	transAB = ((A_.transpose()*A_).inverse())*A_.transpose()*b_;
		
	tx = transAB(0, 0);
	ty = transAB(1, 0);
	a = transAB(2, 0);
	b = transAB(3, 0);
	s = sqrt(a*a + b*b);
	
	transpara[0] = tx;
	transpara[1] = ty;
	transpara[2] = s;
	transpara[3] = b / s; //sin (ang)
	transpara[4] = a / s; //cos (ang)
	
	cout.setf(ios::showpoint);  //��С�����Ⱥ����0��ʾ����;
	cout.precision(12);         //����������ȣ�������Ч����;

	cout<< "Estimated Transformation From A to B" << endl
		<< "tx: " << tx << " m" << endl
		<< "ty: " << ty << " m" << endl
		<< "scale: " << s << endl;

	// Checking
	if (pointnumberA >= pointnumberB) pointnumbercheck = pointnumberB;
	else pointnumbercheck = pointnumberA;

	if (pointnumbercheck <= cp_number) cout << "Not enough points for check ..." << endl;
	else
	{
		pointnumbercheck -= cp_number;
		for (int j = 0; j < pointnumbercheck; j++)
		{
			double X_tran, Y_tran, squaredist;
			X_tran = transpara[2] * transpara[4] * coordinatesA[j + cp_number][0] - transpara[2] * transpara[3] * coordinatesA[j + cp_number][1] + transpara[0];
			Y_tran = transpara[2] * transpara[3] * coordinatesA[j + cp_number][0] + transpara[2] * transpara[4] * coordinatesA[j + cp_number][1] + transpara[1];
			squaredist = (X_tran - coordinatesB[j + cp_number][0])*(X_tran - coordinatesB[j + cp_number][0]) +
				(Y_tran - coordinatesB[j + cp_number][1])*(Y_tran - coordinatesB[j + cp_number][1]);
			sum_squaredist += squaredist;
		}

		RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

		cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
	}

	return 1;
}

template<typename PointT>
bool CRegistration<PointT>::CSTRAN_7DOF(const std::vector <std::vector<double> > & coordinatesA, const std::vector <std::vector<double> > & coordinatesB, std::vector<double> &transpara, int cp_number)
{
	double RMSE_check, sum_squaredist;
	int pointnumberA, pointnumberB, pointnumbercheck;
	VectorXd transAB;
	transAB.resize(7);
	transpara.resize(7);

	pointnumberA = coordinatesA.size();
	pointnumberB = coordinatesB.size();

	sum_squaredist = 0;

	if (cp_number < 4)
	{
		cout << "Error ! Not enough control point number ..." << endl;
		return 0;
	}

	MatrixXd A_;
	VectorXd b_;

	A_.resize(cp_number * 3, 7);
	b_.resize(cp_number * 3, 1);

	for (int j = 0; j < cp_number; j++)
	{
		// A Matrix   tx ty tz rx ry rz s
		A_(j * 3, 0) = 1;
		A_(j * 3, 1) = 0;
		A_(j * 3, 2) = 0;
		A_(j * 3, 3) = 0;
		A_(j * 3, 4) = -coordinatesA[j][2];
		A_(j * 3, 5) = coordinatesA[j][1];
		A_(j * 3, 6) = coordinatesA[j][0];

		A_(j * 3 + 1, 0) = 0;
		A_(j * 3 + 1, 1) = 1;
		A_(j * 3 + 1, 2) = 0;
		A_(j * 3 + 1, 3) = coordinatesA[j][2];
		A_(j * 3 + 1, 4) = 0;
		A_(j * 3 + 1, 5) = -coordinatesA[j][0];
		A_(j * 3 + 1, 6) = coordinatesA[j][1];

		A_(j * 3 + 2, 0) = 0;
		A_(j * 3 + 2, 1) = 0;
		A_(j * 3 + 2, 2) = 1;
		A_(j * 3 + 2, 3) = -coordinatesA[j][1];
		A_(j * 3 + 2, 4) = coordinatesA[j][0];
		A_(j * 3 + 2, 5) = 0;
		A_(j * 3 + 2, 6) = coordinatesA[j][2];

		//b Vector
		b_(j * 3, 0) = coordinatesB[j][0];
		b_(j * 3 + 1, 0) = coordinatesB[j][1];
		b_(j * 3 + 2, 0) = coordinatesB[j][2];
	}
	transAB = ((A_.transpose()*A_).inverse())*A_.transpose()*b_;

	transpara[0] = transAB(0);transpara[1] = transAB(1);transpara[2] = transAB(2);transpara[3] = transAB(3); transpara[4] = transAB(4); transpara[5] = transAB(5);transpara[6] = transAB(6);

	cout.setf(ios::showpoint);  //��С�����Ⱥ����0��ʾ����;
	cout.precision(10);         //����������ȣ�������Ч����;

	cout << "Estimated Transformation From A to B" << endl
		<< "tx: " << transpara[0] << " m" << endl
		<< "ty: " << transpara[1] << " m" << endl
		<< "tz: " << transpara[2] << " m" <<endl
		<< "rx: " << transpara[3] << endl
		<< "ry: " << transpara[4] << endl
		<< "rz: " << transpara[5] << endl
		<< "scale: " << transpara[6] << endl;

	// Checking
	if (pointnumberA >= pointnumberB) pointnumbercheck = pointnumberB;
	else pointnumbercheck = pointnumberA;

	if (pointnumbercheck <= cp_number) cout << "Not enough points for check ..." << endl;
	else
	{
		pointnumbercheck -= cp_number;
		for (int j = 0; j < pointnumbercheck; j++)
		{
			double X_tran, Y_tran, Z_tran, squaredist;
			X_tran = transpara[0] + transpara[6] * coordinatesA[j + cp_number][0] + transpara[5] * coordinatesA[j + cp_number][1] - transpara[4] * coordinatesA[j + cp_number][2];
			Y_tran = transpara[1] + transpara[6] * coordinatesA[j + cp_number][1] - transpara[5] * coordinatesA[j + cp_number][0] + transpara[3] * coordinatesA[j + cp_number][2];
			Z_tran = transpara[2] + transpara[6] * coordinatesA[j + cp_number][2] + transpara[4] * coordinatesA[j + cp_number][0] - transpara[3] * coordinatesA[j + cp_number][1];
			squaredist = (X_tran - coordinatesB[j + cp_number][0])*(X_tran - coordinatesB[j + cp_number][0]) +
				(Y_tran - coordinatesB[j + cp_number][1])*(Y_tran - coordinatesB[j + cp_number][1])+
				(Z_tran - coordinatesB[j + cp_number][2])*(Z_tran - coordinatesB[j + cp_number][2]);
			sum_squaredist += squaredist;
		}

		RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

		cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
	}

	return 1;
}


//Brief: Using the Gauss-Newton Least Square Method to solve 4 Degree of Freedom Transformation from no less than 2 points
//cp_number is the control points' number for LLS calculation, the rest of points are used to check the accuracy;
template<typename PointT>
bool CRegistration<PointT>::LLS_4DOF(const std::vector <std::vector<double> > & coordinatesA, const std::vector <std::vector<double> > & coordinatesB, Matrix4d & TransMatrixA2B, int cp_number, double theta0_degree)
{
	Vector4d transAB;
	Vector4d temp_trans;

	int iter_num = 0;

	double theta, theta0, dtheta, eps;  // in rad
	double yaw_degree;   // in degree
	double tx, ty, tz;                  // in meter
	double RMSE_check, sum_squaredist;
	int pointnumberA, pointnumberB, pointnumbercheck;
	//std::vector <std::vector<double>> coordinatesAT;

	//cout << "Input the approximate yaw angle in degree" << endl;
	//cin >> theta0_degree;

	dtheta = 9999;
	eps = 1e-9;

	theta0 = theta0_degree / 180 * M_PI; //Original Guess

	pointnumberA = coordinatesA.size();
	pointnumberB = coordinatesB.size();

	sum_squaredist = 0;

	if (cp_number < 2)
	{
		cout << "Error ! Not enough control point number ..." << endl;
		return 0;
	}

	while (abs(dtheta)>eps)
	{

		MatrixXd A_;
		VectorXd b_;

		A_.resize(cp_number * 3, 4);
		b_.resize(cp_number * 3, 1);

		for (int j = 0; j < cp_number; j++)
		{
			// A Matrix
			A_(j * 3, 0) = -coordinatesA[j][0] * sin(theta0) - coordinatesA[j][1] * cos(theta0);
			A_(j * 3, 1) = 1;
			A_(j * 3, 2) = 0;
			A_(j * 3, 3) = 0;

			A_(j * 3 + 1, 0) = coordinatesA[j][0] * cos(theta0) - coordinatesA[j][1] * sin(theta0);+
			A_(j * 3 + 1, 1) = 0;
			A_(j * 3 + 1, 2) = 1;
			A_(j * 3 + 1, 3) = 0;

			A_(j * 3 + 2, 0) = 0;
			A_(j * 3 + 2, 1) = 0;
			A_(j * 3 + 2, 2) = 0;
			A_(j * 3 + 2, 3) = 1;

			//b Vector
			b_(j * 3, 0) = coordinatesB[j][0] - coordinatesA[j][0] * cos(theta0) + coordinatesA[j][1] * sin(theta0);
			b_(j * 3 + 1, 0) = coordinatesB[j][1] - coordinatesA[j][0] * sin(theta0) - coordinatesA[j][1] * cos(theta0);
			b_(j * 3 + 2, 0) = coordinatesB[j][2] - coordinatesA[j][2];
		}

		//x=(ATA)-1(ATb)
		temp_trans = ((A_.transpose()*A_).inverse())*A_.transpose()*b_;
		dtheta = temp_trans(0, 0);

		theta0 += dtheta;

		iter_num++;

		//cout << "Result for iteration " << iter_num << " is " << endl
		//<< temp_trans(1, 0) << " , " << temp_trans(2, 0) << " , " << temp_trans(3, 0) << " , " << theta0 * 180 / M_PI <<endl;
	}

	transAB = temp_trans;

	theta = theta0;
	yaw_degree = theta * 180 / M_PI;

	tx = transAB(1, 0);
	ty = transAB(2, 0);
	tz = transAB(3, 0);


	cout.setf(ios::showpoint);  
	cout.precision(12);       


	cout << "Calculated by Linear Least Square"<<endl
		 << "Converged in " << iter_num << " iterations ..." << endl
		 << "Station B 's Coordinate and Orientation in A's System is:" << endl
		 << "X: " << tx << " m" << endl
		 << "Y: " << ty << " m" << endl
		 << "Z: " << tz << " m" << endl
		 << "yaw: " << yaw_degree << " degree" << endl;


	TransMatrixA2B(0, 0) = cos(theta);
	TransMatrixA2B(0, 1) = -sin(theta);
	TransMatrixA2B(0, 2) = 0;
	TransMatrixA2B(0, 3) = tx;

	TransMatrixA2B(1, 0) = sin(theta);
	TransMatrixA2B(1, 1) = cos(theta);
	TransMatrixA2B(1, 2) = 0;
	TransMatrixA2B(1, 3) = ty;

	TransMatrixA2B(2, 0) = 0;
	TransMatrixA2B(2, 1) = 0;
	TransMatrixA2B(2, 2) = 1;
	TransMatrixA2B(2, 3) = tz;

	TransMatrixA2B(3, 0) = 0;
	TransMatrixA2B(3, 1) = 0;
	TransMatrixA2B(3, 2) = 0;
	TransMatrixA2B(3, 3) = 1;

	cout << "The Transformation Matrix from Coordinate System A to B is: " << endl
		<< TransMatrixA2B << endl;


	// Checking
	if (pointnumberA >= pointnumberB) pointnumbercheck = pointnumberB;
	else pointnumbercheck = pointnumberA;

	if (pointnumbercheck <= cp_number) cout << "Not enough points for check ..." << endl;
	else
	{
		pointnumbercheck -= cp_number;
		for (int j = 0; j < pointnumbercheck; j++)
		{
			double X_tran, Y_tran, Z_tran, squaredist;
			X_tran = cos(theta)*coordinatesA[j + cp_number][0] - sin(theta)*coordinatesA[j + cp_number][1] + tx;
			Y_tran = sin(theta)*coordinatesA[j + cp_number][0] + cos(theta)*coordinatesA[j + cp_number][1] + ty;
			Z_tran = coordinatesA[j + cp_number][2] + tz;

			squaredist = (X_tran - coordinatesB[j + cp_number][0])*(X_tran - coordinatesB[j + cp_number][0]) +
				(Y_tran - coordinatesB[j + cp_number][1])*(Y_tran - coordinatesB[j + cp_number][1]) +
				(Z_tran - coordinatesB[j + cp_number][2])*(Z_tran - coordinatesB[j + cp_number][2]);
			sum_squaredist += squaredist;
		}

		RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

		cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
	}

	return 1;
}

template<typename PointT>
bool CRegistration<PointT>::SVD_6DOF(const std::vector <std::vector<double> > & coordinatesA, const std::vector <std::vector<double> > & coordinatesB, Matrix4d & TransMatrixA2B, int cp_number)
{
	Matrix4f transAB2D;
	pcl::PointCloud<PointT> Points2D_A, Points2D_B;
	double ZAB_mean, ZAB_sum;
	int pointnumberA, pointnumberB, pointnumbercheck;
	double RMSE_check, sum_squaredist;
	
	pointnumberA = coordinatesA.size();
	pointnumberB = coordinatesB.size();
	ZAB_sum = 0;
	sum_squaredist = 0;

	for (size_t i = 0; i < cp_number; i++)
	{
		PointT PtA,PtB;
		PtA.x = coordinatesA[i][0];
		PtA.y = coordinatesA[i][1];
		PtA.z = coordinatesA[i][2];
		
		PtB.x = coordinatesB[i][0];
		PtB.y = coordinatesB[i][1];
		PtB.z = coordinatesB[i][2];

		Points2D_A.push_back(PtA);
		Points2D_B.push_back(PtB);
		ZAB_sum += (coordinatesB[i][2] - coordinatesA[i][2]);
	}

	ZAB_mean = ZAB_sum / cp_number;

	if (cp_number < 2)
	{
		cout << "Error ! Not enough control point number ..." << endl;
		return 0;
	}

	pcl::registration::TransformationEstimationSVD<PointT, PointT> svd_estimator;
	svd_estimator.estimateRigidTransformation(Points2D_A, Points2D_B, transAB2D);

	TransMatrixA2B = transAB2D.cast<double>();

	/*
	TransMatrixA2B(0, 0) = transAB2D(0, 0);
	TransMatrixA2B(0, 1) = transAB2D(0, 1);
	TransMatrixA2B(0, 2) = 0;
	TransMatrixA2B(0, 3) = transAB2D(0, 3);

	TransMatrixA2B(1, 0) = transAB2D(1, 0);
	TransMatrixA2B(1, 1) = transAB2D(1, 1);
	TransMatrixA2B(1, 2) = 0;
	TransMatrixA2B(1, 3) = transAB2D(1, 3);

	TransMatrixA2B(2, 0) = 0;
	TransMatrixA2B(2, 1) = 0;
	TransMatrixA2B(2, 2) = 1;
	TransMatrixA2B(2, 3) = ZAB_mean;

	TransMatrixA2B(3, 0) = 0;
	TransMatrixA2B(3, 1) = 0;
	TransMatrixA2B(3, 2) = 0;
	TransMatrixA2B(3, 3) = 1;
	*/

	double tx, ty, tz, yaw_rad, yaw_degree;

	tx = TransMatrixA2B(0, 3);
	ty = TransMatrixA2B(1, 3);
	tz = TransMatrixA2B(2, 3);
	yaw_rad = acos(TransMatrixA2B(0, 0));
	if (TransMatrixA2B(1, 0) < 0) yaw_rad = -yaw_rad;
	yaw_degree = yaw_rad / M_PI * 180;

	cout << "Calculated by SVD" << endl
		 << "Station B 's Coordinate and Orientation in A's System is:" << endl
		 << "X: " << tx << " m" << endl
		 << "Y: " << ty << " m" << endl
		 << "Z: " << tz << " m" << endl
		 << "yaw: " << yaw_degree << " degree" << endl;


	cout << "The Transformation Matrix from Coordinate System A to B is: " << endl
		 << TransMatrixA2B << endl;


	// Checking
	if (pointnumberA >= pointnumberB) pointnumbercheck = pointnumberB;
	else pointnumbercheck = pointnumberA;

	if (pointnumbercheck <= cp_number) cout << "Not enough points for check ..." << endl;
	else
	{
		pointnumbercheck -= cp_number;
		for (int j = 0; j < pointnumbercheck; j++)
		{
			double X_tran, Y_tran, Z_tran, squaredist;
			X_tran = TransMatrixA2B(0, 0)*coordinatesA[j + cp_number][0] + TransMatrixA2B(0, 1)*coordinatesA[j + cp_number][1] + TransMatrixA2B(0, 2)*coordinatesA[j + cp_number][2]+ tx;
			Y_tran = TransMatrixA2B(1, 0)*coordinatesA[j + cp_number][0] + TransMatrixA2B(1, 1)*coordinatesA[j + cp_number][1] + TransMatrixA2B(1, 2)*coordinatesA[j + cp_number][2]+ ty;
			Z_tran = TransMatrixA2B(2, 0)*coordinatesA[j + cp_number][0] + TransMatrixA2B(2, 1)*coordinatesA[j + cp_number][1] + TransMatrixA2B(2, 2)*coordinatesA[j + cp_number][2]+ tz;

			squaredist = (X_tran - coordinatesB[j + cp_number][0])*(X_tran - coordinatesB[j + cp_number][0]) +
				(Y_tran - coordinatesB[j + cp_number][1])*(Y_tran - coordinatesB[j + cp_number][1]) +
				(Z_tran - coordinatesB[j + cp_number][2])*(Z_tran - coordinatesB[j + cp_number][2]);
			sum_squaredist += squaredist;
		}

		RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

		cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
	}
}

// template<typename PointT>
// bool CRegistration<PointT>::add_registration_edge(const typename pcl::PointCloud<PointT>::Ptr & subcloud1, const typename pcl::PointCloud<PointT>::Ptr & subcloud2, Constraint &con, 
// 	float cloud_Registration_PerturbateValue, int cloud_Registration_MaxIterNumber, bool cloud_Registration_UseReciprocalCorres, bool cloud_Registration_UseTrimmedRejector, float cloud_Registration_OverlapSearchRadius, int covariance_K, float cloud_Registration_MinOverlapForReg)
// {
// 	//Apply perturbation
// 	typename pcl::PointCloud<PointT>::Ptr subcloud1_perturbated(new pcl::PointCloud<PointT>());
// 	std::vector<float> p_vector(3, 0);
// 	Perturbation(subcloud1, subcloud1_perturbated,cloud_Registration_PerturbateValue, p_vector);

// 	//ICP Registration  [Trimmed Point-to-Point ICP / or you can switch to other metrics]
// 	typename pcl::PointCloud<PointT>::Ptr regcloud2(new pcl::PointCloud<PointT>());
// 	Eigen::Matrix4f Trans2_1p, Trans1p_2;
// 	if (ptplicp_reg(subcloud2, subcloud1_perturbated, regcloud2, Trans2_1p, cloud_Registration_MaxIterNumber, cloud_Registration_UseReciprocalCorres, cloud_Registration_UseTrimmedRejector, cloud_Registration_OverlapSearchRadius, covariance_K, cloud_Registration_MinOverlapForReg))
// 	{
// 		invTransform(Trans2_1p, Trans1p_2);
// 		con.Trans1_2 = Trans1p_2;
// 		con.Trans1_2(0, 3) += p_vector[0];
// 		con.Trans1_2(1, 3) += p_vector[1];
// 		con.Trans1_2(2, 3) += p_vector[2];
// 		cout.setf(ios::showpoint);
// 		cout.precision(10);
// 		cout << "Registration Done for cloud blocks #" << con.block1.unique_index << "  and  #" << con.block2.unique_index << endl;
// 		cout << "-----------------------------------------------------------------------------"<<endl;
// 		LOG(INFO) << "Registration Done for cloud blocks #" << con.block1.unique_index << "  and  #" << con.block2.unique_index;
// 		LOG(INFO) << "Edge Value: Transformation Matrix with perturbation:" << endl << con.Trans1_2;
// 		LOG(INFO) << "-----------------------------------------------------------------------------";
// 		return true;
// 	}
// 	else
// 	{
// 		con.con_type = 0; //non-constraint
// 		cout << "Registration Failed for cloud blocks #" << con.block1.unique_index << "  and  #" << con.block2.unique_index << endl;
// 		cout << "-----------------------------------------------------------------------------" << endl;
// 		LOG(INFO) << "Registration Failed for cloud blocks #"  << con.block1.unique_index << "  and  #" << con.block2.unique_index;
// 		LOG(INFO) << "-----------------------------------------------------------------------------";
// 		return false; 
// 		//Get rid of the misregistration: Delete the edge with too small point cloud overlapping ratio;
// 	}
// }

