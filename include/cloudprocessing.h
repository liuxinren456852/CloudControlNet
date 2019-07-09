//
// This file is for the general implements of several famous point cloud processing algorithms
// Dependent 3rd Libs: PCL (>1.7)  
// Author: Yue Pan et al. @ WHU LIESMARS
//

#ifndef CLOUD_PRO_H
#define CLOUD_PRO_H

#include "utility.h"

using namespace utility;
using namespace std;
using namespace Eigen;

template <typename PointT>
class CProceesing
{
public:

	void GroundFilter_PMF(const typename pcl::PointCloud<PointT>::Ptr &cloud, typename pcl::PointCloud<PointT>::Ptr &gcloud, typename pcl::PointCloud<PointT>::Ptr &ngcloud, int max_window_size, float slope, float initial_distance, float max_distance);   // PMF �������˲�;

	void planesegRansac(const typename pcl::PointCloud<PointT>::Ptr &cloud, float threshold, typename pcl::PointCloud<PointT>::Ptr & planecloud );    //Ransac ƽ�����;
	
	void groundprojection(const typename pcl::PointCloud<PointT>::Ptr &cloud, typename pcl::PointCloud<PointT>::Ptr & projcloud);                     //����ͶӰ;

	void SORFilter(const typename pcl::PointCloud<PointT>::Ptr & incloud, int MeanK, double std, typename pcl::PointCloud<PointT>::Ptr & outcloud);   //SOR ��Statisics Outliers Remover);

	void alphashape(const typename pcl::PointCloud<PointT>::Ptr &cloud, float alpha_value, typename pcl::PointCloud<PointT>::Ptr & boundary_cloud);   //Concave Hull Generation with alpha_shape;

	void CornerpointKNN(const typename pcl::PointCloud<PointT>::Ptr & boundary_cloud, int K, float disthreshold, float maxcos, typename pcl::PointCloud<PointT>::Ptr & corner_cloud);             //KNN corner point extraction 
	
	void CornerpointRadius(const typename pcl::PointCloud<PointT>::Ptr & boundary_cloud, float radius, float disthreshold, float maxcos, typename pcl::PointCloud<PointT>::Ptr & corner_cloud);   //Radius corner point extraction 

    //... More to add

protected:

private:

};


#endif //CLOUD_PRO_H