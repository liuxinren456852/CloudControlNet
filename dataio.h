//
// This file is used for the Reading, Writing and Displaying of Point Cloud of various formats.
// Dependent 3rd Libs: PCL (>1.7)  liblas
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#ifndef DATAIO 
#define DATAIO

//pcl
#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>
//liblas
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;

template <typename PointT>
class DataIo : public CloudUtility<PointT>
{
public:

	/* //Constructor
	DataIo(){
		
	}*/
	
	// Point Cloud IO for all kinds of formats;
	// pc_format: 1.pcd , 2.las , 3.ply , others;
	bool readCloudFile     (const int pc_format, const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writeCloudFile    (const int pc_format, const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);

	// pcd IO;
	bool readPcdFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writePcdFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	
	// las IO;
	bool readLasFileHeader (const string &fileName, liblas::Header& header);
	bool readLasFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud); //Without translation
	bool writeLasFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud); //Without translation
	bool readLasFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, double & X_origin, double & Y_origin); //With translation @Override
	bool writeLasFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, double X_origin, double Y_origin);     //With translation @Override
	 
	// ply IO;
	bool readPlyFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writePlyFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	
	// txt IO;

	// Display via VTK;
	void display          (const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, string displayname);
	void displaymulti     (const typename pcl::PointCloud<PointT>::Ptr &cloud0, const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, string displayname);
	
protected:

private:
	
};


#endif