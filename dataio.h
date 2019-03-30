//
// This file is used for the Reading, Writing and Displaying of Point Cloud of various formats.
// Dependent 3rd Libs: PCL (>1.7)  liblas  VTK
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#ifndef DATAIO_H 
#define DATAIO_H

//pcl
#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>
//liblas
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

#include <vector>
#include "utility.h"
#include "find_constraint.h"

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
	bool readCloudFile     (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writeCloudFile    (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);

	// pcd IO;
	bool readPcdFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writePcdFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	
	// las IO;
	bool readLasFileHeader (const string &fileName, liblas::Header& header);
	bool readLasFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud); //Without translation
	bool writeLasFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud); //Without translation
	bool readLasFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not);     //With translation @Override
	bool writeLasFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not);     //With translation @Override
	 
	// ply IO;
	bool readPlyFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writePlyFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	
	// txt IO;
	bool readTxtFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writeTxtFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writeTxtFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, int subsample_ratio);  //Randomly Subsample for saving @Override

	// Batch read filename from folder
	bool batchReadFileNamesInFolders(const std::string &folderName, const std::string & extension, std::vector<std::string> &fileNames);
	bool batchReadFileNamesInSubFolders(const std::string &folderName, const std::string & extension, std::vector<std::vector<std::string>> &fileNames);
	bool batchReadFileNamesInFoldersAndSubFolders(const std::string &folderName, const std::string & extension, std::vector<std::string> &fileNames);

	// Display via VTK;
	void display          (const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, string displayname);
	void displaymulti     (const typename pcl::PointCloud<PointT>::Ptr &cloud0, const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, string displayname);
	
	// For ALS Division
	void ALS_block_by_time(const typename pcl::PointCloud<PointT>::Ptr &pointCloud, typename vector<pcl::PointCloud<PointT>> &CloudBlocks, float time_step_in_second);
	bool batchWriteBlockInColor(const string &fileName, typename vector<pcl::PointCloud<PointT>> &CloudBlocks, bool automatic_shift_or_not);

	// For TLS localization and orientation [4DOF] with 2 control points
	void pcXYZ2XY(const typename pcl::PointCloud<PointT>::Ptr &pointcloudxyz, pcl::PointCloud<pcl::PointXY>::Ptr &pointcloudxy);
	bool readindiceslist(std::vector<int> & indicesA, std::vector<int> & indicesB);
	bool readindiceslist(const typename pcl::PointCloud<PointT>::Ptr &CloudA, const typename pcl::PointCloud<PointT>::Ptr &CloudB, std::vector <std::vector<double>> & coordinatesA, std::vector <std::vector<double>> & coordinatesB);
	bool read_XYZ_XYZlist(std::vector <std::vector<double>> & coordinatesA, std::vector <std::vector<double>> & coordinatesB);
	bool read_XYZ_BLHlist(std::vector <std::vector<double>> & coordinatesSC_XYZ, std::vector <std::vector<double>> & coordinatesUTM_XYZ);

	//Read in blocks
	bool readLasBlock(const string &fileName, CloudBlock & block);                                   //Without translation
	bool readLasBlock(const string &fileName, int data_type_, int strip_num_, int num_in_strip_, CloudBlock & block);//@Override
	bool readLasBlock(const string &fileName, CloudBlock & block, bool automatic_shift_or_not);      //With translation @Override


	//Display of boxes
	void display2Dboxes(const vector<CloudBlock> &blocks);
	void display2Dcons(const vector<Constraint> &cons);

	//Point Cloud Translation
	bool lasfileshift(const string &fileName, vector<double> &shift);


protected:

private:
	vector<double> global_shift;
};


#endif //DATAIO_H 