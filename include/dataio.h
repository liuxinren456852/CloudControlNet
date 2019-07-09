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
	
	// Please add some randomly sub-sampling output methods
    
	bool HDmap_data_import (const string &pointcloud_fileList, const string &pose_fileName, std::vector<Frame> &HD_map_data);      

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
	bool readLasFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not);     //With translation @Overload
	bool writeLasFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not);     //With translation @Overload
	bool readLasFileLast   (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud); //With translation of the last global shift

	// ply IO;
	bool readPlyFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writePlyFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	
	// txt IO;
	bool readTxtFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writeTxtFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writeTxtFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, int subsample_ratio);  //Randomly Subsample for saving @Overload

	// dxf IO;
    
    // pose IO;
	bool readposes(const string &fileName, std::vector<Eigen::Matrix4d> &poses);

	// Batch read filename from folder
	bool batchReadFileNamesInFolders(const std::string &folderName, const std::string & extension, std::vector<std::string> &fileNames);
	bool batchReadFileNamesInSubFolders(const std::string &folderName, const std::string & extension, std::vector<std::vector<std::string> > &fileNames);
	bool batchReadFileNamesInFoldersAndSubFolders(const std::string &folderName, const std::string & extension, std::vector<std::string> &fileNames);
	// Batch read multi-source filename from folder in one line
	void batchReadMultiSourceFileNamesInDataFolders(const std::string &ALS_folder, const std::string &TLS_folder, const std::string &MLS_folder, const std::string &BPLS_folder,
		std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files);

	// Display via VTK;
	void display          (const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, string displayname);
	void displaymulti     (const typename pcl::PointCloud<PointT>::Ptr &cloud0, const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, string displayname);
	
	// For ALS Division
	void ALS_block_by_time(const typename pcl::PointCloud<PointT>::Ptr &pointCloud, typename std::vector<pcl::PointCloud<PointT> > &CloudBlocks, float time_step_in_second);
	bool batchWriteBlockInColor(const string &fileName, typename std::vector<pcl::PointCloud<PointT> > &CloudBlocks, bool automatic_shift_or_not);

	// For coordinate transformation
	void pcXYZ2XY(const typename pcl::PointCloud<PointT>::Ptr &pointcloudxyz, pcl::PointCloud<pcl::PointXY>::Ptr &pointcloudxy);
	bool readindiceslist(std::vector<int> & indicesA, std::vector<int> & indicesB);
	bool readindiceslist(const typename pcl::PointCloud<PointT>::Ptr &CloudA, const typename pcl::PointCloud<PointT>::Ptr &CloudB, std::vector <std::vector<double> > & coordinatesA, std::vector <std::vector<double> > & coordinatesB);
	bool read_XYZ_XYZlist(std::vector <std::vector<double> > & coordinatesA, std::vector <std::vector<double> > & coordinatesB);
	bool read_XYZ_BLHlist(std::vector <std::vector<double> > & coordinatesSC_XYZ, std::vector <std::vector<double> > & coordinatesUTM_XYZ);
	bool tran_eng2utm(float centerlong_eng_proj);
	bool tran_wgs2eng(float centerlong_eng_proj, float proj_surface_h_eng);
	bool XYZ_4DOFCSTran(std::vector<double> &transpara); //4 parameters transform (plane)
	bool XYZ_7DOFCSTran(std::vector<double> &transpara); //7 parameters transform (space)
	bool lasfileGK2UTM(const string &fileName); //Gauss Projection to UTM projection
	bool lasfileshift(const string &fileName, vector<double> &shift);  // translation
	double cal_cor_RMSE(std::vector <std::vector<double> > & coordinatesA, std::vector <std::vector<double> > & coordinatesB);

	//Read in blocks
	bool readLasBlock(const string &fileName, CloudBlock & block);                                   //Without translation
	bool readLasBlock(const string &fileName, int data_type_, int strip_num_, int num_in_strip_, CloudBlock & block);//@Overload
	bool readLasBlock(const string &fileName, CloudBlock & block, bool automatic_shift_or_not);      //With translation @Overload
	void batchReadMultiSourceLasBlock(std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files,
		std::vector<std::vector<CloudBlock> > &ALS_strip_blocks, std::vector<CloudBlock> &TLS_blocks, std::vector<CloudBlock> &MLS_blocks, std::vector<CloudBlock> &BPLS_blocks, std::vector<CloudBlock> &All_blocks);

	//Read point cloud pair from constraint
	void readLasCloudPairfromCon(const Constraint &this_con, std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files, 
		string &Filename1, string &Filename2, typename pcl::PointCloud<PointT>::Ptr &cloud1, typename pcl::PointCloud<PointT>::Ptr &cloud2);

	//Batch down-sample point cloud pair
	void batchdownsamplepair(const Constraint &this_con, typename pcl::PointCloud<PointT>::Ptr &cloud1, typename pcl::PointCloud<PointT>::Ptr &cloud2, typename pcl::PointCloud<PointT>::Ptr &subcloud1, typename pcl::PointCloud<PointT>::Ptr &subcloud2,
		float ALS_radius, float TLS_radius, float MLS_radius, float BPLS_radius);

	//Batch output final point clouds
	void batchwritefinalcloud(vector<CloudBlock> &All_blocks, std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files);

	//Display of boxes
	void display2Dboxes(const vector<CloudBlock> &blocks);
	void display2Dcons(const vector<Constraint> &cons);

	
 

protected:

private:
	vector<double> global_shift;
};


#endif //DATAIO_H 