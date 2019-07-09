//
// This file is used for the Reading, Writing and Displaying of Point Cloud of various formats.
// Dependent 3rd Libs: PCL (>1.7)  liblas  VTK
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#include "dataio.h"
#include "utility.h"
#include "GeoTran.h"
#include "voxelFilter.h"
#include "common_reg.h"

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <glog/logging.h>

#include <string>
#include <fstream>
#include <vector>


using namespace  std;
using namespace  utility;
using namespace  boost::filesystem;

template <typename PointT>
bool DataIo<PointT>::HDmap_data_import (const string &pointcloud_fileList, const string &pose_fileName, std::vector<Frame> &HD_map_data)
{
    //Import pointcloud filenames
	vector<string> pointcloud_filenames;
	ifstream lidar_file_list(pointcloud_fileList.c_str());
	while (lidar_file_list.peek()!=EOF)
    {   
		string curfile;
        lidar_file_list>>curfile;
        pointcloud_filenames.push_back(curfile);
	}
    printf("Pointcloud filenames imported done, there are %d frames in [%s]\n",pointcloud_filenames.size(),pointcloud_fileList.c_str());

	//Import poses
    //Pose should be complete (corresponding to point cloud files)
	vector<Eigen::Matrix4d> poses;
    readposes(pose_fileName,poses);
    printf("Pose imported done, there are %d frames in [%s]\n",poses.size(),pose_fileName.c_str());

	//Save data
	int HD_map_datasize=min_(pointcloud_filenames.size(),poses.size());
	HD_map_data.resize(HD_map_datasize);
	for (int i=0;i<HD_map_datasize;i++)
	{
		HD_map_data[i].unique_id=i;
		HD_map_data[i].pcd_file_name=pointcloud_filenames[i];
		HD_map_data[i].oxts_pose=poses[i];
        HD_map_data[i].oxts_postion(0)=poses[i](0,3);
		HD_map_data[i].oxts_postion(1)=poses[i](1,3);
		HD_map_data[i].oxts_postion(2)=poses[i](2,3);

	}
	 printf("Save HDMap Data done, there are %d co-frames.\n",HD_map_datasize);

}   


template <typename PointT>
bool DataIo<PointT>::readposes(const string &fileName, vector<Eigen::Matrix4d> &poses)
{
   std::ifstream in(fileName.c_str(),ios::in);
	if (!in)
	{
		return 0;
	}
	
    Eigen::Matrix4d pose_;
    string file_;
    int i = 0;

	while (!in.eof())
	{
        in >> file_;
        in >> pose_(0,0)>>  pose_(0,1) >>  pose_(0,2) >>  pose_(0,3);
        in >> pose_(1,0)>>  pose_(1,1) >>  pose_(1,2) >>  pose_(1,3);
        in >> pose_(2,0)>>  pose_(2,1) >>  pose_(2,2) >>  pose_(2,3);
        
        if (pose_(0,3)>1000000 || pose_(1,3)>1000000) printf("Translation is too large. Do global shift to avoid precison loss\n");
		if (in.fail())
		{
			break;
		}
		pose_(3,0)=0;pose_(3,1)=0;pose_(3,2)=0;pose_(3,3)=1;
		poses.push_back(pose_);
        
        //cout<<i<<endl<<poses[i]<<endl;

        ++i;   
	}
	in.close();
	//cout << "Import finished ... ..." << endl;
	return 1;
}


template <typename PointT>
bool DataIo<PointT>::readCloudFile(const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	string extension;
	extension = fileName.substr(fileName.find_last_of('.') + 1);     //Get the suffix of the file;
	
	if (!strcmp(extension.c_str(), "pcd"))
	{
		readPcdFile(fileName, pointCloud);
		cout << "A pcd file has been imported" << endl;
	}
	else if (!strcmp(extension.c_str(), "las"))
	{
		bool global_shift_or_not=0;
		cout << "Would you like to do a global shift ?  0. No  1. Yes [default 0]" << endl;
		cin >> global_shift_or_not;
		if (!global_shift_or_not)
		{
			readLasFile(fileName, pointCloud);
		}
		else
		{
			bool use_automatic_drift = 0;
			cout << "Using the automatic shift or enter the global shift yourself ? "<<endl
				<< "0. Read a global shift file  1.Use the automatic shift [default 0]" << endl;
			cin >> use_automatic_drift;
			readLasFile(fileName, pointCloud,use_automatic_drift);
		}
		cout << "A las file has been imported" << endl;
	}
	else if (!strcmp(extension.c_str(), "ply"))
	{
		readPlyFile(fileName, pointCloud);
		cout << "A ply file has been imported" << endl;
	}		
	else if (!strcmp(extension.c_str(), "txt"))
	{
		readTxtFile(fileName, pointCloud);
		cout << "A txt file has been imported" << endl;
	}
	else
	{
		cout << "Undefined Point Cloud Format." << endl;
		return 0;
	}
}

template <typename PointT>
bool DataIo<PointT>::writeCloudFile(const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	string extension;
	extension = fileName.substr(fileName.find_last_of('.') + 1);     //Get the suffix of the file;

	if (!strcmp(extension.c_str(), "pcd"))
	{
		writePcdFile(fileName, pointCloud);
		cout << "A pcd file has been exported" << endl;
	}
	else if (!strcmp(extension.c_str(), "las"))
	{
		bool global_shift_or_not = 0;
		cout << "Would you like to do a global shift ?  0. No  1. Yes [default 0]" << endl;
		cin >> global_shift_or_not;
		if (!global_shift_or_not)
		{
			writeLasFile(fileName, pointCloud);
		}
		else
		{
			bool use_automatic_drift = 0;
			cout << "Using the automatic shift or enter the global shift yourself ? " << endl
				<< "0. Read a global shift file  1.Use the automatic shift [default 0]" << endl;
			cin >> use_automatic_drift;
			writeLasFile(fileName, pointCloud, use_automatic_drift);
		}
		cout << "A las file has been exported" << endl;
	}
	else if (!strcmp(extension.c_str(), "ply"))
	{
		writePlyFile(fileName, pointCloud);
		cout << "A ply file has been exported" << endl;
	}
	else if (!strcmp(extension.c_str(), "txt"))
	{
		writeTxtFile(fileName, pointCloud);
		cout << "A txt file has been exported" << endl;
	}
	else
	{
		cout << "Undefined Point Cloud Format." << endl;
		return 0;
	}
}

template <typename PointT> 
bool DataIo<PointT>::readPcdFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	if (pcl::io::loadPCDFile<PointT>(fileName, *pointCloud) == -1) 
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}
	return true;
}

template <typename PointT>
bool DataIo<PointT>::writePcdFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1) 
	{
		PCL_ERROR("Couldn't write file\n");
		return false;
	}
	return true;
}

template <typename PointT>
bool DataIo<PointT>::readLasFileHeader(const std::string &fileName, liblas::Header& header)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}
	else
	{
		std::ifstream ifs;
		ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			return 0;
		}

		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);

		header = reader.GetHeader();
	}
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::readLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)  //Without translation
{
	//cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;
	
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "Matched Terms are not found." << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	//Bounding box Information 
	double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
	Xmin = header.GetMinX();
	Ymin = header.GetMinY();
	Zmin = header.GetMinZ();
	Xmax = header.GetMaxX();
	Ymax = header.GetMaxY();
	Zmax = header.GetMaxZ();

	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		PointT  pt;
		pt.x = p.GetX();
		pt.y = p.GetY();
		pt.z = p.GetZ();
		
		//------------------------------------------------Assign Intensity--------------------------------------------------//
		//If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
		//If the Point template PointT is without intensity, you should comment the line.
		//pt.intensity = p.GetIntensity();	
		//pt.intensity = p.GetTime();
		//pt.intensity = p.GetScanAngleRank();
		//pt.intensity = p.GetNumberOfReturns();
		//pt.intensity = p.GetScanDirection();

		//---------------------------------------------------Assign Color--------------------------------------------------//
		//If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
		//If the Point template PointT is without RGB, you should comment the line.
		//liblas::Color lasColor;
		//lasColor= p.GetColor();
		//pt.r = lasColor.GetRed();
		//pt.g = lasColor.GetGreen();
		//pt.b = lasColor.GetBlue();
		
		pointCloud->points.push_back(pt);
	}
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::writeLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud) //Without translation
{
	
	Bounds bound;
	getCloudBound(*pointCloud, bound);

	ofstream ofs;
	ofs.open(fileName.c_str(), std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		liblas::Header header;
		header.SetDataFormatId(liblas::ePointFormat2);
		header.SetVersionMajor(1);
		header.SetVersionMinor(2);
		header.SetMin(bound.min_x, bound.min_y, bound.min_z);
		header.SetMax(bound.max_x, bound.max_y, bound.max_z);
		header.SetOffset((bound.min_x + bound.max_x) / 2.0, (bound.min_y + bound.max_y) / 2.0, (bound.min_z + bound.max_z) / 2.0);
		header.SetScale(0.01, 0.01, 0.01);
		header.SetPointRecordsCount(pointCloud->points.size());

		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (int i = 0; i < pointCloud->points.size(); i++)
		{
			pt.SetCoordinates(double(pointCloud->points[i].x), double(pointCloud->points[i].y), double(pointCloud->points[i].z));
			
			//If the Point template PointT is without intensity, you should comment the line.
			//pt.SetIntensity(pointCloud->points[i].intensity);

			//If the Point template PointT is without RGB, you should comment the line.
			//liblas::Color lasColor;
			//lasColor.SetRed(pointCloud->points[i].r);
			//lasColor.SetGreen(pointCloud->points[i].g);
			//lasColor.SetBlue(pointCloud->points[i].b);
			//pt.SetColor(lasColor);

			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::readLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not)  //With translation
{
	global_shift.resize(3);
	//cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;

	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "Matched Terms are not found." << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	//Bounding box Information 
	double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
	Xmin = header.GetMinX();
	Ymin = header.GetMinY();
	Zmin = header.GetMinZ();
	Xmax = header.GetMaxX();
	Ymax = header.GetMaxY();
	Zmax = header.GetMaxZ();

	if (automatic_shift_or_not)
	{
		// Automatic Gloabl Shift Value;
		global_shift[0] = -Xmin;
		global_shift[1] = -Ymin;
		global_shift[2] = -Zmin;

		ofstream out("GlobalShift.txt", ios::out);
		out << setiosflags(ios::fixed) << setprecision(8) << global_shift[0] << endl;
		out << setiosflags(ios::fixed) << setprecision(8) << global_shift[1] << endl;
		out << setiosflags(ios::fixed) << setprecision(8) << global_shift[2] << endl;
		out.close();

		//cout << "A txt File named GlobalShift.txt is saved in current Folder" << endl;
		LOG(INFO) << "A txt File named GlobalShift.txt is saved in current Folder";
	}
	else
	{
		string fileGlobalShift;
		cout << "Please enter or drag in the Global Shift File" << endl
			<<"Example [GlobalShift.txt] :"<<endl
			<<"-366370.90"<<endl
			<<"-3451297.82"<<endl
			<<"-14.29"<<endl;
		
		cin >> fileGlobalShift;
		
		ifstream in(fileGlobalShift, ios::in);
		in >> global_shift[0];
		in >> global_shift[1];
		in >> global_shift[2];
		in.close();
	}

	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		PointT pt;
		
		//A translation to keep the precision
		//��һ��ƽ�ƣ�������UTM WGS84�µĵ�����̫���ˣ�����ɾ�����ʧ��. ��Ϊlas�Ķ�ȡ��������double�ģ���pcd��float��;
		pt.x = p.GetX() + global_shift[0];
		pt.y = p.GetY() + global_shift[1];
		pt.z = p.GetZ() + global_shift[2];
		
		//------------------------------------------------Assign Intensity--------------------------------------------------//
		//If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
		//If the Point template PointT is without intensity, you should comment the line.
		//pt.intensity = p.GetIntensity();	
		//pt.intensity = p.GetTime();
		//pt.intensity = p.GetScanAngleRank();
		//pt.intensity = p.GetNumberOfReturns();
		//pt.intensity = p.GetScanDirection();

		//---------------------------------------------------Assign Color--------------------------------------------------//
		//If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
		//If the Point template PointT is without RGB, you should comment the line.
		//liblas::Color lasColor;
		//lasColor= p.GetColor();
		//pt.r = lasColor.GetRed();
		//pt.g = lasColor.GetGreen();
		//pt.b = lasColor.GetBlue();


		pointCloud->points.push_back(pt);
	}
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::writeLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not) //With translation
{
	global_shift.resize(3);
	
	Bounds bound;
	getCloudBound(*pointCloud, bound);

	if (!automatic_shift_or_not)
	{
		cout << "Use default (last input) global shift or not" << endl
			 << "1. Yes  0. No" << endl;
		bool use_last_shift;
		cin >> use_last_shift;
		if (!use_last_shift)
		{
			string fileGlobalShift;
			cout << "Please enter or drag in the Global Shift File" << endl
				<< "Example [GlobalShift.txt] :" << endl
				<< "-366370.90" << endl
				<< "-3451297.82" << endl
				<< "-14.29" << endl;

			cin >> fileGlobalShift;


			ifstream in(fileGlobalShift, ios::in);
			in >> global_shift[0];
			in >> global_shift[1];
			in >> global_shift[2];
			in.close();
		}
	}

	ofstream ofs;
	ofs.open(fileName.c_str(), std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		liblas::Header header;
		header.SetDataFormatId(liblas::ePointFormat2);
		header.SetVersionMajor(1);
		header.SetVersionMinor(2);
		header.SetMin(bound.min_x - global_shift[0], bound.min_y - global_shift[1], bound.min_z - global_shift[2]);
		header.SetMax(bound.max_x - global_shift[0], bound.max_y - global_shift[1], bound.max_z - global_shift[2]);
		header.SetOffset((bound.min_x + bound.max_x) / 2.0 - global_shift[0], (bound.min_y + bound.max_y) / 2.0 - global_shift[1], (bound.min_z + bound.max_z) / 2.0 - global_shift[2]);
		header.SetScale(0.01, 0.01, 0.01);
		header.SetPointRecordsCount(pointCloud->points.size());

		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (size_t i = 0; i < pointCloud->points.size(); i++)
		{
			pt.SetCoordinates(double(pointCloud->points[i].x) - global_shift[0], double(pointCloud->points[i].y) - global_shift[1], double(pointCloud->points[i].z) - global_shift[2]);
			
			//If the Point template PointT is without intensity, you should comment the line.
			//pt.SetIntensity(pointCloud->points[i].intensity);

			//If the Point template PointT is without RGB, you should comment the line.
			//liblas::Color lasColor;
			//lasColor.SetRed(pointCloud->points[i].r);
			//lasColor.SetGreen(pointCloud->points[i].g);
			//lasColor.SetBlue(pointCloud->points[i].b);
			//pt.SetColor(lasColor);
			
			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::readLasFileLast(const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "Matched Terms are not found." << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		PointT pt;

		//A translation to keep the precision
		//��һ��ƽ�ƣ�������UTM WGS84�µĵ�����̫���ˣ�����ɾ�����ʧ��. ��Ϊlas�Ķ�ȡ��������double�ģ���pcd��float��;
		pt.x = p.GetX() + global_shift[0];
		pt.y = p.GetY() + global_shift[1];
		pt.z = p.GetZ() + global_shift[2];

		//------------------------------------------------Assign Intensity--------------------------------------------------//
		//If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
		//If the Point template PointT is without intensity, you should comment the line.
		//pt.intensity = p.GetIntensity();	
		//pt.intensity = p.GetTime();
		//pt.intensity = p.GetScanAngleRank();
		//pt.intensity = p.GetNumberOfReturns();
		//pt.intensity = p.GetScanDirection();

		//---------------------------------------------------Assign Color--------------------------------------------------//
		//If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
		//If the Point template PointT is without RGB, you should comment the line.
		//liblas::Color lasColor;
		//lasColor= p.GetColor();
		//pt.r = lasColor.GetRed();
		//pt.g = lasColor.GetGreen();
		//pt.b = lasColor.GetBlue();

		pointCloud->points.push_back(pt);
	}
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::readPlyFile(const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	if (pcl::io::loadPLYFile<PointT>(fileName, *pointCloud) == -1) //* load the file 
	{
		PCL_ERROR("Couldn't read file \n");
		return (-1);
	}
}

template <typename PointT>
bool DataIo<PointT>::writePlyFile(const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	if (pcl::io::savePLYFile<PointT>(fileName, *pointCloud) == -1) //* load the file 
	{
		PCL_ERROR("Couldn't write file \n");
		return (-1);
	}
}

template <typename PointT>
bool  DataIo<PointT>::batchReadFileNamesInFolders(const std::string &folderName, const std::string & extension, std::vector<std::string> &fileNames)
{
	if (!exists(folderName))
	{
		return 0;
	}
	else
	{
		directory_iterator end_iter;
		for (directory_iterator iter(folderName); iter != end_iter; ++iter)
		{
			if (is_regular_file(iter->status()))
			{
				string fileName;
				fileName = iter->path().string();

				path dir(fileName);

				if (!dir.extension().string().empty())
				{
					if (!fileName.substr(fileName.rfind('.')).compare(extension))
					{
						fileNames.push_back(fileName);
					}
				}
			}
		}
	}
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::batchReadFileNamesInFoldersAndSubFolders(const std::string &folderName, const std::string & extension, std::vector<std::string> &fileNames)
{
	boost::filesystem::path  fullpath(folderName);
	if (!exists(fullpath))
	{
		return false;
	}
	recursive_directory_iterator end_iter;
	for (recursive_directory_iterator iter(fullpath); iter != end_iter; iter++)
	{
		try
		{
			if (is_directory(*iter))
			{
			}
			else
			{
				std::string sFileName = iter->path().string();
				path dir(sFileName);

				if (!dir.extension().string().empty())
				{
					if (!sFileName.substr(sFileName.rfind('.')).compare(extension))
					{
						fileNames.push_back(sFileName);
					}
				}
			}
		}
		catch (const std::exception & ex)
		{
			std::cerr << ex.what() << std::endl;
			continue;
		}
	}
	return true;
}

template <typename PointT>
bool DataIo<PointT>::batchReadFileNamesInSubFolders(const std::string &folderName, const std::string & extension, std::vector<std::vector<std::string> > &fileNames)
{
	int subfolder_num = 0;

	if (!exists(folderName))
	{
		return 0;
	}
	else
	{
		directory_iterator end_iter;
		for (directory_iterator iter(folderName); iter != end_iter; ++iter)
		{
			if (is_directory(iter->status()))
			{
				string subfoldername;
				subfoldername = iter->path().string();

				std::vector<std::string> fileNames_in_subfolder;
				batchReadFileNamesInFolders(subfoldername, extension, fileNames_in_subfolder);
				fileNames.push_back(fileNames_in_subfolder);
				subfolder_num++;
			}
		}
	}
	//cout << subfolder_num << " Sub-folders in the folder have been processed" << endl;
	return 1;
}

template <typename PointT>
void DataIo<PointT>::batchReadMultiSourceFileNamesInDataFolders(const std::string &ALS_folder, const std::string &TLS_folder, const std::string &MLS_folder, const std::string &BPLS_folder,
	std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files)
{
	batchReadFileNamesInSubFolders(ALS_folder, ".las", ALS_strip_files);
	batchReadFileNamesInFolders(TLS_folder, ".las", TLS_files);
	batchReadFileNamesInFolders(MLS_folder, ".las", MLS_files);
    batchReadFileNamesInFolders(BPLS_folder, ".las", BPLS_files);
	LOG(INFO) << "All Filenames have been imported ...";
}

template <typename PointT>
bool DataIo<PointT>::readTxtFile(const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	ifstream in(fileName, ios::in);
	if (!in)
	{
		return 0;
	}
	double x_ = 0, y_ = 0, z_ = 0;
	int i = 0;
	while (!in.eof())
	{
		in >> x_ >> y_ >> z_;
		if (in.fail())
		{
			break;
		}
		PointT Pt;
		Pt.x = x_;
		Pt.y = y_;
		Pt.z = z_;
		pointCloud->points.push_back(Pt);
		++i;
	}
	in.close();
	//cout << "Import finished ... ..." << endl;
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::writeTxtFile(const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	ofstream ofs;
	ofs.open(fileName.c_str());
	if (ofs.is_open())
	{
		for (size_t i = 0; i < pointCloud->size(); ++i)
		{
				ofs << setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].x << "  "
					<< setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].y << "  "
					<< setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].z
					//<<"  "<< setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].intensity
					<< endl;
		}
		ofs.close();
	}
	else{ return 0; }
	//cout << "Output finished ... ..." << endl;
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::writeTxtFile(const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, int subsample_ratio)
{
	ofstream ofs;
    ofs.open(fileName.c_str());
	if (ofs.is_open())
	{
		for (size_t i = 0; i < pointCloud->size(); ++i)
		{
				if (i % subsample_ratio == 0)//Subsampling;
				{  
					ofs << setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].x << "  "
						<< setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].y << "  "
						<< setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].z 
						//<<"  "<< setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].intensity
						<< endl;
				}
		}
		ofs.close();
	}
	else{ return 0; }
	//cout << "Output finished ... ..." << endl;
	return 1;
}

template <typename PointT>
void DataIo<PointT>::display(const typename pcl::PointCloud<PointT>::Ptr &Cloud1, const typename pcl::PointCloud<PointT>::Ptr &Cloud2, string displayname)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	pcXYZRGBPtr pointcloud1(new pcXYZRGB());
	pcXYZRGBPtr pointcloud2(new pcXYZRGB());

	for (size_t i = 0; i < Cloud1->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = Cloud1->points[i].x;
		pt.y = Cloud1->points[i].y;
		pt.z = Cloud1->points[i].z;
		pt.r = 255;
		pt.g = 215;
		pt.b = 0;
		pointcloud1->points.push_back(pt);
	} // Golden

	viewer->addPointCloud(pointcloud1, "pointcloudT");

	for (size_t i = 0; i < Cloud2->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = Cloud2->points[i].x;
		pt.y = Cloud2->points[i].y;
		pt.z = Cloud2->points[i].z;
		pt.r = 233;
		pt.g = 233;
		pt.b = 216;
		pointcloud2->points.push_back(pt);
	} // Silver

	viewer->addPointCloud(pointcloud2, "pointcloudS");
	
	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

template <typename PointT>
void DataIo<PointT>::displaymulti(const typename pcl::PointCloud<PointT>::Ptr &Cloud0, const typename pcl::PointCloud<PointT>::Ptr &Cloud1, const typename pcl::PointCloud<PointT>::Ptr &Cloud2, string displayname)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	pcXYZRGBPtr pointcloud0(new pcXYZRGB());
	pcXYZRGBPtr pointcloud1(new pcXYZRGB());
	pcXYZRGBPtr pointcloud2(new pcXYZRGB());

	for (size_t i = 0; i < Cloud0->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = Cloud0->points[i].x;
		pt.y = Cloud0->points[i].y;
		pt.z = Cloud0->points[i].z;
		pt.r = 255;
		pt.g = 0;
		pt.b = 0;
		pointcloud0->points.push_back(pt);
	} // Red

	viewer->addPointCloud(pointcloud0, "pointcloud_reference");

	for (size_t i = 0; i < Cloud1->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = Cloud1->points[i].x;
		pt.y = Cloud1->points[i].y;
		pt.z = Cloud1->points[i].z;
		pt.r = 0;
		pt.g = 0;
		pt.b = 255;
		pointcloud1->points.push_back(pt);
	} // Blue

	viewer->addPointCloud(pointcloud1, "pointcloud_reg_method1");

	for (size_t i = 0; i < Cloud2->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = Cloud2->points[i].x;
		pt.y = Cloud2->points[i].y;
		pt.z = Cloud2->points[i].z;
		pt.r = 0;
		pt.g = 255;
		pt.b = 0;
		pointcloud2->points.push_back(pt);
	} // Green

	viewer->addPointCloud(pointcloud2, "pointcloud_reg_method2");

	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


template <typename PointT>
void DataIo<PointT>::ALS_block_by_time(const typename pcl::PointCloud<PointT>::Ptr &pointCloud, typename std::vector<pcl::PointCloud<PointT> > &CloudBlocks, float time_step_in_second)
{
	float time_max = -FLT_MAX;
	float time_min = FLT_MAX;

	float time_range;

	for (size_t i = 0; i < pointCloud->size(); i++)
	{
		if (pointCloud->points[i].intensity > time_max) time_max = pointCloud->points[i].intensity;
		if (pointCloud->points[i].intensity < time_min) time_min = pointCloud->points[i].intensity;
	}

	time_range = time_max - time_min;

	CloudBlocks.resize(int(time_range / time_step_in_second) + 1);

	for (size_t i = 0; i < pointCloud->size(); i++)
	{
		int index = int(1.0 * (pointCloud->points[i].intensity - time_min) / time_step_in_second);
		CloudBlocks[index].push_back(pointCloud->points[i]);
	}
}
template <typename PointT>
bool DataIo<PointT>::batchWriteBlockInColor(const string &fileName, typename std::vector<pcl::PointCloud<PointT> >  &CloudBlocks, bool automatic_shift_or_not)
{
	global_shift.resize(3);

	if (!automatic_shift_or_not)
	{
		string fileGlobalShift;
		cout << "Please enter or drag in the Global Shift File" << endl
			<< "Example [GlobalShift.txt] :" << endl
			<< "-366370.90" << endl
			<< "-3451297.82" << endl
			<< "-14.29" << endl;

		cin >> fileGlobalShift;

		ifstream in(fileGlobalShift, ios::in);
		in >> global_shift[0];
		in >> global_shift[1];
		in >> global_shift[2];
		in.close();
	}

	for (int j = 0; j < CloudBlocks.size(); j++)
	{
		Bounds bound;
		getCloudBound(CloudBlocks[j], bound);
        
		liblas::Color lasColor;
		lasColor.SetRed(255 * (rand() / (1.0 + RAND_MAX)));
		lasColor.SetGreen(255 * (rand() / (1.0 + RAND_MAX)));
		lasColor.SetBlue(255 * (rand() / (1.0 + RAND_MAX)));

		string BlockFolder, BlockFilename;
		ostringstream oss;
		oss.setf(ios::right);      //���ö��뷽ʽΪ�Ҷ��� 
		oss.fill('0');             //������䷽ʽ,����λ��0
		oss.width(3);              //���ÿ���Ϊ2��ֻ������������� 
		oss << j; 
		// �˴����ɣ���000��001��002������˳������;
		
		BlockFolder = fileName.substr(0, fileName.rfind("."));

		if (!boost::filesystem::exists(BlockFolder))
		{
			boost::filesystem::create_directory(BlockFolder);
		}

		BlockFilename = BlockFolder + fileName.substr(fileName.rfind("\\"), fileName.rfind(".") - fileName.rfind("\\"))+ "_" + oss.str() + ".las";
		if (j == 0) cout << BlockFilename << endl;

		ofstream ofs;
		ofs.open(BlockFilename.c_str(), std::ios::out | std::ios::binary);
		if (ofs.is_open())
		{
			liblas::Header header;
			header.SetDataFormatId(liblas::ePointFormat2);
			header.SetVersionMajor(1);
			header.SetVersionMinor(2);
			header.SetMin(bound.min_x - global_shift[0], bound.min_y - global_shift[1], bound.min_z - global_shift[2]);
			header.SetMax(bound.max_x - global_shift[0], bound.max_y - global_shift[1], bound.max_z - global_shift[2]);
			header.SetOffset((bound.min_x + bound.max_x) / 2.0 - global_shift[0], (bound.min_y + bound.max_y) / 2.0 - global_shift[1], (bound.min_z + bound.max_z) / 2.0 - global_shift[2]);
			header.SetScale(0.01, 0.01, 0.01);
			header.SetPointRecordsCount(CloudBlocks[j].points.size());

			liblas::Writer writer(ofs, header);
			liblas::Point pt(&header);

			for (size_t i = 0; i < CloudBlocks[j].points.size(); i++)
			{
				pt.SetCoordinates(double(CloudBlocks[j].points[i].x) - global_shift[0], double(CloudBlocks[j].points[i].y) - global_shift[1], double(CloudBlocks[j].points[i].z) - global_shift[2]);
				pt.SetColor(lasColor);
				pt.SetIntensity(CloudBlocks[j].points[i].intensity);
				writer.WritePoint(pt);
			}
			ofs.flush();
			ofs.close();
		}
	}
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::readindiceslist(std::vector<int> & indicesA, std::vector<int> & indicesB)
{
	string indiceslistFile;

	cout << "Please enter or drag in the Correspondence Tie Point Indices List File" << endl
		<< "Example [IndicesListFile.txt] :" << endl
		<< "107562 934051 " << endl
		<< "275003 18204" << endl
		<< "872055 462058" << endl
		<< "...  ..." << endl;

	cin >> indiceslistFile;

	ifstream in(indiceslistFile, ios::in);
	if (!in)
	{
		return 0;
	}

	int i = 0;
	while (!in.eof())
	{
		int p1, p2;
		in >> p1 >> p2;
		if (in.fail())
		{
			break;
		}
		indicesA.push_back(p1);
		indicesB.push_back(p2);
		++i;
	}
	in.close();
}

template <typename PointT>
bool DataIo<PointT>::readindiceslist(const typename pcl::PointCloud<PointT>::Ptr &CloudA, const typename pcl::PointCloud<PointT>::Ptr &CloudB, std::vector <std::vector<double> > & coordinatesA, std::vector <std::vector<double> > & coordinatesB)
{
	string indiceslistFile;

	cout << "Please enter or drag in the Correspondence Tie Point Indices List File" << endl
		<< "Example [IndicesListFile.txt] :" << endl
		<< "107562 934051 " << endl
		<< "275003 18204" << endl
		<< "872055 462058" << endl
		<< "...  ..." << endl;

	cin >> indiceslistFile;

	ifstream in(indiceslistFile, ios::in);
	if (!in)
	{
		return 0;
	}

	vector<int> pointlistA;
	vector<int>	pointlistB;

	int i = 0;
	while (!in.eof())
	{
		int p1, p2;
		in >> p1 >> p2;
		if (in.fail())
		{
			break;
		}
		pointlistA.push_back(p1);
		pointlistB.push_back(p2);
		++i;
	}
	in.close();

	for (int j = 0; j < pointlistA.size(); j++)
	{
		std::vector<double> pointA(3);
		pointA[0] = CloudA->points[pointlistA[j]].x;
		pointA[1] = CloudA->points[pointlistA[j]].y;
		pointA[2] = CloudA->points[pointlistA[j]].z;
		coordinatesA.push_back(pointA);
	}

	for (int j = 0; j < pointlistB.size(); j++)
	{
		std::vector<double> pointB(3);
		pointB[0] = CloudB->points[pointlistB[j]].x;
		pointB[1] = CloudB->points[pointlistB[j]].y;
		pointB[2] = CloudB->points[pointlistB[j]].z;
		coordinatesB.push_back(pointB);
	}

	cout << "Procession Done ..." << endl;
}
template <typename PointT>
bool DataIo<PointT>::read_XYZ_XYZlist(std::vector <std::vector<double> > & coordinatesA, std::vector <std::vector<double> > & coordinatesB)
{
	string XYZListFileA, XYZListFileB;

	cout << "Please enter or drag in the Tie Points' XYZ List File of Station A" << endl
		<< "Example [pickinglist_XYZ_A.txt] :" << endl
		<< "11.92,23.07,0.82" << endl
		<< "15.34,18.02,1.25" << endl
		<< "27.01,-7.94,1.37" << endl
		<< "...  ... ..." << endl;

	cin >> XYZListFileA;

	ifstream inA(XYZListFileA, ios::in);
	if (!inA)
	{
		return 0;
	}

	int i = 0;
	while (!inA.eof())
	{
		std::vector<double> Pt(3);
		char comma;
		inA >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
		if (inA.fail())
		{
			break;
		}
		coordinatesA.push_back(Pt);
		++i;
	}
	inA.close();

	cout << "Please enter or drag in the Tie Points' XYZ List File of Station B" << endl;

	cin >> XYZListFileB;

	ifstream inB(XYZListFileB, ios::in);
	if (!inB)
	{
		return 0;
	}

	int j = 0;
	while (!inB.eof())
	{
		std::vector<double> Pt(3);
		char comma;
		inB >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
		if (inB.fail())
		{
			break;
		}
		coordinatesB.push_back(Pt);
		++j;
	}
	inB.close();

	cout << "Procession Done ..." << endl;
}

template <typename PointT>
bool DataIo<PointT>::XYZ_4DOFCSTran(std::vector<double> &transpara)
{
	string XYZListFileA, XYZListFileB;
	std::vector <std::vector<double> >  coordinatesA;
	cout << "Please enter or drag in the XYZ List File of Coordinate System A" << endl
		<< "Example [pointlist_XYZ_A.txt] :" << endl
		<< "11.92,23.07,0.82" << endl
		<< "15.34,18.02,1.25" << endl
		<< "27.01,-7.94,1.37" << endl
		<< "...  ... ..." << endl;

	cin >> XYZListFileA;

	ifstream inA(XYZListFileA, ios::in);
	if (!inA)
	{
		return 0;
	}

	int i = 0;
	while (!inA.eof())
	{
		std::vector<double> Pt(3);
		char comma;
		inA >> Pt[0] >> comma>> Pt[1] >>comma>> Pt[2];
		if (inA.fail())
		{
			break;
		}
		coordinatesA.push_back(Pt);
		++i;
	}
	inA.close();

	cout << "Output the transformed result" << endl;
	XYZListFileB = XYZListFileA.substr(0, XYZListFileA.rfind(".")) + "_utm.txt";
	ofstream ofs;
	ofs.open(XYZListFileB.c_str());
	if (ofs.is_open())
	{
		for (int j = 0; j < i; j++)
		{
			double X_tran = transpara[2] * transpara[4] * coordinatesA[j][0] - transpara[2] * transpara[3] * coordinatesA[j][1] + transpara[0];
			double Y_tran = transpara[2] * transpara[3] * coordinatesA[j][0] + transpara[2] * transpara[4] * coordinatesA[j][1] + transpara[1];
			double Z_tran = coordinatesA[j][2];
			ofs << setiosflags(ios::fixed) << setprecision(8) << X_tran << ","
				<< setiosflags(ios::fixed) << setprecision(8) << Y_tran << ","
				<< setiosflags(ios::fixed) << setprecision(8) << Z_tran << endl;
		}
		ofs.close();
	}
	else{ return 0; }
	
	cout << "Procession Done ..." << endl;
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::XYZ_7DOFCSTran(std::vector<double> &transpara)
{
	string XYZListFileA, XYZListFileB;
	std::vector <std::vector<double> >  coordinatesA;
	cout << "Please enter or drag in the XYZ List File of Coordinate System A" << endl
		<< "Example [pointlist_XYZ_A.txt] :" << endl
		<< "11.92,23.07,0.82" << endl
		<< "15.34,18.02,1.25" << endl
		<< "27.01,-7.94,1.37" << endl
		<< "...  ... ..." << endl;

	cin >> XYZListFileA;

	ifstream inA(XYZListFileA, ios::in);
	if (!inA)
	{
		return 0;
	}

	int i = 0;
	while (!inA.eof())
	{
		std::vector<double> Pt(3);
		char comma;
		inA >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
		if (inA.fail())
		{
			break;
		}
		coordinatesA.push_back(Pt);
		++i;
	}
	inA.close();

	cout << "Output the transformed result" << endl;
	XYZListFileB = XYZListFileA.substr(0, XYZListFileA.rfind(".")) + "_utm.txt";
	ofstream ofs;
	ofs.open(XYZListFileB.c_str());
	if (ofs.is_open())
	{
		for (int j = 0; j < i; j++)
		{
			double X_tran = transpara[0] + transpara[6] * coordinatesA[j][0] + transpara[5] * coordinatesA[j][1] - transpara[4] * coordinatesA[j][2];
			double Y_tran = transpara[1] + transpara[6] * coordinatesA[j][1] - transpara[5] * coordinatesA[j][0] + transpara[3] * coordinatesA[j][2];
			double Z_tran = transpara[2] + transpara[6] * coordinatesA[j][2] + transpara[4] * coordinatesA[j][0] - transpara[3] * coordinatesA[j][1];
			ofs << setiosflags(ios::fixed) << setprecision(8) << X_tran << ","
				<< setiosflags(ios::fixed) << setprecision(8) << Y_tran << ","
				<< setiosflags(ios::fixed) << setprecision(8) << Z_tran << endl;
		}
		ofs.close();
	}
	else{ return 0; }

	cout << "Procession Done ..." << endl;
	return 1;
}

template <typename PointT>
double DataIo<PointT>::cal_cor_RMSE(std::vector <std::vector<double> > & coordinatesA, std::vector <std::vector<double> > & coordinatesB)
{
	int pointnumberA, pointnumberB, pointnumbercheck;
	double squaredist, RMSE;
	double sum_squaredist = 0;
	pointnumberA = coordinatesA.size();
	pointnumberB = coordinatesB.size();
	if (pointnumberA >= pointnumberB) pointnumbercheck = pointnumberB;
	else pointnumbercheck = pointnumberA;
	
	for (int j = 0; j < pointnumbercheck; j++)
	{
		squaredist = (coordinatesA[j][0] - coordinatesB[j][0])*(coordinatesA[j][0] - coordinatesB[j][0]) +
			         (coordinatesA[j][1] - coordinatesB[j][1])*(coordinatesA[j][1] - coordinatesB[j][1]) +
			         (coordinatesA[j][2] - coordinatesB[j][2])*(coordinatesA[j][2] - coordinatesB[j][2]);
		sum_squaredist += squaredist;
	}

	RMSE = sqrt(sum_squaredist / pointnumbercheck);
	cout << "The Estimated Root Mean Square Error is: " << RMSE << endl;
	return RMSE;
}

# if 0
template <typename PointT>
bool DataIo<PointT>::tran_eng2utm(float centerlong_eng_proj)
{
	string XYZENGListFile;
	
	cout << "Please enter or drag in the Points' XYZ List File for Engineering Coordinate System" << endl
		<< "Example [Pointlist_XYZ_ENGCS.txt] :" << endl
		<< "485026.778,3409071.864,474.672" <<endl    
		<< "485182.217,3409201.304,474.314" << endl
		<< "487070.108,3411533.570,471.484" << endl
		<< "... ... ..." << endl;
	
	cin >> XYZENGListFile;

	ifstream inlist(XYZENGListFile, ios::in);
	if (!inlist)
	{
		return 0;
	}

	GeoTransform gt;

	int j = 0;
	while (!inlist.eof())
	{
		std::vector<double> PtENGXYZ(3);
		std::vector<double> PtBLH(3);
		std::vector<double> PtUTMXYZ(3);

		char comma;
		inlist >> PtENGXYZ[0] >> comma >> PtENGXYZ[1] >> comma >> PtENGXYZ[2];
		if (inlist.fail())
		{
			break;
		}

		cout.setf(ios::showpoint);  //��С�����Ⱥ����0��ʾ����;
		cout.precision(12);         //����������ȣ�������Ч����;

		gt.XYZ2BLH_ENG(PtENGXYZ, centerlong_eng_proj, PtBLH);
		cout << PtBLH[0] << " , " << PtBLH[1] << " , " << PtBLH[2] << endl;

		gt.BLH2XYZ_WGS84(PtBLH, PtUTMXYZ);
		cout << PtUTMXYZ[0] << " , " << PtUTMXYZ[1] << " , " << PtUTMXYZ[2] << endl;
		//coordinatesUTM_XYZ.push_back(PtUTM);
		++j;
	}
	inlist.close();

	cout << "Procession Done ..." << endl;
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::tran_wgs2eng(float centerlong_eng_proj, float proj_surface_h_eng)
{
	string XYZWGSListFile;

	cout << "Please enter or drag in the Points' BLH List File for WGS84/CGCS2000" << endl
		<< "Example [Pointlist_BLH_WGS84.txt] :" << endl;
		

	cin >> XYZWGSListFile;

	ifstream inlist(XYZWGSListFile, ios::in);
	if (!inlist)
	{
		return 0;
	}

	GeoTransform gt;

	int j = 0;
	while (!inlist.eof())
	{
		std::vector<double> PtENGXYZ(3);
		std::vector<double> PtBLH(3);
		//std::vector<double> PtUTMXYZ(3);

		char comma;
		inlist >> PtBLH[0] >> comma >> PtBLH[1] >> comma >> PtBLH[2];
		if (inlist.fail())
		{
			break;
		}

		cout.setf(ios::showpoint);  //��С�����Ⱥ����0��ʾ����;
		cout.precision(12);         //����������ȣ�������Ч����;

		gt.BLH2XYZ_CGCS(PtBLH, centerlong_eng_proj,proj_surface_h_eng ,PtENGXYZ);
		cout << PtENGXYZ[0] << " , " << PtENGXYZ[1] << " , " << PtENGXYZ[2] << endl;

		++j;
	}
	inlist.close();

	cout << "Procession Done ..." << endl;
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::read_XYZ_BLHlist(std::vector <std::vector<double> > & coordinatesSC_XYZ, std::vector <std::vector<double> > & coordinatesUTM_XYZ)
{
	string XYZListFileA, BLHListFileB;

	cout << "Please enter or drag in the Tie Points' XYZ List File of Station A" << endl
		<< "Example [pickinglist_XYZ_A.txt] :" << endl
		<< "11.92,23.07,0.82" << endl
		<< "15.34,18.02,1.25" << endl
		<< "27.01,-7.94,1.37" << endl
		<< "... ... ..." << endl;

	cin >> XYZListFileA;

	ifstream inA(XYZListFileA, ios::in);
	if (!inA)
	{
		return 0;
	}

	int i = 0;
	while (!inA.eof())
	{
		std::vector<double> Pt(3);
		char comma;
		inA >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
		if (inA.fail())
		{
			break;
		}
		coordinatesSC_XYZ.push_back(Pt);
		++i;
	}
	inA.close();


	GeoTransform gt;
	int utmzone;

	cout << "Please enter or drag in the Tie Points' WGS84 BLH Coordinates List" << endl
		<< "Example [pickinglist_BLH_WGS84.txt] :" << endl
		<< "30.71418,115.71602,202.1275" << endl
		<< "30.71803,115.71870,208.2477" << endl
		<< "... ... ..." << endl;
	cin >> BLHListFileB;

	ifstream inB(BLHListFileB, ios::in);
	if (!inB)
	{
		return 0;
	}

	int j = 0;
	while (!inB.eof())
	{
		std::vector<double> Pt(3);
		std::vector<double> PtUTM(3);
		char comma;
		inB >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
		if (inB.fail())
		{
			break;
		}
		utmzone = gt.BLH2XYZ_WGS84(Pt, PtUTM);

		cout.setf(ios::showpoint);  //��С�����Ⱥ����0��ʾ����;
		cout.precision(12);         //����������ȣ�������Ч����;

		cout << PtUTM[0] << " , " << PtUTM[1] << " , " << PtUTM[2] << endl;
		coordinatesUTM_XYZ.push_back(PtUTM);
		++j;
	}
	inB.close();

	cout << "Procession Done ..." << endl;
}
# endif

template <typename PointT>
bool DataIo<PointT>::readLasBlock(const string &fileName, int data_type_, int strip_num_, int num_in_strip_, CloudBlock & block)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "Matched Terms are not found." << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	//Data Type
	block.data_type = data_type_;

	//Sequence Number
	block.strip_num = strip_num_;
	block.num_in_strip = num_in_strip_;

	//Bounding box Information 
	block.bound.min_x = header.GetMinX();
	block.bound.min_y = header.GetMinY();
	block.bound.min_z = header.GetMinZ();
	block.bound.max_x = header.GetMaxX();
	block.bound.max_y = header.GetMaxY();
	block.bound.max_z = header.GetMaxZ();
	//Center Point
	block.cp.x = 0.5*(block.bound.min_x + block.bound.max_x);
	block.cp.y = 0.5*(block.bound.min_y + block.bound.max_y);
	block.cp.z = 0.5*(block.bound.min_z + block.bound.max_z);

	return 1;
}

template <typename PointT>
void DataIo<PointT>::batchReadMultiSourceLasBlock(std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files,
	std::vector<std::vector<CloudBlock> > &ALS_strip_blocks, std::vector<CloudBlock> &TLS_blocks, std::vector<CloudBlock> &MLS_blocks, std::vector<CloudBlock> &BPLS_blocks, std::vector<CloudBlock> &All_blocks)
{
	//ALS 
	int ALS_count = 0;
	for (int i = 0; i < ALS_strip_files.size(); i++)
	{
		ALS_strip_blocks[i].resize(ALS_strip_files[i].size());
		for (int j = 0; j < ALS_strip_files[i].size(); j++)
		{
			readLasBlock(ALS_strip_files[i][j], 1, i, j, ALS_strip_blocks[i][j]);
			ALS_strip_blocks[i][j].unique_index = ALS_count;  //Set Block Unique Index
			All_blocks.push_back(ALS_strip_blocks[i][j]);
			ALS_count++;
		}
	}
	cout << "ALS boxes import done ..." << endl;

	//TLS
	for (int i = 0; i < TLS_files.size(); i++)
	{
		readLasBlock(TLS_files[i], 2, 0, i, TLS_blocks[i]);
		TLS_blocks[i].unique_index = ALS_count + i;  //Set Block Unique Index
		All_blocks.push_back(TLS_blocks[i]);
	}
	cout << "TLS boxes import done ..." << endl;

	//MLS
	for (int i = 0; i < MLS_files.size(); i++)
	{
		readLasBlock(MLS_files[i], 3, 0, i, MLS_blocks[i]);
		MLS_blocks[i].unique_index = ALS_count + TLS_files.size() + i;  //Set Block Unique Index
		All_blocks.push_back(MLS_blocks[i]);
	}
	cout << "MLS boxes import done ..." << endl;

	//BPLS
	for (int i = 0; i < BPLS_files.size(); i++)
	{
		readLasBlock(BPLS_files[i], 4, 0, i, BPLS_blocks[i]);
		BPLS_blocks[i].unique_index = ALS_count + TLS_files.size() + MLS_files.size() + i;  //Set Block Unique Index
		All_blocks.push_back(BPLS_blocks[i]);
	}
	cout << "BPLS boxes import done ..." << endl;

	cout << "!----------------------------------------------------------------------------!" << endl;
	cout << "The number of all the nodes : " << All_blocks.size() << endl;
	cout << "ALS: " << ALS_count << " blocks in " << ALS_strip_files.size() << " strips" << endl;
	cout << "TLS: " << TLS_files.size() << " stations" << endl;
	cout << "MLS: " << MLS_files.size() << " blocks" << endl;
	cout << "BPLS: " << BPLS_files.size() << " blocks" << endl;
	cout << "!----------------------------------------------------------------------------!" << endl;
}

template <typename PointT>
void DataIo<PointT>::pcXYZ2XY(const typename pcl::PointCloud<PointT>::Ptr &pointcloudxyz, pcl::PointCloud<pcl::PointXY>::Ptr &pointcloudxy)
{
	for (size_t i = 0; i < pointcloudxyz->size(); i++)
	{
		pcl::PointXY ptXY;
		ptXY.x = pointcloudxyz->points[i].x;
		ptXY.y = pointcloudxyz->points[i].y;
		pointcloudxy->push_back(ptXY);
	}
}

template <typename PointT>
void DataIo<PointT>::display2Dboxes(const vector<CloudBlock> &blocks)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Boxes Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	float sphere_size, f_red, f_green, f_blue, line_width;

	sphere_size = 1.5;
	line_width = 1;

	for (int i = 0; i < blocks.size(); i++)
	{
		switch (blocks[i].data_type)
		{
		case 1: //ALS
			f_red = 1.0;
			f_green = 0.0;
			f_blue = 0.0;
			break;
		case 2: //TLS
			f_red = 0.0;
			f_green = 1.0;
			f_blue = 0.0;
			break;
		case 3: //MLS
			f_red = 0.0;
			f_green = 0.0;
			f_blue = 1.0;
			break;
		case 4: //BPLS
			f_red = 1.0;
			f_green = 1.0;
			f_blue = 0.0;
			break;
		default:
			break;
		}

		pcl::PointXYZ pt1;
		pt1.x = blocks[i].bound.min_x;
		pt1.y = blocks[i].bound.min_y;
		pt1.z = 0;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(pt1, sphere_size, f_red, f_green, f_blue, s);
		n++;

		pcl::PointXYZ pt2;
		pt2.x = blocks[i].bound.max_x;
		pt2.y = blocks[i].bound.min_y;
		pt2.z = 0;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(pt2, sphere_size, f_red, f_green, f_blue, s);
		n++;

		pcl::PointXYZ pt3;
		pt3.x = blocks[i].bound.max_x;
		pt3.y = blocks[i].bound.max_y;
		pt3.z = 0;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(pt3, sphere_size, f_red, f_green, f_blue, s);
		n++;

		pcl::PointXYZ pt4;
		pt4.x = blocks[i].bound.min_x;
		pt4.y = blocks[i].bound.max_y;
		pt4.z = 0;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(pt4, sphere_size, f_red, f_green, f_blue, s);
		n++;

		pcl::PointXYZ ptc;
		ptc.x = blocks[i].cp.x;
		ptc.y = blocks[i].cp.y;
		ptc.z = 0;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(ptc, sphere_size, f_red, f_green, f_blue, s);
		n++;

		sprintf(t, "%d", n);
		s = t;
		viewer->addLine(pt1, pt2, f_red, f_green, f_blue, s);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
		n++;

		sprintf(t, "%d", n);
		s = t;
		viewer->addLine(pt2, pt3, f_red, f_green, f_blue, s);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
		n++;

		sprintf(t, "%d", n);
		s = t;
		viewer->addLine(pt3, pt4, f_red, f_green, f_blue, s);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
		n++;

		sprintf(t, "%d", n);
		s = t;
		viewer->addLine(pt4, pt1, f_red, f_green, f_blue, s);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
		n++;

	}

	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

template <typename PointT>
void DataIo<PointT>::display2Dcons(const vector<Constraint> &cons)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Graph Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	float sphere_size, f_red, f_green, f_blue, line_width;

	sphere_size = 35.0;
	line_width = 1.0;

	for (int i = 0; i < cons.size(); i++)
	{
		switch (cons[i].block1.data_type)
		{
		case 1: //ALS
			f_red = 1.0;
			f_green = 0.0;
			f_blue = 0.0;
			break;
		case 2: //TLS
			f_red = 0.0;
			f_green = 1.0;
			f_blue = 0.0;
			break;
		case 3: //MLS
			f_red = 0.0;
			f_green = 0.0;
			f_blue = 1.0;
			break;
		case 4: //BPLS
			f_red = 1.0;
			f_green = 1.0;
			f_blue = 0.0;
			break;
		default:
			break;
		}

		pcl::PointXYZ ptc1;
		ptc1.x = cons[i].block1.cp.x;
		ptc1.y = cons[i].block1.cp.y;
		ptc1.z = 0;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(ptc1, sphere_size, f_red, f_green, f_blue, s);
		n++;

		switch (cons[i].block2.data_type)
		{
		case 1: //ALS
			f_red = 1.0;
			f_green = 0.0;
			f_blue = 0.0;
			break;
		case 2: //TLS
			f_red = 0.0;
			f_green = 1.0;
			f_blue = 0.0;
			break;
		case 3: //MLS
			f_red = 0.0;
			f_green = 0.0;
			f_blue = 1.0;
			break;
		case 4: //BPLS
			f_red = 1.0;
			f_green = 1.0;
			f_blue = 0.0;
			break;
		default:
			break;
		}

		pcl::PointXYZ ptc2;
		ptc2.x = cons[i].block2.cp.x;
		ptc2.y = cons[i].block2.cp.y;
		ptc2.z = 0;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(ptc2, sphere_size, f_red, f_green, f_blue, s);
		n++;

		switch (cons[i].con_type)
		{
		case 1: //Adjacent
			f_red = 0.0;
			f_green = 1.0;
			f_blue = 1.0;
			break;
		case 2: //Registration
			f_red = 1.0;
			f_green = 0.0;
			f_blue = 1.0;
			break;
		default:
			break;
		}
		sprintf(t, "%d", n);
		s = t;
		viewer->addLine(ptc1, ptc2, f_red, f_green, f_blue, s);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
		n++;
	}

	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

template <typename PointT>
bool DataIo<PointT>::lasfileGK2UTM(const string &fileName)
{
	const int drift = 500000;
	const double utm_scale = 0.9996;
	
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "Matched Terms are not found." << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();
	double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax, Xmin_, Ymin_, Xmax_, Ymax_;;
	Xmin = header.GetMinX();
	Ymin = header.GetMinY();
	Zmin = header.GetMinZ();
	Xmax = header.GetMaxX();
	Ymax = header.GetMaxY();
	Zmax = header.GetMaxZ();
	Xmin_ = (Xmin - drift)*utm_scale + drift;
	Xmax_ = (Xmax - drift)*utm_scale + drift;
	Ymin_ = Ymin*utm_scale;
	Ymax_ = Ymax*utm_scale;

	string fileNameout = fileName.substr(0, fileName.rfind(".")) + "_utm.las";

	ofstream ofs;
	ofs.open(fileNameout.c_str(), std::ios::out | std::ios::binary);

	if (ofs.is_open())
	{
		liblas::Header headerout;
		headerout.SetDataFormatId(liblas::ePointFormat2);
		headerout.SetVersionMajor(1);
		headerout.SetVersionMinor(2);
		

		headerout.SetMin(Xmin_, Ymin_, Zmin );
		headerout.SetMax(Xmax_ , Ymax_, Zmax );
		headerout.SetOffset((Xmin_ + Xmax_) / 2.0 , (Ymin_ + Ymax_) / 2.0 , (Zmin + Zmax) / 2.0);
		headerout.SetScale(0.01, 0.01, 0.01);
		headerout.SetPointRecordsCount(header.GetPointRecordsCount());

		liblas::Writer writer(ofs, headerout);

		while (reader.ReadNextPoint())
		{
			liblas::Point p = reader.GetPoint();
			liblas::Point pt(&headerout);
			double pX, pY, pZ;
			pX = (p.GetX()-drift)*utm_scale+drift;
			pY = p.GetY()*utm_scale;
			pZ = p.GetZ();
			pt.SetCoordinates(pX, pY,pZ);
			pt.SetIntensity(p.GetIntensity());
			pt.SetFlightLineEdge(p.GetFlightLineEdge());
			pt.SetNumberOfReturns(p.GetNumberOfReturns());
			//pt.SetTime(p.GetTime());
			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}
	return 1;

}


template <typename PointT>
bool DataIo<PointT>::lasfileshift(const string &fileName, vector<double> &shift)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "Matched Terms are not found." << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();
	double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
	Xmin = header.GetMinX();
	Ymin = header.GetMinY();
	Zmin = header.GetMinZ();
	Xmax = header.GetMaxX();
	Ymax = header.GetMaxY();
	Zmax = header.GetMaxZ();

	string fileNameout = fileName.substr(0, fileName.rfind(".")) + "_t.las";  
	//find ��ǰ�������;
	//rfind �Ӻ���ǰ����;

	ofstream ofs;
	ofs.open(fileNameout.c_str(), std::ios::out | std::ios::binary);

	if (ofs.is_open())
	{
		liblas::Header headerout;
		headerout.SetDataFormatId(liblas::ePointFormat2);
		headerout.SetVersionMajor(1);
		headerout.SetVersionMinor(2);
		headerout.SetMin(Xmin + shift[0], Ymin + shift[1], Zmin + shift[2]);
		headerout.SetMax(Xmax + shift[0], Ymax + shift[1], Zmax + shift[2]);
		headerout.SetOffset((Xmin + Xmax) / 2.0 + shift[0], (Ymin + Ymax) / 2.0 + shift[1], (Zmin + Zmax) / 2.0 + shift[2]);
		headerout.SetScale(0.01, 0.01, 0.01);
		headerout.SetPointRecordsCount(header.GetPointRecordsCount());

		liblas::Writer writer(ofs, headerout);

		while (reader.ReadNextPoint())
		{
			liblas::Point p = reader.GetPoint();
			liblas::Point pt(&headerout);
			double pX, pY, pZ;
			pX = p.GetX();
			pY = p.GetY();
			pZ = p.GetZ();
			pt.SetCoordinates(pX + shift[0], pY + shift[1], pZ + shift[2]);
			pt.SetIntensity(p.GetIntensity());
			pt.SetFlightLineEdge(p.GetFlightLineEdge());
			pt.SetNumberOfReturns(p.GetNumberOfReturns());
			//pt.SetTime(p.GetTime());
			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}
	return 1;

}

template <typename PointT>
void DataIo<PointT>::readLasCloudPairfromCon(const Constraint &this_con, std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files,
	string &Filename1, string &Filename2, typename pcl::PointCloud<PointT>::Ptr &cloud1, typename pcl::PointCloud<PointT>::Ptr &cloud2)
{
	//Find the file index
	switch (this_con.block1.data_type)
	{
	case 1: //ALS
		Filename1 = ALS_strip_files[this_con.block1.strip_num][this_con.block1.num_in_strip];
		break;
	case 2: //TLS
		Filename1 = TLS_files[this_con.block1.num_in_strip];
		break;
	case 3: //MLS
		Filename1 = MLS_files[this_con.block1.num_in_strip];
		break;
	case 4: //BPLS
		Filename1 = BPLS_files[this_con.block1.num_in_strip];
		break;
	default:
		break;
	}
	switch (this_con.block2.data_type)
	{
	case 1: //ALS
		Filename2 = ALS_strip_files[this_con.block2.strip_num][this_con.block2.num_in_strip];
		break;
	case 2: //TLS
		Filename2 = TLS_files[this_con.block2.num_in_strip];
		break;
	case 3: //MLS
		Filename2 = MLS_files[this_con.block2.num_in_strip];
		break;
	case 4: //BPLS
		Filename2 = BPLS_files[this_con.block2.num_in_strip];
		break;
	default:
		break;
	}

	//Import the point clouds for registration  // ÿ��ѭ����һ�εĵ��ƴ洢�ռ�ᱻ�ͷŵ�;
	// Cloud1:T  Cloud2:S  Trans12=inv(Trans21)=inv(RegTrans)  
	readLasFile(Filename1, cloud1, 1);
	LOG(INFO) << "Read File " << this_con.block1.data_type << "-" << this_con.block1.strip_num << "-" << this_con.block1.num_in_strip << " Done...";
	LOG(INFO) << "Filename: " << Filename1;
	readLasFileLast(Filename2, cloud2);
	LOG(INFO) << "Read File " << this_con.block2.data_type << "-" << this_con.block2.strip_num << "-" << this_con.block2.num_in_strip << " Done...";
	LOG(INFO) << "Filename: " << Filename2;
	LOG(INFO) << "Raw point number: [ " << cloud1->size() << "  ,  " << cloud2->size() << "  ]" << endl;
}

template <typename PointT>
void DataIo<PointT>::batchdownsamplepair(const Constraint &this_con, typename pcl::PointCloud<PointT>::Ptr &cloud1, typename pcl::PointCloud<PointT>::Ptr &cloud2, typename pcl::PointCloud<PointT>::Ptr &subcloud1, typename pcl::PointCloud<PointT>::Ptr &subcloud2,
	float ALS_radius, float TLS_radius, float MLS_radius, float BPLS_radius)
{
	VoxelFilter<pcl::PointXYZ> vf_als(ALS_radius);
	VoxelFilter<pcl::PointXYZ> vf_tls(TLS_radius);
	VoxelFilter<pcl::PointXYZ> vf_mls(MLS_radius);
	VoxelFilter<pcl::PointXYZ> vf_bpls(BPLS_radius);

	//Down-sampling
	switch (this_con.block1.data_type)
	{
	case 1: //ALS
		subcloud1 = vf_als.filter(cloud1);
		break;
	case 2: //TLS
		subcloud1 = vf_tls.filter(cloud1);
		break;
	case 3: //MLS
		subcloud1 = vf_mls.filter(cloud1);
		break;
	case 4: //BPLS
		subcloud1 = vf_bpls.filter(cloud1);
		break;
	default:
		break;
	}
	switch (this_con.block2.data_type)
	{
	case 1: //ALS
		subcloud2 = vf_als.filter(cloud2);
		break;
	case 2: //TLS
		subcloud2 = vf_tls.filter(cloud2);
		break;
	case 3: //MLS
		subcloud2 = vf_mls.filter(cloud2);
		break;
	case 4: //BPLS
		subcloud2 = vf_bpls.filter(cloud2);
		break;
	default:
		break;
	}

	LOG(INFO) << "Down-sampled point cloud number: [ " << subcloud1->size() << "  ,  " << subcloud2->size() << "  ]" << endl;
}

template <typename PointT>
void DataIo<PointT>::batchwritefinalcloud(vector<CloudBlock> &All_blocks, std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files)
{
	for (vector<CloudBlock>::iterator iter = All_blocks.begin(); iter != All_blocks.end(); iter++)
	{
		string Filenamein, Folderout, Filenameout;
		switch ((*iter).data_type)
		{
		case 1: //ALS
			Filenamein = ALS_strip_files[(*iter).strip_num][(*iter).num_in_strip];
			break;
		case 2: //TLS
			Filenamein = TLS_files[(*iter).num_in_strip];
			break;
		case 3: //MLS
			Filenamein = MLS_files[(*iter).num_in_strip];
			break;
		case 4: //BPLS
			Filenamein = BPLS_files[(*iter).num_in_strip];
			break;
		default:
			break;
		}

		Folderout = Filenamein.substr(0, Filenamein.rfind("\\")) + "\\Output";

		if (!boost::filesystem::exists(Folderout))
		{
			boost::filesystem::create_directory(Folderout);
		}

		Filenameout = Folderout + Filenamein.substr(Filenamein.rfind("\\"), Filenamein.rfind(".") - Filenamein.rfind("\\")) + "_refine_out.las";
		
		typename pcl::PointCloud<PointT>::Ptr cloudin(new pcl::PointCloud<PointT>()), cloudout(new pcl::PointCloud<PointT>());
		readLasFile(Filenamein, cloudin, 1);
		CRegistration <PointT> regx;
		//Eigen::Matrix4f corrected_pose;
		//regx.invTransform((*iter).optimized_pose, corrected_pose);
		regx.transformcloud(cloudin, cloudout, (*iter).optimized_pose);
		writeLasFile(Filenameout, cloudout, 1);
		LOG(INFO) << "Output Done for cloud with index " << (*iter).unique_index;
		LOG(INFO) << "Its position is " << Filenameout;
		//λ�˽��Ӧ�÷�����; ok
		//�����ʱ���drift������; not ok yet
	}
}