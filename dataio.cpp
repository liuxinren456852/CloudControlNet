//
// This file is used for the Reading, Writing and Displaying of Point Cloud of various formats.
// Dependent 3rd Libs: PCL (>1.7)  liblas  VTK
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#include "dataio.h"
#include "utility.h"
#include "GeoTran.h"

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <fstream>
#include <vector>


using namespace  std;
using namespace  utility;
using namespace  boost::filesystem;

template <typename PointT>
bool DataIo<PointT>::readCloudFile(const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	string extension;
	extension = fileName.substr(fileName.find_last_of('.') + 1);     //Get the suffix of the file;
	
	if (!strcmp(extension.c_str(), "pcd"))
	{
		readPcdFile(fileName, pointCloud);
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
	}
	else if (!strcmp(extension.c_str(), "ply"))
	{
		readPlyFile(fileName, pointCloud);
	}		
	else if (!strcmp(extension.c_str(), "txt"))
	{
		readTxtFile(fileName, pointCloud);
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
	}
	else if (!strcmp(extension.c_str(), "ply"))
	{
		writePlyFile(fileName, pointCloud);
	}
	else if (!strcmp(extension.c_str(), "txt"))
	{
		writeTxtFile(fileName, pointCloud);
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
		ifs.open(fileName, std::ios::in | std::ios::binary);
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
	ifs.open(fileName, std::ios::in | std::ios::binary);
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
	ofs.open(fileName, std::ios::out | std::ios::binary);
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
			pt.SetIntensity(pointCloud->points[i].intensity);

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
	ifs.open(fileName, std::ios::in | std::ios::binary);
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

		cout << "A txt File named GlobalShift.txt is saved in current Folder" << endl;
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
		//做一个平移，否则在UTM WGS84下的点坐标太大了，会造成精度损失的. 因为las的读取点数据是double的，而pcd是float的;
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

	ofstream ofs;
	ofs.open(fileName, std::ios::out | std::ios::binary);
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
			pt.SetIntensity(pointCloud->points[i].intensity);

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
bool DataIo<PointT>::batchReadFileNamesInSubFolders(const std::string &folderName, const std::string & extension, std::vector<std::vector<std::string>> &fileNames)
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
	ofs.open(fileName);
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
    ofs.open(fileName);
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
void DataIo<PointT>::ALS_block_by_time(const typename pcl::PointCloud<PointT>::Ptr &pointCloud, typename vector<pcl::PointCloud<PointT>> &CloudBlocks, float time_step_in_second)
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
bool DataIo<PointT>::batchWriteBlockInColor(const string &fileName, typename vector<pcl::PointCloud<PointT>> &CloudBlocks, bool automatic_shift_or_not)
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

		string BlockFilename;
		ostringstream oss;
		oss << j.tostring("000") << "_" << fileName; // 此处存疑，按000，001，002这样的顺序命名;
		BlockFilename = oss.str();

		ofstream ofs;
		ofs.open(BlockFilename, std::ios::out | std::ios::binary);
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
bool DataIo<PointT>::readindiceslist(const typename pcl::PointCloud<PointT>::Ptr &CloudA, const typename pcl::PointCloud<PointT>::Ptr &CloudB, std::vector <std::vector<double>> & coordinatesA, std::vector <std::vector<double>> & coordinatesB)
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
bool DataIo<PointT>::read_XYZ_XYZlist(std::vector <std::vector<double>> & coordinatesA, std::vector <std::vector<double>> & coordinatesB)
{
	string XYZListFileA, XYZListFileB;

	cout << "Please enter or drag in the Tie Points' XYZ List File of Station A" << endl
		<< "Example [pickinglist_XYZ_A.txt] :" << endl
		<< "11.92 23.07 0.82" << endl
		<< "15.34 18.02 1.25" << endl
		<< "27.01 -7.94 1.37" << endl
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
bool DataIo<PointT>::read_XYZ_BLHlist(std::vector <std::vector<double>> & coordinatesSC_XYZ, std::vector <std::vector<double>> & coordinatesUTM_XYZ)
{
	string XYZListFileA, BLHListFileB;

	cout << "Please enter or drag in the Tie Points' XYZ List File of Station A" << endl
		<< "Example [pickinglist_XYZ_A.txt] :" << endl
		<< "11.92 23.07 0.82" << endl
		<< "15.34 18.02 1.25" << endl
		<< "27.01 -7.94 1.37" << endl
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
		coordinatesSC_XYZ.push_back(Pt);
		++i;
	}
	inA.close();


	GeoTransform gt;
	int utmzone;

	cout << "Please enter or drag in the Tie Points' WGS84 BLH Coordinates List" << endl
		<< "Example [pickinglist_BLH_WGS84.txt] :" << endl
		<< "30.71418 115.71602 202.1275" << endl
		<< "30.71803 115.71870 208.2477" << endl
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

		cout.setf(ios::showpoint);  //将小数精度后面的0显示出来;
		cout.precision(12);         //设置输出精度，保留有效数字;

		cout << PtUTM[0] << " , " << PtUTM[1] << " , " << PtUTM[2] << endl;
		coordinatesUTM_XYZ.push_back(PtUTM);
		++j;
	}
	inB.close();

	cout << "Procession Done ..." << endl;
}

template <typename PointT>
bool DataIo<PointT>::readLasBlock(const string &fileName, int data_type_, int strip_num_, int num_in_strip_, CloudBlock & block)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
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
void DataIo<PointT>::pcXYZ2XY(const typename pcl::PointCloud<PointT>::Ptr &pointcloudxyz, pcl::PointCloud<pcl::PointXY>::Ptr &pointcloudxy)
{
	for (size_t i = 0; i < pointcloudxyz->size(); i++)
	{
		pxl::PointXY ptXY;
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
bool DataIo<PointT>::lasfileshift(const string &fileName, vector<double> &shift)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
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

	ofstream ofs;
	ofs.open(fileNameout, std::ios::out | std::ios::binary);

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