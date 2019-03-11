//
// This file is used for the Reading, Writing and Displaying of Point Cloud of various formats.
// Dependent 3rd Libs: PCL (>1.7)  liblas
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#include "dataio.h"
#include "utility.h"

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>


using namespace  std;
using namespace  utility;

template <typename PointT>
bool DataIo<PointT>::readCloudFile(const int pc_format, const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	switch (pc_format)
	{
	case 1:
		readPcdFile(fileName, pointCloud);
		break;
	case 2:
		readLasFile(fileName, pointCloud);
		break;
	case 3:
		readPlyFile(fileName, pointCloud);
		break;
	default:
		cout << "Undefined Point Cloud Format." << endl;
		return 0;
		break;
	}
}

template <typename PointT>
bool DataIo<PointT>::writeCloudFile(const int pc_format, const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
	string outputfileName;
	ostringstream oss;
	switch (pc_format)
	{
	case 1:	
		oss << fileName << ".pcd";
		outputfileName =  oss.str();
		writePcdFile(outputfileName, pointCloud);
		break;
	case 2:
		oss << fileName << ".las";
		outputfileName = oss.str();
		writeLasFile(outputfileName, pointCloud);
		break;
	case 3:
		oss << fileName << ".ply";
		outputfileName = oss.str();
		writePlyFile(outputfileName, pointCloud);
		break;
	default:
		cout << "Undefined Point Cloud Format." << endl;
		return 0;
		break;
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
	cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;
	
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
		pt.intensity = p.GetIntensity();
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
			pt.SetIntensity(pointCloud->points[i].intensity);
			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}
	return 1;
}

template <typename PointT>
bool DataIo<PointT>::readLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, double & X_origin, double  & Y_origin)  //With translation
{
	cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;

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
		PointT pt;
		
		//A translation to keep the precision
		//做一个平移，否则在UTM WGS84下的点坐标太大了，会造成精度损失的. 因为las的读取点数据是double的，而pcd是float的;
		pt.x = p.GetX() - Xmin;
		pt.y = p.GetY() - Ymin;
		pt.z = p.GetZ();
		pt.intensity = p.GetIntensity();
		pointCloud->points.push_back(pt);
	}

	// Origin coordinate;
	X_min = Xmin;
	Y_min = Ymin;

	return 1;
}

template <typename PointT>
bool DataIo<PointT>::writeLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, double X_origin, double Y_origin) //With translation
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
		header.SetMin(bound.min_x + X_origin, bound.min_y + Y_origin, bound.min_z);
		header.SetMax(bound.max_x + X_origin, bound.max_y + Y_origin, bound.max_z);
		header.SetOffset((bound.min_x + bound.max_x) / 2.0 + X_origin, (bound.min_y + bound.max_y) / 2.0 + Y_origin, (bound.min_z + bound.max_z) / 2.0);
		header.SetScale(0.01, 0.01, 0.01);
		header.SetPointRecordsCount(pointCloud->points.size());

		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (int i = 0; i < pointCloud->points.size(); i++)
		{
			pt.SetCoordinates(double(pointCloud->points[i].x) + X_origin, double(pointCloud->points[i].y) + Y_origin, double(pointCloud->points[i].z));
			pt.SetIntensity(pointCloud->points[i].intensity);
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