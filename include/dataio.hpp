#ifndef _INCLUDE_DATA_IO_HPP
#define _INCLUDE_DATA_IO_HPP

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

//liblas
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

#include <string>
#include <fstream>
#include <vector>

#include "utility.h"
#include "find_constraint.h"
#include <glog/logging.h>

using namespace boost::filesystem;
using namespace std;

namespace ccn
{

template <typename PointT>
class DataIo : public CloudUtility<PointT>
{
  public:
    bool readCloudFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::string extension;
        extension = fileName.substr(fileName.find_last_of('.') + 1); //Get the suffix of the file;

        if (!strcmp(extension.c_str(), "pcd"))
        {
            readPcdFile(fileName, pointCloud);
            std::cout << "A pcd file has been imported" << std::endl;
        }
        else if (!strcmp(extension.c_str(), "las"))
        {
            bool global_shift_or_not = 0;
            std::cout << "Would you like to do a global shift ?  0. No  1. Yes [default 0]" << std::endl;
            std::cin >> global_shift_or_not;
            if (!global_shift_or_not)
            {
                readLasFile(fileName, pointCloud);
            }
            else
            {
                bool use_automatic_drift = 0;
                std::cout << "Using the automatic shift or enter the global shift yourself ? " << std::endl
                          << "0. Read a global shift file  1.Use the automatic shift [default 0]" << std::endl;
                std::cin >> use_automatic_drift;
                readLasFile(fileName, pointCloud, use_automatic_drift);
            }
            std::cout << "A las file has been imported" << std::endl;
        }
        else if (!strcmp(extension.c_str(), "ply"))
        {
            readPlyFile(fileName, pointCloud);
            std::cout << "A ply file has been imported" << std::endl;
        }
        else if (!strcmp(extension.c_str(), "txt"))
        {
            readTxtFile(fileName, pointCloud);
            std::cout << "A txt file has been imported" << std::endl;
        }
        else
        {
            std::cout << "Undefined Point Cloud Format." << std::endl;
            return 0;
        }

        std::cout << "Data loaded (" << pointCloud->points.size() << " points)" << std::endl;
    }

    bool writeCloudFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::string extension;
        extension = fileName.substr(fileName.find_last_of('.') + 1); //Get the suffix of the file;

        if (!strcmp(extension.c_str(), "pcd"))
        {
            writePcdFile(fileName, pointCloud);
            std::cout << "A pcd file has been exported" << std::endl;
        }
        else if (!strcmp(extension.c_str(), "las"))
        {
            bool global_shift_or_not = 0;
            std::cout << "Would you like to do a global shift ?  0. No  1. Yes [default 0]" << std::endl;
            std::cin >> global_shift_or_not;
            if (!global_shift_or_not)
            {
                writeLasFile(fileName, pointCloud);
            }
            else
            {
                bool use_automatic_drift = 0;
                std::cout << "Using the automatic shift or enter the global shift yourself ? " << std::endl
                          << "0. Read a global shift file  1.Use the automatic shift [default 0]" << std::endl;
                std::cin >> use_automatic_drift;
                writeLasFile(fileName, pointCloud, use_automatic_drift);
            }
            std::cout << "A las file has been exported" << std::endl;
        }
        else if (!strcmp(extension.c_str(), "ply"))
        {
            writePlyFile(fileName, pointCloud);
            std::cout << "A ply file has been exported" << std::endl;
        }
        else if (!strcmp(extension.c_str(), "txt"))
        {
            writeTxtFile(fileName, pointCloud);
            std::cout << "A txt file has been exported" << std::endl;
        }
        else
        {
            std::cout << "Undefined Point Cloud Format." << std::endl;
            return 0;
        }
    }

    bool readPcdFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (pcl::io::loadPCDFile<PointT>(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't read file\n");
            return false;
        }
        return true;
    }

    bool writePcdFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't write file\n");
            return false;
        }
        return true;
    }

    bool readLasFileHeader(const std::string &fileName, liblas::Header &header)
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

    bool readLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud) //Without translation
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
            std::cout << "Matched Terms are not found." << std::endl;
        }
        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header const &header = reader.GetHeader();

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
            const liblas::Point &p = reader.GetPoint();
            PointT pt;
            pt.x = p.GetX();
            pt.y = p.GetY();
            pt.z = p.GetZ();

            //------------------------------------------------Assign Intensity--------------------------------------------------//
            //If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
            // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
            // if (intensity_available)
            // {
            //     pt.intensity = p.GetIntensity();
            // }

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

    bool writeLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud) //Without translation
    {

        Bounds bound;
        this->getCloudBound(*pointCloud, bound);

        std::ofstream ofs;
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

                // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
                // if (intensity_available)
                // {
                //     pt.SetIntensity(pointCloud->points[i].intensity);
                // }

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

    bool readLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not) //With translation
    {
        global_shift.resize(3);
        //std::cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;

        if (fileName.substr(fileName.rfind('.')).compare(".las"))
        {
            return 0;
        }

        std::ifstream ifs;
        ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
        if (ifs.bad())
        {
            std::cout << "Matched Terms are not found." << std::endl;
        }
        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header const &header = reader.GetHeader();

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

            std::ofstream out("GlobalShift.txt", std::ios::out);
            out << setiosflags(std::ios::fixed) << std::setprecision(8) << global_shift[0] << std::endl;
            out << setiosflags(std::ios::fixed) << std::setprecision(8) << global_shift[1] << std::endl;
            out << setiosflags(std::ios::fixed) << std::setprecision(8) << global_shift[2] << std::endl;
            out.close();

            std::cout << "A txt File named GlobalShift.txt is saved in current Folder" << std::endl;
        }
        else
        {
            std::string fileGlobalShift;
            std::cout << "Please enter or drag in the Global Shift File" << std::endl
                      << "Example [GlobalShift.txt] :" << std::endl
                      << "-366370.90" << std::endl
                      << "-3451297.82" << std::endl
                      << "-14.29" << std::endl;

            std::cin >> fileGlobalShift;

            std::ifstream in(fileGlobalShift.c_str(), std::ios::in);
            in >> global_shift[0];
            in >> global_shift[1];
            in >> global_shift[2];
            in.close();
        }

        while (reader.ReadNextPoint())
        {
            const liblas::Point &p = reader.GetPoint();
            PointT pt;

            //A translation to keep the precision
            pt.x = p.GetX() + global_shift[0];
            pt.y = p.GetY() + global_shift[1];
            pt.z = p.GetZ() + global_shift[2];

            //------------------------------------------------Assign Intensity--------------------------------------------------//
            //If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
            //If the Point template PointT is without intensity, you should comment the line.
            // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
            // if (intensity_available)
            // {
            //     pt.intensity = p.GetIntensity();
            // }

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

    bool writeLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not) //With translation
    {
        global_shift.resize(3);

        Bounds bound;
        this->getCloudBound(*pointCloud, bound);

        if (!automatic_shift_or_not)
        {
            std::cout << "Use default (last input) global shift or not" << std::endl
                      << "1. Yes  0. No" << std::endl;
            bool use_last_shift;
            std::cin >> use_last_shift;
            if (!use_last_shift)
            {
                std::string fileGlobalShift;
                std::cout << "Please enter or drag in the Global Shift File" << std::endl
                          << "Example [GlobalShift.txt] :" << std::endl
                          << "-366370.90" << std::endl
                          << "-3451297.82" << std::endl
                          << "-14.29" << std::endl;

                std::cin >> fileGlobalShift;

                std::ifstream in(fileGlobalShift, std::ios::in);
                in >> global_shift[0];
                in >> global_shift[1];
                in >> global_shift[2];
                in.close();
            }
        }

        std::ofstream ofs;
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

                bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
                
                // figure out why this cannot be used properly.
                // if (intensity_available)
                // {
                //     pt.SetIntensity(pointCloud->points[i].intensity);
                // }

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

    bool readLasFileLast(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (fileName.substr(fileName.rfind('.')).compare(".las"))
        {
            return 0;
        }

        std::ifstream ifs;
        ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
        if (ifs.bad())
        {
            std::cout << "Matched Terms are not found." << std::endl;
        }
        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header const &header = reader.GetHeader();

        while (reader.ReadNextPoint())
        {
            const liblas::Point &p = reader.GetPoint();
            PointT pt;

            //A translation to keep the precision
            pt.x = p.GetX() + global_shift[0];
            pt.y = p.GetY() + global_shift[1];
            pt.z = p.GetZ() + global_shift[2];

            //------------------------------------------------Assign Intensity--------------------------------------------------//
            //If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
            // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
            // if (intensity_available)
            // {
            //     pt.intensity = p.GetIntensity();
            // }
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

    bool readPlyFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (pcl::io::loadPLYFile<PointT>(fileName, *pointCloud) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file \n");
            return (-1);
        }
    }

    bool writePlyFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (pcl::io::savePLYFile<PointT>(fileName, *pointCloud) == -1) //* load the file
        {
            PCL_ERROR("Couldn't write file \n");
            return (-1);
        }
    }

    bool readTxtFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::ifstream in(fileName.c_str(), std::ios::in);
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
        //std::cout << "Import finished ... ..." << std::endl;
        return 1;
    }

    bool writeTxtFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::ofstream ofs;
        ofs.open(fileName.c_str());
        if (ofs.is_open())
        {
            for (size_t i = 0; i < pointCloud->size(); ++i)
            {
                ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].x << "  "
                    << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].y << "  "
                    << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].z
                    //<<"  "<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].intensity
                    << std::endl;
            }
            ofs.close();
        }
        else
        {
            return 0;
        }
        //std::cout << "Output finished ... ..." << std::endl;
        return 1;
    }

    bool writeTxtFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, int subsample_ratio)
    {
        std::ofstream ofs;
        ofs.open(fileName.c_str());
        if (ofs.is_open())
        {
            for (size_t i = 0; i < pointCloud->size(); ++i)
            {
                if (i % subsample_ratio == 0) //Subsampling;
                {
                    ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].x << "  "
                        << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].y << "  "
                        << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].z
                        //<<"  "<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].intensity
                        << std::endl;
                }
            }
            ofs.close();
        }
        else
        {
            return 0;
        }
        //std::cout << "Output finished ... ..." << std::endl;
        return 1;
    }

    bool batchReadFileNamesInFolders(const std::string &folderName, const std::string &extension, std::vector<std::string> &fileNames)
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
                            LOG(INFO) << "Record the file: [" << fileName << "].";
                        }
                    }
                }
            }
        }

        return 1;
    }

    bool batchReadFileNamesInFoldersAndSubFolders(const std::string &folderName, const std::string &extension, std::vector<std::string> &fileNames)
    {
        boost::filesystem::path fullpath(folderName);
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
            catch (const std::exception &ex)
            {
                std::cerr << ex.what() << std::endl;
                continue;
            }
        }
        return true;
    }

    bool batchReadFileNamesInSubFolders(const std::string &folderName, const std::string &extension, std::vector<std::vector<std::string>> &fileNames)
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
        LOG(INFO) << subfolder_num << " Sub-folders in the folder have been processed";
        return 1;
    }

    bool batchReadMultiSourceFileNamesInDataFolders(const std::string &ALS_folder, const std::string &TLS_folder, const std::string &MLS_folder, const std::string &BPLS_folder,
                                                    std::vector<std::vector<std::string>> &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files)
    {
        batchReadFileNamesInSubFolders(ALS_folder, ".las", ALS_strip_files);
        batchReadFileNamesInFolders(TLS_folder, ".las", TLS_files);
        batchReadFileNamesInFolders(MLS_folder, ".las", MLS_files);
        batchReadFileNamesInFolders(BPLS_folder, ".las", BPLS_files);
        LOG(INFO) << "All Filenames have been imported ...";
        return 1;
    }

    bool outputKeypoints(const std::string &filename, const pcl::PointIndicesPtr &indices, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::ofstream ofs;
        ofs.open(filename);

        if (ofs.is_open())
        {
            for (int i = 0; i < indices->indices.size(); ++i)
            {
                ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[indices->indices[i]].x << "\t"
                    << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[indices->indices[i]].y << "\t"
                    << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[indices->indices[i]].z << std::endl;
            }
            ofs.close();
        }
        else
        {
            return 0;
        }
        return 1;
    }

    bool savecoordinates(const typename pcl::PointCloud<PointT>::Ptr &Source_FPC, const typename pcl::PointCloud<PointT>::Ptr &Target_FPC,
                         pcl::PointIndicesPtr &Source_KPI, pcl::PointIndicesPtr &Target_KPI,
                         Eigen::MatrixX3d &SXYZ, Eigen::MatrixX3d &TXYZ)
    {

        SXYZ.resize(Source_KPI->indices.size(), 3);
        TXYZ.resize(Target_KPI->indices.size(), 3);
        for (int i = 0; i < Source_KPI->indices.size(); ++i)
        {
            SXYZ.row(i) << Source_FPC->points[Source_KPI->indices[i]].x, Source_FPC->points[Source_KPI->indices[i]].y, Source_FPC->points[Source_KPI->indices[i]].z;
        }
        for (int i = 0; i < Target_KPI->indices.size(); ++i)
        {
            TXYZ.row(i) << Target_FPC->points[Target_KPI->indices[i]].x, Target_FPC->points[Target_KPI->indices[i]].y, Target_FPC->points[Target_KPI->indices[i]].z;
        }
        std::cout << "Key points saved." << std::endl;
        return 1;
    }

    bool ALS_block_by_time(const typename pcl::PointCloud<PointT>::Ptr &pointCloud, vector<typename pcl::PointCloud<PointT>> &cloudblock_ts, float time_step_in_second)
    {
        float time_max = -FLT_MAX;
        float time_min = FLT_MAX;

        float time_range;

        for (size_t i = 0; i < pointCloud->size(); i++)
        {
            if (pointCloud->points[i].intensity > time_max)
                time_max = pointCloud->points[i].intensity;
            if (pointCloud->points[i].intensity < time_min)
                time_min = pointCloud->points[i].intensity;
        }

        time_range = time_max - time_min;

        cloudblock_ts.resize(int(time_range / time_step_in_second) + 1);

        for (size_t i = 0; i < pointCloud->size(); i++)
        {
            int index = int(1.0 * (pointCloud->points[i].intensity - time_min) / time_step_in_second);
            cloudblock_ts[index].push_back(pointCloud->points[i]);
        }
        return 1;
    }

    bool batchWriteBlockInColor(const string &fileName, vector<typename pcl::PointCloud<PointT>> &cloudblock_ts, bool automatic_shift_or_not)
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

        for (int j = 0; j < cloudblock_ts.size(); j++)
        {
            Bounds bound;
            getCloudBound(cloudblock_ts[j], bound);

            liblas::Color lasColor;
            lasColor.SetRed(255 * (rand() / (1.0 + RAND_MAX)));
            lasColor.SetGreen(255 * (rand() / (1.0 + RAND_MAX)));
            lasColor.SetBlue(255 * (rand() / (1.0 + RAND_MAX)));

            string BlockFolder, BlockFilename;
            ostringstream oss;
            oss.setf(ios::right);
            oss.fill('0');
            oss.width(3);
            oss << j;

            BlockFolder = fileName.substr(0, fileName.rfind("."));

            if (!boost::filesystem::exists(BlockFolder))
            {
                boost::filesystem::create_directory(BlockFolder);
            }

            BlockFilename = BlockFolder + fileName.substr(fileName.rfind("\\"), fileName.rfind(".") - fileName.rfind("\\")) + "_" + oss.str() + ".las";
            if (j == 0)
                cout << BlockFilename << endl;

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
                header.SetPointRecordsCount(cloudblock_ts[j].points.size());

                liblas::Writer writer(ofs, header);
                liblas::Point pt(&header);

                for (size_t i = 0; i < cloudblock_ts[j].points.size(); i++)
                {
                    pt.SetCoordinates(double(cloudblock_ts[j].points[i].x) - global_shift[0], double(cloudblock_ts[j].points[i].y) - global_shift[1], double(cloudblock_ts[j].points[i].z) - global_shift[2]);
                    pt.SetColor(lasColor);
                    pt.SetIntensity(cloudblock_ts[j].points[i].intensity);
                    writer.WritePoint(pt);
                }
                ofs.flush();
                ofs.close();
            }
        }
        return 1;
    }

    bool readindiceslist(std::vector<int> &indicesA, std::vector<int> &indicesB)
    {
        std::string indiceslistFile;

        std::cout << "Please enter or drag in the Correspondence Tie Point Indices List File" << std::endl
                  << "Example [IndicesListFile.txt] :" << std::endl
                  << "107562 934051 " << std::endl
                  << "275003 18204" << std::endl
                  << "872055 462058" << std::endl
                  << "...  ..." << std::endl;

        std::cin >> indiceslistFile;

        std::ifstream in(indiceslistFile, ios::in);
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

    bool readindiceslist(const typename pcl::PointCloud<PointT>::Ptr &CloudA, const typename pcl::PointCloud<PointT>::Ptr &CloudB, std::vector<std::vector<double>> &coordinatesA, std::vector<std::vector<double>> &coordinatesB)
    {
        std::string indiceslistFile;

        std::cout << "Please enter or drag in the Correspondence Tie Point Indices List File" << std::endl
                  << "Example [IndicesListFile.txt] :" << std::endl
                  << "107562 934051 " << std::endl
                  << "275003 18204" << std::endl
                  << "872055 462058" << std::endl
                  << "...  ..." << std::endl;

        std::cin >> indiceslistFile;

        std::ifstream in(indiceslistFile, ios::in);
        if (!in)
        {
            return 0;
        }

        std::vector<int> pointlistA;
        std::vector<int> pointlistB;

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

        std::cout << "Procession Done ..." << std::endl;
    }

    bool read_XYZ_XYZlist(std::vector<std::vector<double>> &coordinatesA, std::vector<std::vector<double>> &coordinatesB)
    {
        std::string XYZListFileA, XYZListFileB;

        std::cout << "Please enter or drag in the Tie Points' XYZ List File of Station A" << std::endl
                  << "Example [pickinglist_XYZ_A.txt] :" << std::endl
                  << "11.92,23.07,0.82" << std::endl
                  << "15.34,18.02,1.25" << std::endl
                  << "27.01,-7.94,1.37" << std::endl
                  << "...  ... ..." << std::endl;

        std::cin >> XYZListFileA;

        std::ifstream inA(XYZListFileA, ios::in);
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

        std::cout << "Please enter or drag in the Tie Points' XYZ List File of Station B" << std::endl;

        std::cin >> XYZListFileB;

        std::ifstream inB(XYZListFileB, ios::in);
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

        std::cout << "Procession Done ..." << std::endl;
    }

    bool XYZ_4DOFCSTran(std::vector<double> &transpara)
    {
        std::string XYZListFileA, XYZListFileB;
        std::vector<std::vector<double>> coordinatesA;
        std::cout << "Please enter or drag in the XYZ List File of Coordinate System A" << std::endl
                  << "Example [pointlist_XYZ_A.txt] :" << std::endl
                  << "11.92,23.07,0.82" << std::endl
                  << "15.34,18.02,1.25" << std::endl
                  << "27.01,-7.94,1.37" << std::endl
                  << "...  ... ..." << std::endl;

        std::cin >> XYZListFileA;

        std::ifstream inA(XYZListFileA, ios::in);
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

        std::cout << "Output the transformed result" << std::endl;
        XYZListFileB = XYZListFileA.substr(0, XYZListFileA.rfind(".")) + "_utm.txt";
        std::ofstream ofs;
        ofs.open(XYZListFileB);
        if (ofs.is_open())
        {
            for (int j = 0; j < i; j++)
            {
                double X_tran = transpara[2] * transpara[4] * coordinatesA[j][0] - transpara[2] * transpara[3] * coordinatesA[j][1] + transpara[0];
                double Y_tran = transpara[2] * transpara[3] * coordinatesA[j][0] + transpara[2] * transpara[4] * coordinatesA[j][1] + transpara[1];
                double Z_tran = coordinatesA[j][2];
                ofs << setiosflags(ios::fixed) << setprecision(8) << X_tran << ","
                    << setiosflags(ios::fixed) << setprecision(8) << Y_tran << ","
                    << setiosflags(ios::fixed) << setprecision(8) << Z_tran << std::endl;
            }
            ofs.close();
        }
        else
        {
            return 0;
        }

        std::cout << "Procession Done ..." << std::endl;
        return 1;
    }

    bool XYZ_7DOFCSTran(std::vector<double> &transpara)
    {
        std::string XYZListFileA, XYZListFileB;
        std::vector<std::vector<double>> coordinatesA;
        std::cout << "Please enter or drag in the XYZ List File of Coordinate System A" << std::endl
                  << "Example [pointlist_XYZ_A.txt] :" << std::endl
                  << "11.92,23.07,0.82" << std::endl
                  << "15.34,18.02,1.25" << std::endl
                  << "27.01,-7.94,1.37" << std::endl
                  << "...  ... ..." << std::endl;

        std::cin >> XYZListFileA;

        std::ifstream inA(XYZListFileA, ios::in);
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

        std::cout << "Output the transformed result" << std::endl;
        XYZListFileB = XYZListFileA.substr(0, XYZListFileA.rfind(".")) + "_utm.txt";
        std::ofstream ofs;
        ofs.open(XYZListFileB);
        if (ofs.is_open())
        {
            for (int j = 0; j < i; j++)
            {
                double X_tran = transpara[0] + transpara[6] * coordinatesA[j][0] + transpara[5] * coordinatesA[j][1] - transpara[4] * coordinatesA[j][2];
                double Y_tran = transpara[1] + transpara[6] * coordinatesA[j][1] - transpara[5] * coordinatesA[j][0] + transpara[3] * coordinatesA[j][2];
                double Z_tran = transpara[2] + transpara[6] * coordinatesA[j][2] + transpara[4] * coordinatesA[j][0] - transpara[3] * coordinatesA[j][1];
                ofs << setiosflags(ios::fixed) << setprecision(8) << X_tran << ","
                    << setiosflags(ios::fixed) << setprecision(8) << Y_tran << ","
                    << setiosflags(ios::fixed) << setprecision(8) << Z_tran << std::endl;
            }
            ofs.close();
        }
        else
        {
            return 0;
        }

        std::cout << "Procession Done ..." << std::endl;
        return 1;
    }

#if 0
    bool tran_eng2utm(float centerlong_eng_proj)
    {
        string XYZENGListFile;

        cout << "Please enter or drag in the Points' XYZ List File for Engineering Coordinate System" << endl
             << "Example [Pointlist_XYZ_ENGCS.txt] :" << endl
             << "485026.778,3409071.864,474.672" << endl
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

            cout.setf(ios::showpoint);
            cout.precision(12);

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

    bool tran_wgs2eng(float centerlong_eng_proj, float proj_surface_h_eng)
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

            cout.setf(ios::showpoint);
            cout.precision(12);

            gt.BLH2XYZ_CGCS(PtBLH, centerlong_eng_proj, proj_surface_h_eng, PtENGXYZ);
            cout << PtENGXYZ[0] << " , " << PtENGXYZ[1] << " , " << PtENGXYZ[2] << endl;

            ++j;
        }
        inlist.close();

        cout << "Procession Done ..." << endl;
        return 1;
    }

    bool read_XYZ_BLHlist(std::vector<std::vector<double>> &coordinatesSC_XYZ,
                          std::vector<std::vector<double>> &coordinatesUTM_XYZ)
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

            cout.setf(ios::showpoint);
            cout.precision(12);

            cout << PtUTM[0] << " , " << PtUTM[1] << " , " << PtUTM[2] << endl;
            coordinatesUTM_XYZ.push_back(PtUTM);
            ++j;
        }
        inB.close();

        cout << "Procession Done ..." << endl;
    }
#endif
    bool readLasBlock(const string &fileName, DataType data_type_, int strip_num_, int num_in_strip_, cloudblock_t &block)
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
        liblas::Header const &header = reader.GetHeader();

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
        block.cp.x = 0.5 * (block.bound.min_x + block.bound.max_x);
        block.cp.y = 0.5 * (block.bound.min_y + block.bound.max_y);
        block.cp.z = 0.5 * (block.bound.min_z + block.bound.max_z);

        return 1;
    }

    void batchReadMultiSourceLasBlock(std::vector<std::vector<std::string>> &ALS_strip_files,
                                      std::vector<std::string> &TLS_files,
                                      std::vector<std::string> &MLS_files,
                                      std::vector<std::string> &BPLS_files,
                                      std::vector<std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>>> &ALS_strip_blocks,
                                      std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &TLS_blocks,
                                      std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &MLS_blocks,
                                      std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &BPLS_blocks,
                                      std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &All_blocks)
    {
        //ALS
        int ALS_count = 0;
        for (int i = 0; i < ALS_strip_files.size(); i++)
        {
            ALS_strip_blocks[i].resize(ALS_strip_files[i].size());
            for (int j = 0; j < ALS_strip_files[i].size(); j++)
            {
                LOG(INFO) << "Read ALS file: [" << ALS_strip_files[i][j] << "].";

                readLasBlock(ALS_strip_files[i][j], ALS, i, j, ALS_strip_blocks[i][j]);
                ALS_strip_blocks[i][j].unique_index = ALS_count; //Set Block Unique Index
                All_blocks.push_back(ALS_strip_blocks[i][j]);
                ALS_count++;
            }
        }
        cout << "ALS boxes import done ..." << endl;

        //TLS
        for (int i = 0; i < TLS_files.size(); i++)
        {
            LOG(INFO) << "Read TLS file: [" << TLS_files[i] << "].";
            readLasBlock(TLS_files[i], TLS, 0, i, TLS_blocks[i]);
            TLS_blocks[i].unique_index = ALS_count + i; //Set Block Unique Index
            All_blocks.push_back(TLS_blocks[i]);
        }
        cout << "TLS boxes import done ..." << endl;

        //MLS
        for (int i = 0; i < MLS_files.size(); i++)
        {
            LOG(INFO) << "Read MLS file: [" << MLS_files[i] << "].";
            readLasBlock(MLS_files[i], MLS, 0, i, MLS_blocks[i]);
            MLS_blocks[i].unique_index = ALS_count + TLS_files.size() + i; //Set Block Unique Index
            All_blocks.push_back(MLS_blocks[i]);
        }
        cout << "MLS boxes import done ..." << endl;

        //BPLS
        for (int i = 0; i < BPLS_files.size(); i++)
        {
            LOG(INFO) << "Read BPLS file: [" << BPLS_files[i] << "].";
            readLasBlock(BPLS_files[i], BPLS, 0, i, BPLS_blocks[i]);
            BPLS_blocks[i].unique_index = ALS_count + TLS_files.size() + MLS_files.size() + i; //Set Block Unique Index
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

    bool lasfileGK2UTM(const string &fileName)
    {
        const int drift = 500000;
        const double utm_scale = 0.9996;

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
        liblas::Header const &header = reader.GetHeader();
        double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax, Xmin_, Ymin_, Xmax_, Ymax_;
        ;
        Xmin = header.GetMinX();
        Ymin = header.GetMinY();
        Zmin = header.GetMinZ();
        Xmax = header.GetMaxX();
        Ymax = header.GetMaxY();
        Zmax = header.GetMaxZ();
        Xmin_ = (Xmin - drift) * utm_scale + drift;
        Xmax_ = (Xmax - drift) * utm_scale + drift;
        Ymin_ = Ymin * utm_scale;
        Ymax_ = Ymax * utm_scale;

        string fileNameout = fileName.substr(0, fileName.rfind(".")) + "_utm.las";

        ofstream ofs;
        ofs.open(fileNameout, std::ios::out | std::ios::binary);

        if (ofs.is_open())
        {
            liblas::Header headerout;
            headerout.SetDataFormatId(liblas::ePointFormat2);
            headerout.SetVersionMajor(1);
            headerout.SetVersionMinor(2);

            headerout.SetMin(Xmin_, Ymin_, Zmin);
            headerout.SetMax(Xmax_, Ymax_, Zmax);
            headerout.SetOffset((Xmin_ + Xmax_) / 2.0, (Ymin_ + Ymax_) / 2.0, (Zmin + Zmax) / 2.0);
            headerout.SetScale(0.01, 0.01, 0.01);
            headerout.SetPointRecordsCount(header.GetPointRecordsCount());

            liblas::Writer writer(ofs, headerout);

            while (reader.ReadNextPoint())
            {
                liblas::Point p = reader.GetPoint();
                liblas::Point pt(&headerout);
                double pX, pY, pZ;
                pX = (p.GetX() - drift) * utm_scale + drift;
                pY = p.GetY() * utm_scale;
                pZ = p.GetZ();
                pt.SetCoordinates(pX, pY, pZ);
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

    bool lasfileshift(const string &fileName, vector<double> &shift)
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
        liblas::Header const &header = reader.GetHeader();
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

    void readLasCloudPairfromCon(const constraint_t &this_con, std::vector<std::vector<std::string>> &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files,
                                 std::string &Filename1, std::string &Filename2,
                                 typename pcl::PointCloud<PointT>::Ptr &cloud1, typename pcl::PointCloud<PointT>::Ptr &cloud2)
    {
        //Find the file index
        switch (this_con.block1.data_type)
        {
        case ALS: //ALS
            Filename1 = ALS_strip_files[this_con.block1.strip_num][this_con.block1.num_in_strip];
            break;
        case TLS: //TLS
            Filename1 = TLS_files[this_con.block1.num_in_strip];
            break;
        case MLS: //MLS
            Filename1 = MLS_files[this_con.block1.num_in_strip];
            break;
        case BPLS: //BPLS
            Filename1 = BPLS_files[this_con.block1.num_in_strip];
            break;
        default:
            break;
        }
        switch (this_con.block2.data_type)
        {
        case ALS: //ALS
            Filename2 = ALS_strip_files[this_con.block2.strip_num][this_con.block2.num_in_strip];
            break;
        case TLS: //TLS
            Filename2 = TLS_files[this_con.block2.num_in_strip];
            break;
        case MLS: //MLS
            Filename2 = MLS_files[this_con.block2.num_in_strip];
            break;
        case BPLS: //BPLS
            Filename2 = BPLS_files[this_con.block2.num_in_strip];
            break;
        default:
            break;
        }

        //Import the point clouds for registration
        // Cloud1:T  Cloud2:S  Trans12=inv(Trans21)=inv(RegTrans)
        readLasFile(Filename1, cloud1, 1);
        LOG(INFO) << "Read File " << this_con.block1.data_type << "-" << this_con.block1.strip_num << "-" << this_con.block1.num_in_strip << " Done...";
        LOG(INFO) << "Filename: " << Filename1;
        readLasFileLast(Filename2, cloud2);
        LOG(INFO) << "Read File " << this_con.block2.data_type << "-" << this_con.block2.strip_num << "-" << this_con.block2.num_in_strip << " Done...";
        LOG(INFO) << "Filename: " << Filename2;
        LOG(INFO) << "Raw point number: [ " << cloud1->size() << "  ,  " << cloud2->size() << "  ]" << endl;
    }

    bool batchwritefinalcloud(vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &All_blocks, std::vector<std::vector<std::string>> &ALS_strip_files,
                              std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files,
                              std::vector<std::string> &BPLS_files)
    {
        for (auto iter = All_blocks.begin(); iter != All_blocks.end(); iter++)
        {
            string Filenamein, Folderout, Filenameout;
            switch ((*iter).data_type)
            {
            case ALS: //ALS
                Filenamein = ALS_strip_files[(*iter).strip_num][(*iter).num_in_strip];
                break;
            case TLS: //TLS
                Filenamein = TLS_files[(*iter).num_in_strip];
                break;
            case MLS: //MLS
                Filenamein = MLS_files[(*iter).num_in_strip];
                break;
            case BPLS: //BPLS
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
            //Eigen::Matrix4f corrected_pose;
            //regx.invTransform((*iter).optimized_pose, corrected_pose);
            pcl::transformPointCloud(*cloudin, *cloudout, (*iter).optimized_pose);
            writeLasFile(Filenameout, cloudout, 1);
            LOG(INFO) << "Output Done for cloud with index " << (*iter).unique_index;
            LOG(INFO) << "Its position is " << Filenameout;
        }

        return 1;
    }

  private:
    void pcXYZ2XY(const typename pcl::PointCloud<PointT>::Ptr &pointcloudxyz, pcl::PointCloud<pcl::PointXY>::Ptr &pointcloudxy)
    {
        for (size_t i = 0; i < pointcloudxyz->size(); i++)
        {
            pcl::PointXY ptXY;
            ptXY.x = pointcloudxyz->points[i].x;
            ptXY.y = pointcloudxyz->points[i].y;
            pointcloudxy->push_back(ptXY);
        }
    }

    std::vector<double> global_shift;
};

} // namespace ccn

#endif // _INCLUDE_DATA_IO_HPP