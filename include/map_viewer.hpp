#ifndef _INCLUDE_MAP_VIEWER_HPP
#define _INCLUDE_MAP_VIEWER_HPP

#include <string>
#include <fstream>
#include <vector>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <glog/logging.h>

#include "utility.h"
#include "find_constraint.h"

namespace ccn
{

// Visualization color scale
enum color_type
{
    INTENSITY,
    HEIGHT,
    SINGLE,
    FRAME
};

template <typename PointT>
class MapViewer
{
  public:
    //Constructor
    MapViewer(){};
    ~MapViewer(){};

    void display2Dboxes(const vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &blocks)
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
            case ALS: //ALS
                f_red = 1.0;
                f_green = 0.0;
                f_blue = 0.0;
                break;
            case TLS: //TLS
                f_red = 0.0;
                f_green = 1.0;
                f_blue = 0.0;
                break;
            case MLS: //MLS
                f_red = 0.0;
                f_green = 0.0;
                f_blue = 1.0;
                break;
            case BPLS: //BPLS
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

    void display2Dcons(const vector<constraint_t> &cons)
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
            case ALS: //ALS
                f_red = 1.0;
                f_green = 0.0;
                f_blue = 0.0;
                break;
            case TLS: //TLS
                f_red = 0.0;
                f_green = 1.0;
                f_blue = 0.0;
                break;
            case MLS: //MLS
                f_red = 0.0;
                f_green = 0.0;
                f_blue = 1.0;
                break;
            case BPLS: //BPLS
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
            case ALS: //ALS
                f_red = 1.0;
                f_green = 0.0;
                f_blue = 0.0;
                break;
            case TLS: //TLS
                f_red = 0.0;
                f_green = 1.0;
                f_blue = 0.0;
                break;
            case MLS: //MLS
                f_red = 0.0;
                f_green = 0.0;
                f_blue = 1.0;
                break;
            case BPLS: //BPLS
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
            case ADJACENT: //Adjacent
                f_red = 0.0;
                f_green = 1.0;
                f_blue = 1.0;
                break;
            case REGISTRATION: //Registration
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

    void Dispaly2Cloud(const typename pcl::PointCloud<PointT>::Ptr &Cloud1, const typename pcl::PointCloud<PointT>::Ptr &Cloud2,
                       std::string displayname, int display_downsample_ratio)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));

        viewer->setBackgroundColor(255, 255, 255);
        char t[256];
        std::string s;
        int n = 0;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < Cloud1->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud1->points[i].x;
                pt.y = Cloud1->points[i].y;
                pt.z = Cloud1->points[i].z;
                pt.r = 255;
                pt.g = 215;
                pt.b = 0;
                pointcloud1->points.push_back(pt);
            }
        } // Golden

        viewer->addPointCloud(pointcloud1, "pointcloudT");

        for (size_t i = 0; i < Cloud2->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud2->points[i].x;
                pt.y = Cloud2->points[i].y;
                pt.z = Cloud2->points[i].z;
                pt.r = 233;
                pt.g = 233;
                pt.b = 216;
                pointcloud2->points.push_back(pt);
            }
        } // Silver

        viewer->addPointCloud(pointcloud2, "pointcloudS");

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
};
} // namespace ccn

#endif //_INCLUDE_MAP_VIEWER_HPP