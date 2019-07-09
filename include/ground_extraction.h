#ifndef EXTRACTGROUND_H
#define EXTRACTGROUND_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include "utility.h"
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

//Reference: Two-step adaptive extraction method for ground points and breaklines from lidar point clouds, Bisheng Yang, Ronggang Huang, et al. ISPRS Journal of Photogrammetry and Remote Sensing
//Copyright: Ronggang Huang et al.

using namespace utility;

namespace groundExtraction
{  
	class Ground_Extraction
	{
	public:

		Ground_Extraction();
		~Ground_Extraction();

		void ExtractGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
			Bounds bounds, CenterPoint center_pt);

		void SetMaxHeightDifference(float max_height_difference){ max_height_difference_ = max_height_difference; }
		void SetMinPointNumInGrid(int min_pt_num_in_grid){ min_pt_num_in_grid_ = min_pt_num_in_grid; }
		void SetGridResolution(float grid_resolution){ grid_resolution_ = grid_resolution; }

		

	protected:

	private:


		void CalculateGridAndMinZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			double max_x, double max_y, double min_x, double min_y,
			int row, int list, int num_voxel, Voxel* grid);

		void JudgeGroundAndNonGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
			Voxel* grid, int num_voxel);

		void FilterNotGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud);

		float grid_resolution_;
		int   min_pt_num_in_grid_;
		float max_height_difference_;


	};
}
#endif