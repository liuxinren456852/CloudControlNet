#include "find_constraint.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#define max_(a,b) (((a) > (b)) ? (a) : (b))
#define min_(a,b) (((a) < (b)) ? (a) : (b))

void Constraint_Finder::find_adjacent_constraint_in_strip(vector<CloudBlock> &blocks_strip, vector<Constraint> &innerstrip_cons)
{
	for (int i = 0; i < blocks_strip.size() - 1; i++)
	{
		Constraint con;
		con.block1 = blocks_strip[i];
		con.block2 = blocks_strip[i + 1];
		con.con_type = 1;   //Adjacent
		innerstrip_cons.push_back(con);
	}
}

void Constraint_Finder::find_strip_adjacent_constraint(vector<vector<CloudBlock>> &blocks_all, vector<Constraint> &innerstrip_cons_all)
{
	for (int i = 0; i < blocks_all.size(); i++)
	{
		vector<Constraint> innerstrip_cons;
		find_adjacent_constraint_in_strip(blocks_all[i], innerstrip_cons);
		for (int j = 0; j < innerstrip_cons.size(); j++)
		{
			innerstrip_cons_all.push_back(innerstrip_cons[j]);
		}
		innerstrip_cons.clear();
	}
}


void Constraint_Finder::find_overlap_registration_constraint(vector<CloudBlock> &blocks, vector<Constraint> &registration_cons_all, int k_search_neighbor, double min_iou)
{
	pcl::PointCloud<pcl::PointXY>::Ptr cp_cloud(new pcl::PointCloud<pcl::PointXY>());
	
	for (int i = 0; i < blocks.size(); i++)
	{
		pcl::PointXY cp;
		cp.x = blocks[i].cp.x;
		cp.y = blocks[i].cp.y;
		cp_cloud->push_back(cp);
	}

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(cp_cloud);	

	for (int i = 0; i < blocks.size(); i++)
	{
		std::vector<int> pointIdx;
		std::vector<float> pointSquaredDistance;

		pcl::PointXY cp_search;
		cp_search.x = blocks[i].cp.x;
		cp_search.y = blocks[i].cp.y;

		kdtree.nearestKSearch(cp_search, k_search_neighbor, pointIdx, pointSquaredDistance);
        
		for (int j = 0; j < pointIdx.size(); j++)
		{
			//Ч�ʿ�����;
			double iou = calculate_iou(blocks[i].bound, blocks[pointIdx[j]].bound);
			bool is_adjacent = judge_adjacent(blocks[i],blocks[pointIdx[j]]);

			if ((pointIdx[j]>i) && (iou > min_iou) && (!is_adjacent))
			{
				Constraint registration_con;
				registration_con.block1 = blocks[i];
				registration_con.block2 = blocks[pointIdx[j]];
				registration_con.con_type = 2; //Registration

				registration_cons_all.push_back(registration_con);
			}
		}
	}
}

bool Constraint_Finder::judge_adjacent(CloudBlock & block1, CloudBlock & block2)
{
	bool is_adjacent = false;
	if (block1.data_type == block2.data_type && block1.strip_num == block2.strip_num)
	{
		if ((block1.num_in_strip == block2.num_in_strip + 1) || (block1.num_in_strip == block2.num_in_strip - 1))
		{
			is_adjacent=true;
		}
	}
	return is_adjacent;
}

double Constraint_Finder::calculate_iou(Bounds & bound1, Bounds & bound2)
{
	double area1 = (bound1.max_x - bound1.min_x) * (bound1.max_y - bound1.min_y); 
	double area2 = (bound2.max_x - bound2.min_x) * (bound2.max_y - bound2.min_y);

	double x1 = max_(bound1.min_x, bound2.min_x);
	double y1 = max_(bound1.min_y, bound2.min_y);

	double x2 = min_(bound1.max_x, bound2.max_x);
	double y2 = min_(bound1.max_y, bound2.max_y);
	
	double w = max_(0, x2 - x1);
	double h = max_(0, y2 - y1);

	double area1i2 = w*h;

	double iou = area1i2 / (area1 + area2 - area1i2);

	return iou;
}

void Constraint_Finder::batch_find_multisource_constranits(std::vector<std::vector<CloudBlock>> &ALS_strip_blocks, std::vector<CloudBlock> &TLS_blocks, std::vector<CloudBlock> &MLS_blocks, std::vector<CloudBlock> &BPLS_blocks, std::vector<CloudBlock> &All_blocks,
	std::vector<Constraint> &ALS_inner_strip_cons_all, std::vector<Constraint> &MLS_adjacent_cons, std::vector<Constraint> &BPLS_adjacent_cons, std::vector<Constraint> &registration_cons, std::vector<Constraint> &All_cons,
	int overlap_Registration_KNN, float overlap_Registration_OverlapRatio)
{
	int adjacent_cons_num;
	find_strip_adjacent_constraint(ALS_strip_blocks, ALS_inner_strip_cons_all);
	if(MLS_blocks.size()>1) find_adjacent_constraint_in_strip(MLS_blocks, MLS_adjacent_cons);
	if(BPLS_blocks.size()>1) find_adjacent_constraint_in_strip(BPLS_blocks, BPLS_adjacent_cons);
	find_overlap_registration_constraint(All_blocks, registration_cons, overlap_Registration_KNN, overlap_Registration_OverlapRatio);

	cout << "Constraints Found" << endl;
	if (ALS_inner_strip_cons_all.size()>0) All_cons.insert(All_cons.end(), ALS_inner_strip_cons_all.begin(), ALS_inner_strip_cons_all.end());
	if (MLS_adjacent_cons.size()>0) All_cons.insert(All_cons.end(), MLS_adjacent_cons.begin(), MLS_adjacent_cons.end());
	if (BPLS_adjacent_cons.size()>0) All_cons.insert(All_cons.end(), BPLS_adjacent_cons.begin(), BPLS_adjacent_cons.end());
	adjacent_cons_num = All_cons.size();
	if (registration_cons.size()>0) All_cons.insert(All_cons.end(), registration_cons.begin(), registration_cons.end());

	cout << "The number of constraints : " << All_cons.size() << endl;
	cout << "The number of adjacent constraints : " << adjacent_cons_num << endl;
	cout << "The number of registration constraints : " << registration_cons.size() << endl;
	cout << "!----------------------------------------------------------------------------!" << endl;
}