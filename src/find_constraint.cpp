#include "find_constraint.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#define max_(a,b) (((a) > (b)) ? (a) : (b))
#define min_(a,b) (((a) < (b)) ? (a) : (b))
#define abs_(a) (((a) < (0)) ? (-a) : (a))


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

void Constraint_Finder::find_strip_adjacent_constraint(vector<vector<CloudBlock> > &blocks_all, vector<Constraint> &innerstrip_cons_all)
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


bool Constraint_Finder::judge_adjacent_hdmap_index(Frame & frame1, Frame & frame2, int index_min_interval)
{
	bool is_adjacent = false;
	
	if (frame1.type == frame2.type && frame1.transaction_id ==  frame2.transaction_id)
	{
		if (abs_(frame1.id_in_transaction - frame2.id_in_transaction) <= index_min_interval)
		{
			is_adjacent=true;
		}
	}
	return is_adjacent;
}


bool Constraint_Finder::judge_adjacent_hdmap_time(Frame & frame1, Frame & frame2, double min_deltatime_in_us)
{
	bool is_adjacent = false;
	

	if (frame1.type == frame2.type && frame1.transaction_id ==  frame2.transaction_id)
	{
		double deltatime_in_us=abs_( (frame1.time_stamp.tv_sec * 1000000 + frame1.time_stamp.tv_usec) -
			(frame2.time_stamp.tv_sec * 1000000 + frame2.time_stamp.tv_usec) );
		
		if (deltatime_in_us <= min_deltatime_in_us)
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

void Constraint_Finder::batch_find_multisource_constraints(std::vector<std::vector<CloudBlock> > &ALS_strip_blocks, std::vector<CloudBlock> &TLS_blocks, std::vector<CloudBlock> &MLS_blocks, std::vector<CloudBlock> &BPLS_blocks, std::vector<CloudBlock> &All_blocks,
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

void Constraint_Finder::find_hdmap_adjacent_constraints(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_adjacent_edges)
{
   //Fix it for multi-transaction cases
   for (int i = 0; i < HDmap_frames.size() - 1; i++)
	{
		Edge_between_2Frames edge_adjacent;
		edge_adjacent.frame1 = HDmap_frames[i];
		edge_adjacent.frame2 = HDmap_frames[i+1];
		edge_adjacent.type = ADJACENT;   
		HDmap_adjacent_edges.push_back(edge_adjacent);
	}
}


void Constraint_Finder::find_hdmap_revisit_constraints(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_revisit_edges,  int index_min_interval, float max_revisit_pos_distance)
{
	pcl::PointCloud<pcl::PointXY>::Ptr cp_cloud(new pcl::PointCloud<pcl::PointXY>());
	
	for (int i = 0; i < HDmap_frames.size(); i++)
	{
		pcl::PointXY cp;
		cp.x = HDmap_frames[i].oxts_position(0);
		cp.y = HDmap_frames[i].oxts_position(1);
		cp_cloud->push_back(cp);
	}

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(cp_cloud);	

	for (int i = 0; i < HDmap_frames.size(); i++)
	{
		std::vector<int> pointIdx;
		std::vector<float> pointSquaredDistance;

		pcl::PointXY cp_search;
		cp_search.x = HDmap_frames[i].oxts_position(0);
		cp_search.y = HDmap_frames[i].oxts_position(1);

		kdtree.radiusSearch(cp_search, max_revisit_pos_distance * max_revisit_pos_distance, pointIdx, pointSquaredDistance);
        
		for (int j = 0; j < pointIdx.size(); j++)
		{
			if ( (pointIdx[j]>i) && 
			( !judge_adjacent_hdmap_index(HDmap_frames[i],HDmap_frames[pointIdx[j]],index_min_interval) ) )
			{
				Edge_between_2Frames revisit_edge;
				revisit_edge.frame1 = HDmap_frames[i];
				revisit_edge.frame2 = HDmap_frames[pointIdx[j]];
				revisit_edge.type = REVISIT ; 

				HDmap_revisit_edges.push_back(revisit_edge);
			}
		}
	}
}


void Constraint_Finder::batch_find_hdmap_constraints(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_edges, int index_min_interval, float max_revisit_pos_distance)
{
    vector<Edge_between_2Frames> HDmap_revisit_edges;
	vector<Edge_between_2Frames> HDmap_adjacent_edges;
	
	find_hdmap_adjacent_constraints(HDmap_frames,HDmap_adjacent_edges);
	find_hdmap_revisit_constraints(HDmap_frames,HDmap_revisit_edges, index_min_interval, max_revisit_pos_distance);
	
	HDmap_edges.insert(HDmap_edges.end(),HDmap_adjacent_edges.begin(),HDmap_adjacent_edges.end());
	HDmap_edges.insert(HDmap_edges.end(),HDmap_revisit_edges.begin(),HDmap_revisit_edges.end());
	
	//output
	printf("The number of constraints : %d\n", HDmap_edges.size());
	printf("The number of adjacent constraints : %d\n", HDmap_adjacent_edges.size());
	printf("The number of revisit constraints : %d\n", HDmap_revisit_edges.size());

}