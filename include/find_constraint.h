#ifndef F_CONSTRAINT_H
#define F_CONSTRAINT_H

#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;
using namespace Eigen;

namespace utility
{
	struct CloudBlock  //Cloud Block Node
	{
		int unique_index;   
		int strip_num;      
		int num_in_strip;   
		int data_type;      
		Bounds bound;       
		CenterPoint cp;     
		Eigen::Matrix4f optimized_pose;   //Transformation Matrix after optimization
	};

	struct Constraint //Constraint between two Cloud Blocks
	{
		CloudBlock block1, block2; 
		int con_type;              
		Eigen::Matrix4f Trans1_2 = Eigen::Matrix4f::Identity(4,4);  
		float confidence;          
	};
    
	enum frame_type{ HDL64, VLP16, VLP32};
    
    enum edge_type{ REVISIT, ADJACENT };

    struct Frame // Cloud Frame Node
	{
        int unique_id;
		frame_type type;
		int transaction_id;
        int id_in_transaction;

		string pcd_file_name;
		Eigen::Matrix4d oxts_pose;
        Eigen::Vector3d oxts_position;
		timeval time_stamp;
    
		Bounds boundingbox;
		CenterPoint centerpoint;
		Eigen::Matrix4d optimized_pose;
        
		Frame* lastframe;
		pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud;

	};

	struct Edge_between_2Frames //Registration edge between two cloud frame nodes
	{
		int unique_id;
		edge_type type;
     
		Frame frame1,frame2;
		Eigen::Matrix4d Trans1_2 = Eigen::Matrix4d::Identity(4,4);
		float confidence;    
	};



	class Constraint_Finder
	{
	public:
		// Constraint Type 1
		void find_strip_adjacent_constraint(vector<vector<CloudBlock> > &blocks_all, vector<Constraint> &innerstrip_cons_all);
		void find_adjacent_constraint_in_strip(vector<CloudBlock> &blocks_strip, vector<Constraint> &innerstrip_cons);

		// Constraint Type 2
		void find_overlap_registration_constraint(vector<CloudBlock> &blocks, vector<Constraint> &registration_cons_all, int k_search_neighbo, double min_iou);

		// Batch Construct Constraints
		void batch_find_multisource_constraints(vector<vector<CloudBlock> > &ALS_strip_blocks, vector<CloudBlock> &TLS_blocks, vector<CloudBlock> &MLS_blocks, vector<CloudBlock> &BPLS_blocks, vector<CloudBlock> &All_blocks,
			vector<Constraint> &ALS_inner_strip_cons_all, vector<Constraint> &MLS_adjacent_cons, vector<Constraint> &BPLS_adjacent_cons, vector<Constraint> &registration_cons, vector<Constraint> &All_cons,
			int overlap_Registration_KNN, float overlap_Registration_OverlapRatio);
        

		//For HDMap application
        void find_hdmap_adjacent_constraints(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_adjacent_edges);

        void find_hdmap_revisit_constraints(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_revisit_edges, int index_min_interval, float max_revisit_pos_dis);
        
        void batch_find_hdmap_constraints(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_edges, int index_min_interval, float max_revisit_pos_distance);
       
	protected:
		void find_neighbor_k_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, int k);
		void find_neighbor_r_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, float r);
        
		double calculate_iou(Bounds & bound1, Bounds & bound2);
		bool judge_adjacent(CloudBlock & block1, CloudBlock & block2);
        bool judge_adjacent_hdmap_index(Frame & frame1, Frame & frame2, int index_min_interval);         //for HDMap application
        bool judge_adjacent_hdmap_time(Frame & frame1, Frame & frame2, double min_deltatime_in_us);      //for HDMap application   

	private:

	};
}
#endif //F_CONSTRAINT_H