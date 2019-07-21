#ifndef F_CONSTRAINT_H
#define F_CONSTRAINT_H

#include <vector>
#include "utility.h"
#include "common_reg.cpp"
#include "filter.cpp"

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
    

	class Constraint_Finder
	{
	public:
		
		//Constraint_Finder(); //Constructor

		//~Constraint_Finder(); //Destructor

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
    
		void divide_submap(Transaction & transaction, int max_frame_num, float max_tran_dis, float max_heading_angle);

		void front_end(Transaction &transaction);
        
		void back_end(Transaction &transaction, int index_min_interval, float max_revisit_pos_distance, float min_iou);
        
		//Find Constraints among Frame Nodes
        void find_hdmap_adjacent_constraints_frame(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_adjacent_edges);

        void find_hdmap_revisit_constraints_frame(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_revisit_edges, int index_min_interval, float max_revisit_pos_dis);
        
        void batch_find_hdmap_constraints_frame(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_edges, int index_min_interval, float max_revisit_pos_distance);
       
	    //Find Constraints among Submap Nodes
        void find_hdmap_adjacent_constraints_submap(Transaction & transaction, vector<Edge_between_2Submaps> & HDmap_adjacent_edges);

        void find_hdmap_revisit_constraints_submap(Transaction & transaction, vector<Edge_between_2Submaps> & HDmap_revisit_edges, int index_min_interval, float max_revisit_pos_dis,  float min_iou);
        
        void batch_find_hdmap_constraints_submap(Transaction & transaction, int index_min_interval, float max_revisit_pos_distance, float min_iou);

        //Submap Odometry
	   	bool submapOdom(SubMap & submap);
        
		//Frame to Map Reg
	    bool pairwiseReg(Frame &frame1, Frame &frame2);
        
		//Map to Map Reg 
		bool pairwiseReg(Edge_between_2Submaps &edge);

	protected:

		void find_neighbor_k_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, int k);
		void find_neighbor_r_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, float r);
        
		double calculate_iou(Bounds & bound1, Bounds & bound2);
		
		bool merge_frame_bounds_and_centers(SubMap & submap);  //Use the submap's frames' bounds and center points to calculate the submap's bounds and center

		bool judge_adjacent(CloudBlock & block1, CloudBlock & block2);
        bool judge_adjacent_hdmap_index(Frame & frame1, Frame & frame2, int index_min_interval);         //for HDMap application
        bool judge_adjacent_hdmap_time(Frame & frame1, Frame & frame2, double min_deltatime_in_us);      //for HDMap application   
        bool judge_adjacent_hdmap_index(SubMap & submap1, SubMap & submap2, int index_min_interval);     //for HDMap application  
		
		void add_noise(Frame &frame, float nosie_max_t, float noise_max_r);

		float getHeading(vector<IMU_data> &imu_datas, float delta_time_second);
		float getDistance(Eigen::Vector3f &position1, Eigen::Vector3f &position2);
        
		float getMAE(SubMap & submap);

	private:

	};
}
#endif //F_CONSTRAINT_H