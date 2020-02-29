#ifndef F_CONSTRAINT_H
#define F_CONSTRAINT_H

#include <vector>
#include "utility.h"

using namespace std;
using namespace Eigen;

namespace ccn
{
	struct CloudBlock  //点云块（站）;
	{
		int unique_index;   //在所有Block中的唯一识别号;
		int strip_num;      //航带序号,对于非ALS点云，默认为0;
		int num_in_strip;   //一条航带内的块序号;
		int data_type;      //哪种平台采集的点云：1.机载  2.地面站  3.车载  4.背包;
		Bounds bound;       //Bounding Box
		CenterPoint cp;     //Center Point
		Eigen::Matrix4f optimized_pose;   //Transformation Matrix after optimization
	};

	struct Constraint //约束边;
	{
		CloudBlock block1, block2; //所连接的两个点云块;
		int con_type;              //约束类型：1.邻接平滑 2.重叠配准;
		Eigen::Matrix4f Trans1_2 = Eigen::Matrix4f::Identity(4,4);  //约束pose变化观测;
		float confidence;          //该变换的置信度;
	};

	class Constraint_Finder
	{
	public:
		// Constraint Type 1
		void find_strip_adjacent_constraint(vector<vector<CloudBlock>> &blocks_all, vector<Constraint> &innerstrip_cons_all);
		void find_adjacent_constraint_in_strip(vector<CloudBlock> &blocks_strip, vector<Constraint> &innerstrip_cons);

		// Constraint Type 2
		void find_overlap_registration_constraint(vector<CloudBlock> &blocks, vector<Constraint> &registration_cons_all, int k_search_neighbo, double min_iou);

		// Batch Construct Constraints
		void batch_find_multisource_constranits(std::vector<std::vector<CloudBlock>> &ALS_strip_blocks, std::vector<CloudBlock> &TLS_blocks, std::vector<CloudBlock> &MLS_blocks, std::vector<CloudBlock> &BPLS_blocks, std::vector<CloudBlock> &All_blocks,
			std::vector<Constraint> &ALS_inner_strip_cons_all, std::vector<Constraint> &MLS_adjacent_cons, std::vector<Constraint> &BPLS_adjacent_cons, std::vector<Constraint> &registration_cons, std::vector<Constraint> &All_cons,
			int overlap_Registration_KNN, float overlap_Registration_OverlapRatio);

	protected:
		void find_neighbor_k_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, int k);
		void find_neighbor_r_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, float r);

		double calculate_iou(Bounds & bound1, Bounds & bound2);
		bool judge_adjacent(CloudBlock & block1, CloudBlock & block2);
	private:

	};
}
#endif //F_CONSTRAINT_H