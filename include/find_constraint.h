#ifndef _INCLUDE_FIND_CONSTRAINT_H
#define _INCLUDE_FIND_CONSTRAINT_H

#include <vector>
#include "utility.h"

using namespace std;

namespace ccn
{
	class Constraint_Finder
	{
	public:
		// Constraint Type 1
		void find_strip_adjacent_constraint(vector<vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>>> &blocks_all, vector<constraint_t, Eigen::aligned_allocator<constraint_t>> &innerstrip_cons_all);
		void find_adjacent_constraint_in_strip(vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &blocks_strip, vector<constraint_t, Eigen::aligned_allocator<constraint_t>> &innerstrip_cons);

		// Constraint Type 2
		void find_overlap_registration_constraint(vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &blocks, vector<constraint_t, Eigen::aligned_allocator<constraint_t>> &registration_cons_all, int k_search_neighbo, double min_iou);

		// Batch Construct Constraints
		void batch_find_multisource_constranits(std::vector<std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>>> &ALS_strip_blocks, std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &TLS_blocks, std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &MLS_blocks, std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &BPLS_blocks, std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> &All_blocks,
			std::vector<constraint_t, Eigen::aligned_allocator<constraint_t>> &ALS_inner_strip_cons_all, std::vector<constraint_t, Eigen::aligned_allocator<constraint_t>> &MLS_adjacent_cons, std::vector<constraint_t, Eigen::aligned_allocator<constraint_t>> &BPLS_adjacent_cons, std::vector<constraint_t, Eigen::aligned_allocator<constraint_t>> &registration_cons, std::vector<constraint_t, Eigen::aligned_allocator<constraint_t>> &All_cons,
			int overlap_Registration_KNN, float overlap_Registration_OverlapRatio);

	protected:
		void find_neighbor_k_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, int k);
		void find_neighbor_r_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, float r);

		double calculate_iou(Bounds & bound1, Bounds & bound2);
		bool judge_adjacent(cloudblock_t & block1, cloudblock_t & block2);
	private:

	};
}
#endif //F_CONSTRAINT_H