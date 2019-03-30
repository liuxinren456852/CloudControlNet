#ifndef F_CONSTRAINT_H
#define F_CONSTRAINT_H

#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;

namespace utility
{
	struct CloudBlock
	{
		int strip_num;      //航带序号,对于非ALS点云，默认为0;
		int num_in_strip;   //一条航带内的块序号;
		int data_type;      //哪种平台采集的点云：1.机载  2.地面站  3.车载  4.背包;
		Bounds bound;       //Bounding Box
		CenterPoint cp;     //Center Point

	};

	struct Constraint
	{
		CloudBlock block1, block2;
		int con_type;
	};

	class Constraint_Finder
	{
	public:
		// Constraint Type 1
		void find_strip_adjacent_constraint(vector<vector<CloudBlock>> &blocks_all, vector<Constraint> &innerstrip_cons_all);
		void find_adjacent_constraint_in_strip(vector<CloudBlock> &blocks_strip, vector<Constraint> &innerstrip_cons);

		// Constraint Type 2
		void find_overlap_registration_constraint(vector<CloudBlock> &blocks, vector<Constraint> &registration_cons_all, int k_search_neighbo, double min_iou);



	protected:
		void find_neighbor_k_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, int k);
		void find_neighbor_r_cps(CenterPoint &cp_search, vector<CenterPoint> &cp_points, float r);

		double calculate_iou(Bounds & bound1, Bounds & bound2);
		bool judge_adjacent(CloudBlock & block1, CloudBlock & block2);
	private:

	};
}
#endif //F_CONSTRAINT_H