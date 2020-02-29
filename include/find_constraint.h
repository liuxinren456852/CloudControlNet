#ifndef F_CONSTRAINT_H
#define F_CONSTRAINT_H

#include <vector>
#include "utility.h"

using namespace std;
using namespace Eigen;

namespace ccn
{
	struct CloudBlock  //���ƿ飨վ��;
	{
		int unique_index;   //������Block�е�Ψһʶ���;
		int strip_num;      //�������,���ڷ�ALS���ƣ�Ĭ��Ϊ0;
		int num_in_strip;   //һ�������ڵĿ����;
		int data_type;      //����ƽ̨�ɼ��ĵ��ƣ�1.����  2.����վ  3.����  4.����;
		Bounds bound;       //Bounding Box
		CenterPoint cp;     //Center Point
		Eigen::Matrix4f optimized_pose;   //Transformation Matrix after optimization
	};

	struct Constraint //Լ����;
	{
		CloudBlock block1, block2; //�����ӵ��������ƿ�;
		int con_type;              //Լ�����ͣ�1.�ڽ�ƽ�� 2.�ص���׼;
		Eigen::Matrix4f Trans1_2 = Eigen::Matrix4f::Identity(4,4);  //Լ��pose�仯�۲�;
		float confidence;          //�ñ任�����Ŷ�;
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