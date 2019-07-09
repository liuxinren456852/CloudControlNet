#ifndef GLOBAL_OPTIMIZER_H
#define GLOBAL_OPTIMIZER_H


// #include <g2o/core/block_solver.h>
// #include <g2o/core/sparse_optimizer.h>
// #include <g2o/solvers/eigen/linear_solver_eigen.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/types/sba/types_six_dof_expmap.h>
// #include <g2o/core/robust_kernel.h>
// #include <g2o/core/robust_kernel_impl.h>
// //#include <g2o/core/base_unary_edge.h>

#include "find_constraint.h"

using namespace std;
using namespace utility;
//using namespace Eigen;

class GlobalOptimize
{
public:
	void optimizePoseGraph(vector<CloudBlock> &all_blocks, vector<Constraint> &all_cons);
	float determineWeight(int node0_type, int node1_type, int edge_type);

protected:

private:

};



#endif //GLOBAL_OPTIMIZER_H