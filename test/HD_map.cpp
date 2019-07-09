#include "dataio.cpp"
#include "common_reg.cpp"
#include "cloudprocessing.cpp"
#include "find_constraint.cpp"
#include "utility.h"
#include "parameters.h"
#include "graph_optimizer.h"

using namespace std;
using namespace utility;
using namespace Eigen;

typedef pcl::PointXYZI PointT; 

int main(int argc,char** argv)
{
    if (argc != 6) {
       printf("Input error, should be: binary pcd_folder pcd_filelist pose_file config_file output_folder\n");
       return -1;
    }
    
    cout<<"!--------------Pose Optimization-------------!"<<endl;

    string pcd_folder = argv[1];
    string pcd_filelist =argv[2];
    string pose_file = argv[3];
    string config_file = argv[4];
    string output_folder = argv[5];
	
    //Read the config file

	//Timing
	clock_t t0, t1, t2, t3, t4, t5, t6;

	//Read the point clouds' filename bounding box data 
	t0 = clock();
	DataIo <PointT> io;
    vector<Frame> HDmap_frames;
    io.HDmap_data_import(pcd_filelist,pose_file,HDmap_frames);
    
    Constraint_Finder cf;
    vector<Edge_between_2Frames> HDmap_edges;
    cf.batch_find_hdmap_constraints(HDmap_frames, HDmap_edges, 50, 25.0);  // interval frame , max_distance
	
    cout << "Ready to display ..." << endl;
	io.display_hdmap_edges(HDmap_edges);

    return 1;
}
