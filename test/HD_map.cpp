#include "dataio.cpp"
#include "find_constraint.cpp"
#include "utility.h"


using namespace std;
using namespace utility;
using namespace Eigen;

typedef pcl::PointXYZINormal Point_T; 

int main(int argc,char** argv)
{
    if (argc != 9) {
       printf("Input error, should be: binary pcd_folder pcd_filelist pose_file imu_file config_file output_folder begin_frame end_frame\n");
       return -1;
    }
    
    cout<<"!--------------------Pose Optimization------------------!"<<endl;
    
    google::InitGoogleLogging("Mylog");
	google::SetLogDestination(google::GLOG_INFO, "/data/data/");
	LOG(INFO) << "Launch the program!";

    string pcd_folder = argv[1];
    string pcd_filelist =argv[2];
    string pose_file = argv[3];
    string imu_file = argv[4];
    string config_file = argv[5];
    string output_folder = argv[6];
    int begin_frame_id = atoi(argv[7]);
    int end_frame_id = atoi(argv[8]);

    //Read the config file (json) 
    int max_frame_num = 5000; //25
    float max_tran_dis= 5000;     //meter  //10
    float max_heading_angle=5000;  //degree  //8
    int interval_submap_num=8;
    float max_distance_between_submaps=15.0;
    float min_iou = 0.3;
    color_type display_color_mode=HEIGHT;

	//Timing
	clock_t t0, t1, t2, t3, t4, t5, t6;

    //Import Data
    DataIo <Point_T> io;
    Transaction trans1_data; //A transcation's data
    io.HDmap_data_import(pcd_folder, pcd_filelist, pose_file, imu_file, begin_frame_id, end_frame_id, trans1_data);
    
    //Initialize Submaps 
    Constraint_Finder cf;
    cf.divide_submap(trans1_data,max_frame_num,max_tran_dis,max_heading_angle);
    
    // Frontend for each submap (multi-thread)
    cout<<"Begin front end."<<endl;
    cf.front_end(trans1_data);

    //Backend
    //You need to get the submap first
    cout<<"Begin back end."<<endl;
    cf.back_end(trans1_data,interval_submap_num,max_distance_between_submaps,min_iou);
    
    // Output Graph Optimization's input
    io.writePoseGraph(output_folder, trans1_data);
    io.writeOdomPose(output_folder, trans1_data);
    
    
    //TO DO LIST
    //1. Use other aligner method (switch pcl's regsitration)
    //2. Install CUDA and try the parallel caculation (Jinwei)
    //3. Add the Active Object Removing Module (Jinwei)
    //4. Better Encapsulation
    //5. Combine with Bojun's Codes
    //6. Display the optimization result and do the comparsion
    //7. Graph Optimization
    //8. Loop closure detection (special cases: bad gps)
    //9. Add different initial guess
 



    google::ShutdownGoogleLogging();
    
    return 1;
}
