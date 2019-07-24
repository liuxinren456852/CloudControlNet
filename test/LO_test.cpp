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
    
    cout<<"!------------------Lidar Odometry Test------------------!"<<endl;
    
    google::InitGoogleLogging("Mylog");
	google::SetLogDestination(google::GLOG_INFO, "/data/data/log");
	LOG(INFO) << "Launch the program!";

    string pcd_folder = argv[1];
    string pcd_filelist =argv[2];
    string pose_file = argv[3];
    string imu_file = argv[4];
    string config_file = argv[5];
    string output_folder = argv[6];
    int begin_frame_id = atoi(argv[7]);
    int end_frame_id = atoi(argv[8]);

    int max_frame_num = 999999; 
    float max_tran_dis = 999999;    
    float max_heading_angle = 999999;  

    //Import Data
    DataIo <Point_T> io;
    Transaction trans1_data; //A transcation's data
    io.HDmap_data_import(pcd_folder, pcd_filelist, pose_file, imu_file, begin_frame_id, end_frame_id, trans1_data);
    
    //Initialize Submaps 
    Constraint_Finder cf;
    cf.divide_submap(trans1_data,max_frame_num,max_tran_dis,max_heading_angle);
    
    // Frontend for each submap (multi-thread)
    cout<<"Begin front end."<<endl;
    cf.lidar_odom_test(trans1_data);
    
    google::ShutdownGoogleLogging();
    
    return 1;
}
