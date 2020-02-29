#include "dataio.cpp"
#include "common_reg.cpp"
#include "cloudprocessing.cpp"
#include "find_constraint.h"
#include "parameters.h"
#include "graph_optimizer.h"
#include <glog/logging.h>


using namespace std;
using namespace ccn;

int main(/*int argc, char** argv*/)
{
	//整体流程;
	//前端;
	//1.建图;
	//2.按图配准;

	//ALS ALS subsample 0.5 0.5   search_dis 1.0  
	//ALS TLS subsample 0.6 0.2   search_dis 1.5

	//后端;
	//3.配准结果作为观测数据进行优化;
	//4.优化结果重解算;

	//To do list
	//加并行，提升效率;
	//ALS+ALS航带配准算法改进;
	//位姿图赋权设计;
	//点云重解算;
	//稳健性，鲁棒性增强;
	//精度自检较;

	cout << "!----------------------------------------------------------------------------!" << endl;
	cout << "!                               ALS Refinement                               !" << endl;
	cout << "!                              by Yue Pan et al.                             !" << endl;
	cout << "!----------------------------------------------------------------------------!" << endl;

	google::InitGoogleLogging("Mylog");
	google::SetLogDestination(google::GLOG_INFO, "./log/MyLogInfo");
	LOG(INFO) << "Launch the program!";
	
	/*
	//点云两两配准;
	// Input the filename (Or just drag in) by yourself
	string filenameA, filenameB;
	//cout << "Input ALS Point Cloud File:" << endl;  //ALS
	//cin >> filenameT;
	//cout << "Input TLS Point Cloud File:" << endl;  //TLS
	//cin >> filenameS;
	pcXYZPtr cloudA(new pcXYZ()), cloudB(new pcXYZ());
	DataIo <pcl::PointXYZ> io;
	//io.readCloudFile(filenameT, cloudT);
	//io.readCloudFile(filenameS, cloudS);
	//TLS+TLS test
	//io.readLasFile("7-1-TLS.las", cloudA, 1);
	//io.readLasFileLast("2-MLS.las", cloudB);
	//ALS+ALS test
	//io.readLasFile("1-3-11-ALS.las", cloudA, 1);
	//io.readLasFileLast("2-1-11-ALS.las", cloudB);
	//ALS+TLS test
	//io.readLasFile("4-1-TLS.las", cloudA, 1);
	//io.readLasFileLast("1-3-2-ALS.las", cloudB);
	io.readLasFile("5-1-TLS.las", cloudA, 1);
	io.readLasFileLast("1-3-14-ALS.las", cloudB);
	cout << "Read File Done..." << endl << "Raw point number: [ A:  " << cloudA->size() << "  , B:  " << cloudB->size() << " ]" << endl;
	// Filter the Point Cloud for registration
	pcXYZPtr subcloudA(new pcXYZ()), subcloudB(new pcXYZ());
	float downsample_resolution_A;  //Voxel Down-sampling's size (unit:m)
	float downsample_resolution_B;  //Voxel Down-sampling's size (unit:m)
	cout << "Input A's downsample resolution." << endl; cin >> downsample_resolution_A;
	cout << "Input B's downsample resolution." << endl; cin >> downsample_resolution_B;
	VoxelFilter<pcl::PointXYZ> vf_A(downsample_resolution_A); VoxelFilter<pcl::PointXYZ> vf_B(downsample_resolution_B);
	subcloudA = vf_A.filter(cloudA); subcloudB = vf_B.filter(cloudB);
	cout << "Down-sampled point cloud number: [ A:  " << subcloudA->size() << " , B:  " << subcloudB->size() << " ]" << endl;
	// Apply other filters here ...
	// Fine Registration of ALS and TLS point cloud
	cout << "Registration begin." << endl;
	int icp_type = 0;
	//cout << "Select the Registration Method." << endl;
	//cout << "1.Point-to-Point ICP   2.Point-to-Plane ICP   3.Generalized ICP  [default 1]" << endl;
	//cin >> icp_type;
	bool use_reciprocal_correspondence = 0;
	//cout << "Use Reciprocal Correspondences or not?  0. No  1. Yes  [default 0]" << endl;
	//cin >> use_reciprocal_correspondence;
	bool use_trimmed_rejector = 1;
	//cout << "Use Trimmed Correspondences Rejector or not?  0. No  1. Yes  [default 1]" << endl;
	//cin >> use_trimmed_rejector;
	CRegistration <pcl::PointXYZ> reg;
	pcXYZPtr regB(new pcXYZ()), regA(new pcXYZ());  // regS: Transformed Sub-sampled Source Cloud   regALS: Transformed Raw Source Cloud
	Eigen::Matrix4f TransA2B, TransB2A;                     // Trans: TLS->TLS' for ALS (Target)   TransALS: ALS->ALS'  for TLS (Target)
	// other parameters
	int max_iter_num = 50;               // Maximum iteration number before convergence
	float overlap_search_radius = 5.0;   // The search radius in meter for overlapping ratio estimation [This is a subtle parameter, 初值越好，该值设的越小;]
	int covariance_knn = 15;             // The number of neighbor points for covariance and normal calculation
	float covariance_radius = 1.5;       //
	float min_overlap_for_reg = 0.1;
	//float overlap_ratio = 0.4; 
	//Input Parameters
	//cout << "Max iteration number is: " << endl; cin >> max_iter_num;  //50 ?
	cout << "Overlap search radius is: " << endl; cin >> overlap_search_radius;  //1.0?
	//cout << "Covariance calculate radius is: " << endl; cin >> covariance_radius;
	//cout << "Overlap ratio is: " << endl; cin >> overlap_ratio;
	reg.ptplicp_reg(subcloudB, subcloudA, regB, TransB2A, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius, covariance_knn, min_overlap_for_reg);
	reg.icp_reg(subcloudB, subcloudA, regB, TransB2A, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius, min_overlap_for_reg);
	reg.gicp_reg(subcloudB, subcloudA, regB, TransB2A, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius, covariance_knn, min_overlap_for_reg);
	

	//Compare different registration method: ICPs on ALS2ALS and ALS2TLS testing cases
	//
	/*switch (icp_type)  // Do the registration
	{
	case 1:
		reg.icp_reg(subcloudS, subcloudT, regS, Trans, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius, 0.15);  // You can also use CloudT instead of subcloudT if efficiency is not your top priority
		break;
	case 2:
		cout << " Attention please, The Input Point Clouds must have normal." << endl;
		//regn.ptplicp_reg(subcloudS, cloudT, regS, Trans, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius);
		break;
	case 3:
		reg.gicp_reg(subcloudS, subcloudT, regS, Trans, covariance_knn, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius);
		break;
	default:
		reg.icp_reg(subcloudS, subcloudT, regS, Trans, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius,0.15);
		break;
	}*/
	//reg.invTransform(TransB2A, TransA2B);                //To get the transformation from ALS to ALS' with TLS as the target
	//reg.transformcloud(cloudA, regA, TransA2B);          //To get the transformed ALS
	//io.writeCloudFile("Acloud_after_registration.las", regA);
	//cout << "Write File Done..." << endl;
	//io.displaymulti(cloudS, cloudT, regALS, "TLS[R] ALS_before[B] ALS_transformed[G]"); // Display the result. Click "X" to continue.
	//ALS2ALS  点点ICP 10s    点面ICP 15s    GICP  19s
	//ALS2TLS  点点ICP 25s    点面ICP 35s    GICP  40s
	//TLS2TLS  点点ICP 35s    点面ICP 40s    GICP  45s

	
	//Set parameters or read the parameter configuration file.        
	ParameterList project_parameters;                                  //参数表解释;
	project_parameters.Overlap_Registration_KNN = 10;                  //建立重叠配准约束的点云块近邻搜索个数;
	project_parameters.Overlap_Registration_OverlapRatio = 0.2;        //建立重叠配准约束的点云块包围盒最小重叠率;
	project_parameters.Cloud_Registration_Downsampledis_ALS = 0.6;     //ALS机载点云抽稀体素大小(m);  //0.5 m
	project_parameters.Cloud_Registration_Downsampledis_TLS = 0.3;     //TLS地面站点云抽稀体素大小(m); //0.25 m
	project_parameters.Cloud_Registration_Downsampledis_MLS = 0.3;     //MLS车载点云抽稀体素大小(m);   //0.25 m
	project_parameters.Cloud_Registration_Downsampledis_BPLS = 0.3;   //BPLS背包点云抽稀体素大小(m);  //0.25 m
	project_parameters.Cloud_Registration_MaxIterNumber = 40;          //ICP配准最大迭代次数;
	project_parameters.Cloud_Registration_OverlapSearchRadius = 2.0;   //Trimmed-ICP重叠搜索半径(m);
	project_parameters.Cloud_Registration_MinOverlapForReg = 0.3;      //Trimmed-ICP配准允许最小重叠率;
	project_parameters.Cloud_Registration_UseReciprocalCorres = 0;     //配准是否使用双向最佳对应(0不用,1用);
	project_parameters.Cloud_Registration_UseTrimmedRejector = 1;      //配准是否使用Trimmed策略(0不用,1用);
	project_parameters.Cloud_Registration_PerturbateValue = 0.5;       //配准最大微扰量(m);
	project_parameters.Cloud_Registration_CovarianceK = 15;            //估算法向量所用邻域点数;

	//Timing
	clock_t t0, t1, t2, t3, t4, t5, t6;

	//Preprocessing
	//Unify the coordinate system 
	//geo.BLH2XYZ_WGS84() //...
	//Divide the point cloud into blocks by time sequence
	//io.ALS_block_by_time() //...

	//Set data folder or drag in the folder
	string ALS_folder, TLS_folder, MLS_folder, BPLS_folder;
	ALS_folder = "I:\\Data\\高速公路改扩建项目-数据\\局部2\\ALS"; 
	//ALS_folder = "I:\\Data\\高速公路改扩建项目-数据\\ALS\\blocks";
	TLS_folder = "I:\\Data\\高速公路改扩建项目-数据\\局部2\\TLS";
	//TLS_folder = "I:\\Data\\高速公路改扩建项目-数据\\TLS\\utm48-las";
	MLS_folder = "I:\\Data\\高速公路改扩建项目-数据\\局部2\\MLS";
	//MLS_folder = "I:\\Data\\高速公路改扩建项目-数据\\MLS\\UTM48_tran-las";
	BPLS_folder = "I:\\Data\\高速公路改扩建项目-数据\\局部2\\BPLS1";
	//BPLS_folder = "I:\\Data\\高速公路改扩建项目-数据\\BPLS\\UTM48_tran-las";
	
	//Read the point clouds' filename bounding box data 
	t0 = clock();
	DataIo <pcl::PointXYZ> io;
	vector<vector<string>> ALS_strip_files;
	vector<string> TLS_files, MLS_files, BPLS_files;
	io.batchReadMultiSourceFileNamesInDataFolders(ALS_folder, TLS_folder, MLS_folder, BPLS_folder, ALS_strip_files, TLS_files, MLS_files, BPLS_files);
	
	//Read the point clouds' bounding box data 
	vector<vector<CloudBlock>> ALS_strip_blocks(ALS_strip_files.size());
	vector<CloudBlock> TLS_blocks(TLS_files.size());
	vector<CloudBlock> MLS_blocks(MLS_files.size());
	vector<CloudBlock> BPLS_blocks(BPLS_files.size());
	vector<CloudBlock> All_blocks;
	io.batchReadMultiSourceLasBlock(ALS_strip_files, TLS_files, MLS_files, BPLS_files, ALS_strip_blocks, TLS_blocks, MLS_blocks, BPLS_blocks, All_blocks);
     
	//Pre-construct the global pose graph
	Constraint_Finder cf;
	vector<Constraint> All_cons, ALS_inner_strip_cons_all, MLS_adjacent_cons, BPLS_adjacent_cons, registration_cons;
	cf.batch_find_multisource_constranits(ALS_strip_blocks, TLS_blocks, MLS_blocks, BPLS_blocks, All_blocks, ALS_inner_strip_cons_all, MLS_adjacent_cons, BPLS_adjacent_cons, registration_cons, All_cons, project_parameters.Overlap_Registration_KNN, project_parameters.Overlap_Registration_OverlapRatio);
	t1 = clock();
	LOG(INFO) << "Pose graph pre-construction done in "<< float(t1 - t0) / CLOCKS_PER_SEC << " s.";
	
	//Display the bounding box graph and the pose graph. Click "X" on the right-top corner to move on.
	cout << "Ready to display ..." << endl;
	io.display2Dboxes(All_blocks);
	LOG(INFO) << "Display the bounding box graph!";
	io.display2Dcons(All_cons);
	LOG(INFO) << "Display the pose graph!";
	
	//Accomplish the pairwise point cloud block(station) registration according to the global pose graph
	t2 = clock();
	cout << "Begin Overlapping Registration ..." << endl;
	CRegistration <pcl::PointXYZ> reg;
	int all_cons_number_pre = All_cons.size(); int reg_cons_number_pre = registration_cons.size();
	int removed_cons_number = 0; int reg_cons_number_post;

	for (int i = all_cons_number_pre - reg_cons_number_pre; i < all_cons_number_pre; i++) // For all the registration cons
	{
		//Import point cloud pair
		pcXYZPtr cloud1(new pcXYZ()), cloud2(new pcXYZ());
		string Filename1, Filename2;
		io.readLasCloudPairfromCon(All_cons[i], ALS_strip_files, TLS_files, MLS_files, BPLS_files, Filename1, Filename2, cloud1, cloud2);
		//Down-sample point cloud pair
		pcXYZPtr subcloud1(new pcXYZ()), subcloud2(new pcXYZ()), subcloud1_perturbated(new pcXYZ());
		io.batchdownsamplepair(All_cons[i], cloud1, cloud2, subcloud1, subcloud2, project_parameters.Cloud_Registration_Downsampledis_ALS, project_parameters.Cloud_Registration_Downsampledis_TLS, project_parameters.Cloud_Registration_Downsampledis_MLS, project_parameters.Cloud_Registration_Downsampledis_BPLS);
		//Pairwise registration using trimmed-ICP with/without perturbation
		if (!reg.add_registration_edge(subcloud1, subcloud2, All_cons[i], project_parameters.Cloud_Registration_PerturbateValue, project_parameters.Cloud_Registration_MaxIterNumber, project_parameters.Cloud_Registration_UseReciprocalCorres, project_parameters.Cloud_Registration_UseTrimmedRejector, project_parameters.Cloud_Registration_OverlapSearchRadius, project_parameters.Cloud_Registration_CovarianceK, project_parameters.Cloud_Registration_MinOverlapForReg)) removed_cons_number++;
	}
	reg_cons_number_post = reg_cons_number_pre - removed_cons_number;
	t3 = clock();
	LOG(INFO) << "All the required constraint edges has been assigned with the Transformation";
	LOG(INFO) << "Pairwise point cloud registration done in " << float(t3 - t2) / CLOCKS_PER_SEC << " s.";
	cout << "All the required constraint edges has been assigned with the Transformation, Number is " << reg_cons_number_post << endl;
	
	//Graph optimization
	cout << "Waiting For the Next Step: Graph Optimization" << endl;
	GlobalOptimize my_go;
	my_go.optimizePoseGraph(All_blocks,All_cons);
	t4 = clock();
	LOG(INFO) << "Pose graph optimization done in " << float(t4 - t3) / CLOCKS_PER_SEC << " s.";
	LOG(INFO) << "Graph Optimization Finished, recalculate the point cloud";
	
	//Recalculation (To be detailed)
	t5 = clock();
	cout << "Recalculate the point cloud"<<endl;
	
	//Output the final result;
	cout << "Output the final result" << endl;
	io.batchwritefinalcloud(All_blocks, ALS_strip_files, TLS_files, MLS_files, BPLS_files); //Refined point cloud results would be output in the same folder of input data.
	t6 = clock();
	LOG(INFO) << "Point cloud refinement result output done in " << float(t6 - t5) / CLOCKS_PER_SEC << " s.";
	cout << "Output Finished, Check the accuracy." << endl;
	

    google::ShutdownGoogleLogging();
    bool endindex;
    cin >> endindex;
    return 1;
}




//废弃测试代码;

/*
//精度检核，求RMSE; (原15cm左右);
DataIo <pcl::PointXYZ> io;
std::vector <std::vector<double>> coordinatesA;
std::vector <std::vector<double>> coordinatesB;
std::vector<double> transpara;
io.read_XYZ_XYZlist(coordinatesA, coordinatesB);
double RMSE;
RMSE = io.cal_cor_RMSE(coordinatesA, coordinatesB);
*/

/*
//高斯投影转UTM投影;
string foldername;
cout << "Input Folder Name" << endl;
cin >> foldername;

vector<string> filenames;
DataIo <pcl::PointXYZI> io;
io.batchReadFileNamesInFolders(foldername, ".las", filenames);
cout << "Read filename done" << endl;
for (int i = 0; i < filenames.size(); i++)
{
	io.lasfileGK2UTM(filenames[i]);
	cout << "File " << i << " Procession done ..." << endl;
}
*/

/*
//点云坐标整体变换;
string foldername;
cout << "Input Folder Name" << endl;
cin >> foldername;

vector<string> filenames;
DataIo <pcl::PointXYZI> io;
io.batchReadFileNamesInFolders(foldername, ".las", filenames);
cout << "Read filename done" << endl;
for (int i = 0; i < filenames.size(); i++)
{
	
	//io.lasfileGK2UTM(filenames[i]);
	cout << "File " << i << " Procession done ..." << endl;
}
*/

/*
//点云坐标整体平移;
string foldername;
cout << "Input Folder Name" << endl;
cin >> foldername;

vector<double> shift(3);
shift[0] = 28.890503;
shift[1] = -1364.721558;
shift[2] = 0;

vector<string> filenames;
DataIo <pcl::PointXYZI> io;
io.batchReadFileNamesInFolders(foldername, ".las", filenames);
cout << "Read filename done" << endl;
for (int i = 0; i < filenames.size(); i++)
{
    io.lasfileshift(filenames[i], shift);
    cout << "File " << i << " Procession done ..." << endl;
}
*/

/*
//7&4参数坐标转换;
DataIo <pcl::PointXYZ> io;
std::vector <std::vector<double>> coordinatesA;
std::vector <std::vector<double>> coordinatesB;
std::vector<double> transpara;
io.read_XYZ_XYZlist(coordinatesA, coordinatesB);
CRegistration <pcl::PointXYZ> reg;
//7 Parameters: X Y Z roll pitch yaw scale
reg.CSTRAN_7DOF(coordinatesA, coordinatesB, transpara, 20); //20: point pair number for transform estimation;
io.XYZ_7DOFCSTran(transpara);
//4 Parameters: X Y yaw scale
//reg.CSTRAN_4DOF(coordinatesA, coordinatesB, transpara, 20); //20: point pair number for transform estimation;
//io.XYZ_4DOFCSTran(transpara);
*/

/*
//坐标系投影变换;
cout << "Processing the engineering coordinate system" << endl;
float center_long_eng;
cout << "Please enter the projection center longitude" << endl;
cin >> center_long_eng;
cout << "Please enter the projection surface's height" << endl;
float proj_surface_h_eng;
cin >> proj_surface_h_eng;
//cout << center_long_eng << endl;
io.tran_wgs2eng(center_long_eng,proj_surface_h_eng);
*/

/*
//点云时序分块;
cout << "Please make sure all the ALS, MLS and BPLS point clouds are divided into blocks." << endl;
cout << "If they are not divided, you can divide them now" << endl;
float time_of_block;
string ALS_folder;
ALS_folder = "I:\\Data\\高速公路改扩建项目-数据\\ALS\\ALS_strips"; //

cout << "Please input the time step of a block in second" << endl;
cin >> time_of_block;
DataIo <pcl::PointXYZI> io;
vector<string> ALS_filenames;
io.batchReadFileNamesInFolders(ALS_folder, ".las", ALS_filenames);
cout << "Read Filenames Done..." << endl;
for (int i = 0; i < ALS_filenames.size(); i++)
{
pcXYZIPtr cloudA(new pcXYZI());
vector<pcXYZI> cloud_blocks;
io.readLasFile(ALS_filenames[i], cloudA, 1);
io.ALS_block_by_time(cloudA, cloud_blocks, time_of_block);
cout << "Divide ALS strip into blocks Done for" << endl << ALS_filenames[i] << endl;
io.batchWriteBlockInColor(ALS_filenames[i], cloud_blocks, 1);
cout << "Write File Done..." << endl;
}
*/

/*
//点云两两配准;
// Input the filename (Or just drag in) by yourself
string filenameS, filenameT;
//cout << "Input ALS Point Cloud File:" << endl;  //ALS
//cin >> filenameT;
//cout << "Input TLS Point Cloud File:" << endl;  //TLS
//cin >> filenameS;
pcXYZIPtr cloudT(new pcXYZI()), cloudS(new pcXYZI());
DataIo <pcl::PointXYZI> io;
//io.readCloudFile(filenameT, cloudT);
//io.readCloudFile(filenameS, cloudS);
io.readCloudFile("31_ALS_Block.pcd", cloudT);
io.readCloudFile("tls7-1-utm.pcd", cloudS);
cout << "Read File Done..." << endl << "Raw point number: [ S:  " << cloudS->size() << "  , T:  " << cloudT->size() << " ]" << endl;
// Filter the Point Cloud for registration
pcXYZIPtr subcloudT(new pcXYZI()), subcloudS(new pcXYZI());
float downsample_resolution_ALS;  //Voxel Down-sampling's size (unit:m)
float downsample_resolution_TLS;  //Voxel Down-sampling's size (unit:m)
cout << "Input ALS's downsample resolution." << endl; cin >> downsample_resolution_ALS;
cout << "Input TLS's downsample resolution." << endl; cin >> downsample_resolution_TLS;
VoxelFilter<pcl::PointXYZI> vf_ALS(downsample_resolution_ALS);
VoxelFilter<pcl::PointXYZI> vf_TLS(downsample_resolution_TLS);
subcloudT = vf_ALS.filter(cloudT);
subcloudS = vf_TLS.filter(cloudS);
cout << "Down-sampled point cloud number: [ S:  " << subcloudS->size() << " , T:  " << subcloudT->size() << " ]" << endl;
// Apply other filters here ...
// Fine Registration of ALS and TLS point cloud
cout << "Registration of TLS and ALS." << endl;
int icp_type = 0;
cout << "Select the Registration Method." << endl;
cout << "1.Point-to-Point ICP   2.Point-to-Plane ICP   3.Generalized ICP  [default 1]" << endl;
cin >> icp_type;
bool use_reciprocal_correspondence = 0;
cout << "Use Reciprocal Correspondences or not?  0. No  1. Yes  [default 0]" << endl;
cin >> use_reciprocal_correspondence;
bool use_trimmed_rejector = 1;
cout << "Use Trimmed Correspondences Rejector or not?  0. No  1. Yes  [default 1]" << endl;
cin >> use_trimmed_rejector;
CRegistration <pcl::PointXYZI> reg;
CRegistration <pcl::PointXYZINormal> regn;
pcXYZIPtr regS(new pcXYZI()), regALS(new pcXYZI());  // regS: Transformed Sub-sampled Source Cloud   regALS: Transformed Raw Source Cloud
Eigen::Matrix4f Trans, TransALS;                     // Trans: TLS->TLS' for ALS (Target)   TransALS: ALS->ALS'  for TLS (Target)
// other parameters
int max_iter_num = 100;               // Maximum iteration number before convergence
float overlap_search_radius = 5.0;   // The search radius in meter for overlapping ratio estimation [This is a subtle parameter, 初值越好，该值设的越小;]
int covariance_knn = 15;             // The number of neighbor points for covariance and normal calculation
// Input Parameters
cout << "Max iteration number is: " << endl; cin >> max_iter_num;
cout << "Overlap search radius is: " << endl; cin >>overlap_search_radius;
switch (icp_type)  // Do the registration
{
case 1:
reg.icp_reg(subcloudS, subcloudT, regS, Trans, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius);  // You can also use CloudT instead of subcloudT if efficiency is not your top priority
break;
case 2:
cout << " Attention please, The Input Point Clouds must have normal." << endl;
//regn.ptplicp_reg(subcloudS, cloudT, regS, Trans, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius);
break;
case 3:
reg.gicp_reg(subcloudS, subcloudT, regS, Trans, covariance_knn, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius);
break;
default:
reg.icp_reg(subcloudS, subcloudT, regS, Trans, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius);
break;
}
reg.invTransform(Trans,TransALS);                      //To get the transformation from ALS to ALS' with TLS as the target
reg.transformcloud(cloudT, regALS, TransALS);          //To get the transformed ALS
io.writeCloudFile("ALS_after_registration.las", regALS);
cout << "Write File Done..." << endl;
//io.displaymulti(cloudS, cloudT, regALS, "TLS[R] ALS_before[B] ALS_transformed[G]"); // Display the result. Click "X" to continue.
*/

/*
// 同名点坐标转换;
// For TLS test
// Input the filename (Or just drag in) by yourself
std::vector <std::vector<double>> coordinatesA;
std::vector <std::vector<double>> coordinatesB;
Matrix4d  TransMatrixA2B;
DataIo <pcl::PointXYZ> io;
cout << "Would you like to import the point cloud with tie points indices or just import the correspondence coordinates?" << endl
<< "0. Import Point Clouds and the Tie points Indices List" << endl
<< "1. Import the Correspondence Coordinates List" << endl
<< "Please type in 0 or 1 to make a choice [default 1]" << endl;
int whichmethod; cin >> whichmethod;
if (whichmethod == 0)
{
string filenameA, filenameB;
pcXYZPtr cloudA(new pcXYZ()), cloudB(new pcXYZ());
cout << "Input base TLS Point Cloud File:" << endl;       //TLS A
cin >> filenameA;
io.readCloudFile(filenameA, cloudA);
cout << "Input practical TLS Point Cloud File:" << endl;  //TLS B
cin >> filenameB;
io.readCloudFile(filenameB, cloudB);
io.readindiceslist(cloudA, cloudB, coordinatesA, coordinatesB);
}
else
{
io.read_XYZ_BLHlist(coordinatesA, coordinatesB);
//io.read_XYZ_XYZlist(coordinatesA, coordinatesB);
}
CRegistration <pcl::PointXYZ> reg;
reg.LLS_4DOF(coordinatesA, coordinatesB, TransMatrixA2B, 2, 0.); //[Linear Least Square Transformation Esitmator]  Use 2 control points to do the calculation; The initial yaw angle guess [in degree] can be set causually (here it's 0.)
//reg.SVD_6DOF(coordinatesA, coordinatesB, TransMatrixA2B, 3);
cout << "Procession Done ..." << endl;
*/

/*
//手动拖入点云文件夹;
//cout << "Input ALS Point Cloud (Blocks of all the strips) Folder:" << endl;
//cin >> ALS_folder;
//cout << "Input TLS Point Cloud Folder:" << endl;
//cin >> TLS_folder;
//cout << "Input MLS Point Cloud (Blocks) Folder:" << endl;
//cin >> MLS_folder;
//cout << "Input BPLS Point Cloud (Blocks) Folder:" << endl;
//cin >> BPLS_folder;
*/

