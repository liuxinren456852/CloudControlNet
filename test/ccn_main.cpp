#include "dataio.hpp"
#include "find_constraint.h"
#include "utility.h"
#include "filter.hpp"
#include "map_viewer.hpp"
#include "graph_optimizer.h"
#include "common_reg.hpp"
#include <glog/logging.h>

using namespace std;
using namespace ccn;

typedef pcl::PointXYZ Point_T;

int main(int argc, char **argv)
{

	cout << "!----------------------------------------------------------------------------!" << endl;
	cout << "!                            Cloud Control Net                               !" << endl;
	cout << "!----------------------------------------------------------------------------!" << endl;

	google::InitGoogleLogging("Mylog");
	google::SetLogDestination(google::GLOG_INFO, "./log/MyLogInfo");
	LOG(INFO) << "Launch the program!";

	//Import configuration
	//Data path (4 formats are available: *.pcd, *.las, *.ply, *.txt)
	std::string ALS_folder = argv[1];
	std::string TLS_folder = argv[2];
	std::string MLS_folder = argv[3];
	std::string BPLS_folder = argv[4];

	int find_constraint_knn = atoi(argv[5]);
	float find_constraint_overlap_ratio = atof(argv[6]);
	float downsample_voxels_size_als = atof(argv[7]);
	float downsample_voxels_size_tls = atof(argv[8]);
	float downsample_voxels_size_mls = atof(argv[9]);
	float downsample_voxels_size_bpls = atof(argv[10]);
	float registration_overlap_search_radius = atof(argv[11]);
	float registration_min_overlap_ratio = atof(argv[12]);
	float registration_perturbate_dis = atof(argv[13]);
	bool registration_use_reciprocal_corres = atoi(argv[14]);
	bool registration_use_use_trimmed_rejector = atoi(argv[15]);

	//Timing
	clock_t t0, t1, t2, t3, t4, t5, t6;

	//Preprocessing
	//Unify the coordinate system
	//geo.BLH2XYZ_WGS84() //...
	//Divide the point cloud into blocks by time sequence
	//io.ALS_block_by_time() //...

	//Read the point clouds' filename bounding box data
	t0 = clock();
	DataIo<Point_T> io;
	vector<vector<string>> ALS_strip_files;
	vector<string> TLS_files, MLS_files, BPLS_files;
	io.batchReadMultiSourceFileNamesInDataFolders(ALS_folder, TLS_folder, MLS_folder, BPLS_folder, ALS_strip_files, TLS_files, MLS_files, BPLS_files);

	//Read the point clouds' bounding box data
	strips ALS_strip_blocks(ALS_strip_files.size());
	strip TLS_blocks(TLS_files.size()), MLS_blocks(MLS_files.size()), BPLS_blocks(BPLS_files.size()), All_blocks;
	io.batchReadMultiSourceLasBlock(ALS_strip_files, TLS_files, MLS_files, BPLS_files, ALS_strip_blocks, TLS_blocks, MLS_blocks, BPLS_blocks, All_blocks);

	//Pre-construct the global pose graph
	Constraint_Finder cf;
	constraints All_cons, ALS_inner_strip_cons_all, MLS_adjacent_cons, BPLS_adjacent_cons, registration_cons;
	cf.batch_find_multisource_constranits(ALS_strip_blocks, TLS_blocks, MLS_blocks, BPLS_blocks, All_blocks,
										  ALS_inner_strip_cons_all, MLS_adjacent_cons, BPLS_adjacent_cons, registration_cons, All_cons,
										  find_constraint_knn, find_constraint_overlap_ratio);
	t1 = clock();
	LOG(INFO) << "Pose graph pre-construction done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s.";

	//Display the bounding box graph and the pose graph. Click "X" on the right-top corner to move on.
	MapViewer<Point_T> mviewer;
	cout << "Ready to display ..." << endl;
	mviewer.display2Dboxes(All_blocks);
	LOG(INFO) << "Display the bounding box graph!";
	mviewer.display2Dcons(All_cons);
	LOG(INFO) << "Display the pose graph!";

	//Accomplish the pairwise point cloud block(station) registration according to the global pose graph
	t2 = clock();
	cout << "Begin Overlapping Registration ..." << endl;
	CRegistration<Point_T> reg;
	int all_cons_number_pre = All_cons.size();
	int reg_cons_number_pre = registration_cons.size();
	int removed_cons_number = 0;
	int reg_cons_number_post;

	CFilter<Point_T> cfilter;
	for (int i = all_cons_number_pre - reg_cons_number_pre; i < all_cons_number_pre; i++) // For all the registration cons
	{
		//Import point cloud pair
		pcXYZPtr cloud1(new pcXYZ()), cloud2(new pcXYZ());
		string Filename1, Filename2;
		io.readLasCloudPairfromCon(All_cons[i], ALS_strip_files, TLS_files, MLS_files, BPLS_files, Filename1, Filename2, cloud1, cloud2);
		//Down-sample point cloud pair
		pcXYZPtr subcloud1(new pcXYZ()), subcloud2(new pcXYZ()), subcloud1_perturbated(new pcXYZ());
		cfilter.batchdownsamplepair(All_cons[i], cloud1, cloud2, subcloud1, subcloud2,
									downsample_voxels_size_als, downsample_voxels_size_tls, downsample_voxels_size_mls, downsample_voxels_size_bpls);
		//Pairwise registration using trimmed-ICP with/without perturbation
		if (!reg.add_registration_edge(subcloud1, subcloud2, All_cons[i],
									   registration_perturbate_dis, 30, registration_use_reciprocal_corres, registration_use_use_trimmed_rejector,
									   registration_overlap_search_radius, 10, registration_min_overlap_ratio))
			removed_cons_number++;
	}
	reg_cons_number_post = reg_cons_number_pre - removed_cons_number;
	t3 = clock();
	LOG(INFO) << "All the required constraint edges has been assigned with the Transformation";
	LOG(INFO) << "Pairwise point cloud registration done in " << float(t3 - t2) / CLOCKS_PER_SEC << " s.";
	cout << "All the required constraint edges has been assigned with the Transformation, Number is " << reg_cons_number_post << endl;

	// //Graph optimization
	cout << "Waiting For the Next Step: Graph Optimization" << endl;
	GlobalOptimize my_go;
	my_go.optimizePoseGraph(All_blocks, All_cons);
	t4 = clock();
	LOG(INFO) << "Pose graph optimization done in " << float(t4 - t3) / CLOCKS_PER_SEC << " s.";
	LOG(INFO) << "Graph Optimization Finished, recalculate the point cloud";

	// //Recalculation (To be detailed)
	t5 = clock();
	cout << "Recalculate the point cloud" << endl;

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

//testing code

//TO DO
//1.improve the registration efficieny
//2.assign weight according to the posterior covariance matrix 
//3.robust iterative least square


/*
	//����������׼;
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
	float overlap_search_radius = 5.0;   // The search radius in meter for overlapping ratio estimation [This is a subtle parameter, ��ֵԽ�ã���ֵ���ԽС;]
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
//ALS2ALS  ���ICP 10s    ����ICP 15s    GICP  19s
//ALS2TLS  ���ICP 25s    ����ICP 35s    GICP  40s
//TLS2TLS  ���ICP 35s    ����ICP 40s    GICP  45s

/*
//���ȼ�ˣ���RMSE; (ԭ15cm����);
DataIo <pcl::PointXYZ> io;
std::vector <std::vector<double>> coordinatesA;
std::vector <std::vector<double>> coordinatesB;
std::vector<double> transpara;
io.read_XYZ_XYZlist(coordinatesA, coordinatesB);
double RMSE;
RMSE = io.cal_cor_RMSE(coordinatesA, coordinatesB);
*/

/*
//��˹ͶӰתUTMͶӰ;
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
//������������任;
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
//������������ƽ��;
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
//7&4��������ת��;
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
//����ϵͶӰ�任;
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
//����ʱ��ֿ�;
cout << "Please make sure all the ALS, MLS and BPLS point clouds are divided into blocks." << endl;
cout << "If they are not divided, you can divide them now" << endl;
float time_of_block;
string ALS_folder;
ALS_folder = "I:\\Data\\���ٹ�·��������Ŀ-����\\ALS\\ALS_strips"; //

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
//����������׼;
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
float overlap_search_radius = 5.0;   // The search radius in meter for overlapping ratio estimation [This is a subtle parameter, ��ֵԽ�ã���ֵ���ԽС;]
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
// ͬ��������ת��;
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
//�ֶ���������ļ���;
//cout << "Input ALS Point Cloud (Blocks of all the strips) Folder:" << endl;
//cin >> ALS_folder;
//cout << "Input TLS Point Cloud Folder:" << endl;
//cin >> TLS_folder;
//cout << "Input MLS Point Cloud (Blocks) Folder:" << endl;
//cin >> MLS_folder;
//cout << "Input BPLS Point Cloud (Blocks) Folder:" << endl;
//cin >> BPLS_folder;
*/
