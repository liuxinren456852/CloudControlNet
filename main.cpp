#include "dataio.cpp"
#include "common_reg.cpp"
#include "cloudprocessing.cpp"
#include "voxelFilter.h"
#include "find_constraint.h"



using namespace std;
using namespace utility;

int main(/*int argc, char** argv*/)
{

	cout << "!----------------------------------------------------------------------------!" << endl;
	cout << "!                               ALS Refinement                               !" << endl;
	cout << "!                              by Yue Pan et al.                             !" << endl;
	cout << "!----------------------------------------------------------------------------!" << endl;

	// This program accomplish the fine registration between ALS block and TLS point clouds. [Use TLS as the control cloud (alignment target)]
	// It also provides various methods to achieve the same goal. You can do comprehensive experiment based on this program.
	// It is part of the ALS Refinement Project for Highway Expansion and Reconstruction Engineering
	/*
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
	cout << "Please make sure all the ALS, MLS and BPLS point clouds are divided into blocks." << endl;
	cout << "If they are not divided, you can divide them now" << endl;
	*/
	//... 
	/*
	float time_of_block;
	vector<pcXYZI> cloud_blocks;

	cout << "Please input the time step of a block in second" << endl;
	cin >> time_of_block;

	DataIo <pcl::PointXYZI> io;
	io.ALS_block_by_time(cloudT,cloud_blocks,time_of_block);
	cout << "Divide ALS strip into blocks Done..." << endl;

	io.batchWriteBlockInColor("ALS_Block.las", cloud_blocks, 0);
	cout << "Write File Done..." << endl;
	*/

	/*
	string ALS_folder, TLS_folder, MLS_folder, BPLS_folder;
	cout << "Input ALS Point Cloud (Blocks of all the strips) Folder:" << endl;
	cin >> ALS_folder;
	cout << "Input TLS Point Cloud Folder:" << endl;
	cin >> TLS_folder;
	cout << "Input MLS Point Cloud (Blocks) Folder:" << endl;
	cin >> MLS_folder;
	cout << "Input BPLS Point Cloud (Blocks) Folder:" << endl;
	cin >> BPLS_folder;

	vector<vector<string>> ALS_strip_files;
	vector<string> TLS_files, MLS_files, BPLS_files;
	
	DataIo <pcl::PointXYZI> io;
	//io.batchReadFileNamesInFoldersAndSubFolders(ALS_folder, ".las", ALS_files);
	io.batchReadFileNamesInSubFolders(ALS_folder, ".las", ALS_strip_files);
	io.batchReadFileNamesInFolders(TLS_folder, ".las", TLS_files);
	io.batchReadFileNamesInFolders(MLS_folder, ".las", MLS_files);
	io.batchReadFileNamesInFolders(BPLS_folder, ".las", BPLS_files);

	vector<vector<CloudBlock>> ALS_strip_blocks(ALS_strip_files.size());
	vector<CloudBlock> TLS_blocks(TLS_files.size());
	vector<CloudBlock> MLS_blocks(MLS_files.size());
	vector<CloudBlock> BPLS_blocks(BPLS_files.size());

	vector<CloudBlock> All_blocks;
	int ALS_count=0;

	for (int i = 0; i < ALS_strip_files.size(); i++)
	{
		ALS_strip_blocks[i].resize(ALS_strip_files[i].size());
		for (int j = 0; j < ALS_strip_files[i].size(); j++)
		{
			io.readLasBlock(ALS_strip_files[i][j], 1, i, j, ALS_strip_blocks[i][j]);
			All_blocks.push_back(ALS_strip_blocks[i][j]);
			ALS_count++;
		}
	}
	cout << "ALS blocks import done ..." << endl;
	
	for (int i = 0; i < TLS_files.size(); i++)
	{
		io.readLasBlock(TLS_files[i], 2, 0, i, TLS_blocks[i]);
		All_blocks.push_back(TLS_blocks[i]);
	}
	cout << "TLS boxes import done ..." << endl;

	for (int i = 0; i < MLS_files.size(); i++)
	{
		io.readLasBlock(MLS_files[i], 3, 0, i, MLS_blocks[i]);
		All_blocks.push_back(MLS_blocks[i]);
	}
	cout << "MLS boxes import done ..." << endl;
	
	for (int i = 0; i < BPLS_files.size(); i++)
	{
		io.readLasBlock(BPLS_files[i], 4, 0, i, BPLS_blocks[i]);
		All_blocks.push_back(BPLS_blocks[i]);
	}
	cout << "BPLS boxes import done ..." << endl;

	cout << "!----------------------------------------------------------------------------!" << endl;
	cout << "The number of all the nodes : " << All_blocks.size() << endl;
	cout << "ALS: " << ALS_count << " blocks in " << ALS_strip_files.size() << " strips" << endl;
	cout << "TLS: " << TLS_files.size() << " stations" << endl;
	cout << "MLS: " << MLS_files.size() << " blocks" << endl;
	cout << "BPLS: " << BPLS_files.size() << " blocks" << endl;
	cout << "!----------------------------------------------------------------------------!" << endl;

	Constraint_Finder cf;
	vector<Constraint> All_cons, ALS_inner_strip_cons_all, MLS_adjacent_cons, BPLS_adjacent_cons, registration_cons;
	int adjacent_cons_num;
	cf.find_strip_adjacent_constraint(ALS_strip_blocks, ALS_inner_strip_cons_all);
	cf.find_adjacent_constraint_in_strip(MLS_blocks, MLS_adjacent_cons);
	cf.find_adjacent_constraint_in_strip(BPLS_blocks, BPLS_adjacent_cons);
	cf.find_overlap_registration_constraint(All_blocks, registration_cons, 8, 0.15);

	All_cons.insert(All_cons.end(), ALS_inner_strip_cons_all.begin(), ALS_inner_strip_cons_all.end());
	All_cons.insert(All_cons.end(), MLS_adjacent_cons.begin(), MLS_adjacent_cons.end());
	All_cons.insert(All_cons.end(), BPLS_adjacent_cons.begin(), BPLS_adjacent_cons.end());
	adjacent_cons_num = All_cons.size();
	All_cons.insert(All_cons.end(), registration_cons.begin(), registration_cons.end());
	
	cout << "The number of constraints : " << All_cons.size() << endl;
	cout << "The number of adjacent constraints : " << adjacent_cons_num << endl;
	cout << "The number of registration constraints : " << registration_cons.size()<<endl;
	cout << "!----------------------------------------------------------------------------!" << endl;
	
	cout << "Ready to display ..." << endl;
	io.display2Dboxes(All_blocks);
	io.display2Dcons(All_cons);
	*/


	//------------------------------------Read the Point Cloud for registration---------------------------------//
	
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
	
	cout << "Read File Done..." << endl;
	cout << "Raw point number: [ S:  " << cloudS->size() << "  , T:  " << cloudT->size() << " ]" << endl;
	
	
	//------------------------------------Filter the Point Cloud for registration--------------------------------//
	
	pcXYZIPtr subcloudT(new pcXYZI()), subcloudS(new pcXYZI());

	float downsample_resolution_ALS;  //Voxel Down-sampling's size (unit:m)
	float downsample_resolution_TLS;  //Voxel Down-sampling's size (unit:m)
	
	cout << "Input ALS's downsample resolution." << endl;
	cin >> downsample_resolution_ALS;
	cout << "Input TLS's downsample resolution." << endl;
	cin >> downsample_resolution_TLS;
	
	VoxelFilter<pcl::PointXYZI> vf_ALS(downsample_resolution_ALS);
	VoxelFilter<pcl::PointXYZI> vf_TLS(downsample_resolution_TLS);
	
	subcloudT = vf_ALS.filter(cloudT);
	subcloudS = vf_TLS.filter(cloudS);
	
	cout << "Down-sampled point cloud number: [ S:  " << subcloudS->size() << " , T:  " << subcloudT->size() << " ]" << endl;
	// Apply other filters here ...

	//---------------------------------Fine Registration of ALS and TLS point cloud------------------------------//
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
	cout << "Max iteration number is: " << endl;
	cin >> max_iter_num;

	cout << "Overlap search radius is: " << endl;
	cin >>overlap_search_radius;

	//

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
	
   
  /*
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
  
   int whichmethod;
   cin >> whichmethod;

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
   cout << "!----------------------------------------------------------------------------!" << endl;
   //reg.SVD_6DOF(coordinatesA, coordinatesB, TransMatrixA2B, 3);
   
   */
   bool endindex;
   cin >> endindex;

   return 1;
}