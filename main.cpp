#include "dataio.cpp"
#include "voxelFilter.h"
#include "reg_ALSTLS.cpp"


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

	//------------------------------------Read the Point Cloud for registration---------------------------------//
	
	// Input the filename (Or just drag in) by yourself 
	string filenameS, filenameT;
	cout << "Input ALS Point Cloud File:" << endl;  //ALS
	cin >> filenameT;
	cout << "Input TLS Point Cloud File:" << endl;  //TLS
	cin >> filenameS;

	int pc_format;
	cout << "Input Point Cloud Format: 1.pcd , 2.las , 3.ply;" << endl;
	cin >> pc_format;

	pcXYZIPtr cloudT(new pcXYZI()), cloudS(new pcXYZI());
	
	DataIo <pcl::PointXYZI> io;
	io.readCloudFile(pc_format, filenameT, cloudT);
	io.readCloudFile(pc_format, filenameS, cloudS);
	
	//io.readCloudFile(1, "ALS1.pcd", cloudT);
	//io.readCloudFile(1, "TLS1.pcd", cloudS);
	
	cout << "Read File Done..." << endl;
	cout << "Raw point number: [ S:  " << cloudS->size() << "  , T:  " << cloudT->size() << " ]" << endl;

	//------------------------------------Filter the Point Cloud for registration--------------------------------//

	pcXYZIPtr subcloudT(new pcXYZI()), subcloudS(new pcXYZI());
	float downsmaple_resolution=0.4;  //Voxel Down-sampling's size (unit:m)
	VoxelFilter<pcl::PointXYZI> vf(downsmaple_resolution);
	subcloudT = vf.filter(cloudT);
	subcloudS = vf.filter(cloudS);
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
	
	ICPs <pcl::PointXYZI> reg; 
	ICPs <pcl::PointXYZINormal> regn;

	pcXYZIPtr regS(new pcXYZI()), regALS(new pcXYZI());  // regS: Transformed Sub-sampled Source Cloud   regALS: Transformed Raw Source Cloud
	Eigen::Matrix4f Trans, TransALS;                     // Trans: TLS->TLS' for ALS (Target)   TransALS: ALS->ALS'  for TLS (Target)

	// other parameters
	int max_iter_num = 50;               // Maximum iteration number before convergence
	float overlap_search_radius = 4.0;   // The search radius in meter for overlapping ratio estimation
	int covariance_knn = 15;             // The number of neighbor points for covariance and normal calculation

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
		reg.gicp_reg(subcloudS, cloudT, regS, Trans, covariance_knn, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius);
		break;
	default:
		reg.icp_reg(subcloudS, cloudT, regS, Trans, max_iter_num, use_reciprocal_correspondence, use_trimmed_rejector, overlap_search_radius);
		break;
	}
	
	reg.invTransform(Trans,TransALS);                      //To get the transformation from ALS to ALS' with TLS as the target
	reg.transformcloud(cloudT, regALS, TransALS);          //To get the transformed ALS 
	
	
	io.writeCloudFile(pc_format, "ALS_after_registration", regALS);    
	cout << "Write File Done..." << endl;
	
	//io.displaymulti(cloudS, cloudT, regALS, "TLS[R] ALS_before[B] ALS_transformed[G]"); // Display the result. Click "X" to continue.

	bool endindex;
	cin >> endindex;

	return 1;
}