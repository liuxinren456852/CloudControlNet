#include "find_constraint.h"
#include "error_compute.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Geometry>

#define max_(a,b) (((a) > (b)) ? (a) : (b))
#define min_(a,b) (((a) < (b)) ? (a) : (b))


void Constraint_Finder::find_adjacent_constraint_in_strip(vector<CloudBlock> &blocks_strip, vector<Constraint> &innerstrip_cons)
{
	for (int i = 0; i < blocks_strip.size() - 1; i++)
	{
		Constraint con;
		con.block1 = blocks_strip[i];
		con.block2 = blocks_strip[i + 1];
		con.con_type = 1;   //Adjacent
		innerstrip_cons.push_back(con);
	}
}

void Constraint_Finder::find_strip_adjacent_constraint(vector<vector<CloudBlock> > &blocks_all, vector<Constraint> &innerstrip_cons_all)
{
	for (int i = 0; i < blocks_all.size(); i++)
	{
		vector<Constraint> innerstrip_cons;
		find_adjacent_constraint_in_strip(blocks_all[i], innerstrip_cons);
		for (int j = 0; j < innerstrip_cons.size(); j++)
		{
			innerstrip_cons_all.push_back(innerstrip_cons[j]);
		}
		innerstrip_cons.clear();
	}
}


void Constraint_Finder::find_overlap_registration_constraint(vector<CloudBlock> &blocks, vector<Constraint> &registration_cons_all, int k_search_neighbor, double min_iou)
{
	pcl::PointCloud<pcl::PointXY>::Ptr cp_cloud(new pcl::PointCloud<pcl::PointXY>());
	
	for (int i = 0; i < blocks.size(); i++)
	{
		pcl::PointXY cp;
		cp.x = blocks[i].cp.x;
		cp.y = blocks[i].cp.y;
		cp_cloud->push_back(cp);
	}

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(cp_cloud);	

	for (int i = 0; i < blocks.size(); i++)
	{
		std::vector<int> pointIdx;
		std::vector<float> pointSquaredDistance;

		pcl::PointXY cp_search;
		cp_search.x = blocks[i].cp.x;
		cp_search.y = blocks[i].cp.y;

		kdtree.nearestKSearch(cp_search, k_search_neighbor, pointIdx, pointSquaredDistance);
        
		for (int j = 0; j < pointIdx.size(); j++)
		{
			
			double iou = calculate_iou(blocks[i].bound, blocks[pointIdx[j]].bound);
			bool is_adjacent = judge_adjacent(blocks[i],blocks[pointIdx[j]]);

			if ((pointIdx[j]>i) && (iou > min_iou) && (!is_adjacent))
			{
				Constraint registration_con;
				registration_con.block1 = blocks[i];
				registration_con.block2 = blocks[pointIdx[j]];
				registration_con.con_type = 2; //Registration

				registration_cons_all.push_back(registration_con);
			}
		}
	}
}

bool Constraint_Finder::judge_adjacent(CloudBlock & block1, CloudBlock & block2)
{
	bool is_adjacent = false;
	if (block1.data_type == block2.data_type && block1.strip_num == block2.strip_num)
	{
		if ((block1.num_in_strip == block2.num_in_strip + 1) || (block1.num_in_strip == block2.num_in_strip - 1))
		{
			is_adjacent=true;
		}
	}
	return is_adjacent;
}

bool Constraint_Finder::judge_adjacent_hdmap_index(SubMap & submap1, SubMap & submap2, int index_min_interval)
{
	bool is_adjacent = false;
	
	if (submap1.type == submap2.type && submap1.transaction_id == submap2.transaction_id)
	{
		if (abs(submap1.id_in_transaction - submap2.id_in_transaction) <= index_min_interval)
		{
			is_adjacent=true;
		}
	}
	return is_adjacent;
}


bool Constraint_Finder::judge_adjacent_hdmap_index(Frame & frame1, Frame & frame2, int index_min_interval)
{
	bool is_adjacent = false;
	
	if (frame1.type == frame2.type && frame1.transaction_id == frame2.transaction_id)
	{
		if (abs(frame1.id_in_transaction - frame2.id_in_transaction) <= index_min_interval)
		{
			is_adjacent=true;
		}
	}
	return is_adjacent;
}


bool Constraint_Finder::judge_adjacent_hdmap_time(Frame & frame1, Frame & frame2, double min_deltatime_in_us)
{
	bool is_adjacent = false;
	

	if (frame1.type == frame2.type && frame1.transaction_id ==  frame2.transaction_id)
	{
		double deltatime_in_us=abs( (frame1.time_stamp.tv_sec * 1000000 + frame1.time_stamp.tv_usec) -
			(frame2.time_stamp.tv_sec * 1000000 + frame2.time_stamp.tv_usec) );
		
		if (deltatime_in_us <= min_deltatime_in_us)
		{
			is_adjacent=true;
		}
	}
	return is_adjacent;
}


double Constraint_Finder::calculate_iou(Bounds & bound1, Bounds & bound2)
{
	double area1 = (bound1.max_x - bound1.min_x) * (bound1.max_y - bound1.min_y); 
	double area2 = (bound2.max_x - bound2.min_x) * (bound2.max_y - bound2.min_y);

	double x1 = max_(bound1.min_x, bound2.min_x);
	double y1 = max_(bound1.min_y, bound2.min_y);

	double x2 = min_(bound1.max_x, bound2.max_x);
	double y2 = min_(bound1.max_y, bound2.max_y);
	
	double w = max_(0, x2 - x1);
	double h = max_(0, y2 - y1);

	double area1i2 = w*h;

	double iou = area1i2 / (area1 + area2 - area1i2);

	return iou;
}

void Constraint_Finder::batch_find_multisource_constraints(std::vector<std::vector<CloudBlock> > &ALS_strip_blocks, std::vector<CloudBlock> &TLS_blocks, std::vector<CloudBlock> &MLS_blocks, std::vector<CloudBlock> &BPLS_blocks, std::vector<CloudBlock> &All_blocks,
	std::vector<Constraint> &ALS_inner_strip_cons_all, std::vector<Constraint> &MLS_adjacent_cons, std::vector<Constraint> &BPLS_adjacent_cons, std::vector<Constraint> &registration_cons, std::vector<Constraint> &All_cons,
	int overlap_Registration_KNN, float overlap_Registration_OverlapRatio)
{
	int adjacent_cons_num;
	find_strip_adjacent_constraint(ALS_strip_blocks, ALS_inner_strip_cons_all);
	if(MLS_blocks.size()>1) find_adjacent_constraint_in_strip(MLS_blocks, MLS_adjacent_cons);
	if(BPLS_blocks.size()>1) find_adjacent_constraint_in_strip(BPLS_blocks, BPLS_adjacent_cons);
	find_overlap_registration_constraint(All_blocks, registration_cons, overlap_Registration_KNN, overlap_Registration_OverlapRatio);

	cout << "Constraints Found" << endl;
	if (ALS_inner_strip_cons_all.size()>0) All_cons.insert(All_cons.end(), ALS_inner_strip_cons_all.begin(), ALS_inner_strip_cons_all.end());
	if (MLS_adjacent_cons.size()>0) All_cons.insert(All_cons.end(), MLS_adjacent_cons.begin(), MLS_adjacent_cons.end());
	if (BPLS_adjacent_cons.size()>0) All_cons.insert(All_cons.end(), BPLS_adjacent_cons.begin(), BPLS_adjacent_cons.end());
	adjacent_cons_num = All_cons.size();
	if (registration_cons.size()>0) All_cons.insert(All_cons.end(), registration_cons.begin(), registration_cons.end());

	cout << "The number of constraints : " << All_cons.size() << endl;
	cout << "The number of adjacent constraints : " << adjacent_cons_num << endl;
	cout << "The number of registration constraints : " << registration_cons.size() << endl;
	cout << "!----------------------------------------------------------------------------!" << endl;
}



void Constraint_Finder::find_hdmap_adjacent_constraints_submap(Transaction & transaction, vector<Edge_between_2Submaps> & HDmap_adjacent_edges)
{
    for (int i = 0; i < transaction.submap_number-1; i++)
	{
		Edge_between_2Submaps edge_adjacent;
		edge_adjacent.submap1 = transaction.submaps[i];
		edge_adjacent.submap2 = transaction.submaps[i+1];
		edge_adjacent.type = ADJACENT;   
		HDmap_adjacent_edges.push_back(edge_adjacent);
	}
}

void Constraint_Finder::find_hdmap_revisit_constraints_submap(Transaction & transaction, vector<Edge_between_2Submaps> & HDmap_revisit_edges, int index_min_interval, float max_revisit_pos_dis, float min_iou)
{
    int max_revisit_edge_for_one_subamp = 8; //Set it //Random or from small to big?

	pcl::PointCloud<pcl::PointXY>::Ptr cp_cloud(new pcl::PointCloud<pcl::PointXY>());
	
	for (int i = 0; i < transaction.submap_number; i++)
	{
		pcl::PointXY cp;
		cp.x = transaction.submaps[i].centerpoint.x;
		cp.y = transaction.submaps[i].centerpoint.y;
		//printf("[%d]: %lf,%lf\n", HDmap_frames[i].unique_id, cp.x,cp.y);
		cp_cloud->push_back(cp);
	}

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(cp_cloud);	

	for (int i = 0; i < transaction.submap_number; i++)
	{
		std::vector<int> pointIdx;
		std::vector<float> pointSquaredDistance;

		pcl::PointXY cp_search;
		cp_search.x = transaction.submaps[i].centerpoint.x;
		cp_search.y = transaction.submaps[i].centerpoint.y;

		if( kdtree.radiusSearch(cp_search, max_revisit_pos_dis, pointIdx, pointSquaredDistance) > 0 )
        {
		   int count=0;
		   for (int j = 0; j < min_(pointIdx.size(), max_revisit_edge_for_one_subamp); j++)
		   {
			    bool isadjacent = judge_adjacent_hdmap_index(transaction.submaps[i],transaction.submaps[pointIdx[j]],index_min_interval);
				double iou = calculate_iou(transaction.submaps[i].boundingbox, transaction.submaps[pointIdx[j]].boundingbox);

			    if ( !isadjacent && iou > min_iou && pointIdx[j]>i )
			    {
				   Edge_between_2Submaps revisit_edge;
				   revisit_edge.submap1 = transaction.submaps[i];
				   revisit_edge.submap2 = transaction.submaps[pointIdx[j]];
				   revisit_edge.type = REVISIT ; 
 
			       HDmap_revisit_edges.push_back(revisit_edge);
				   count++;	
			 }
		   }
		  //printf("revisit number : %d \n",count);
		}
	}
}

void Constraint_Finder::batch_find_hdmap_constraints_submap(Transaction & transaction, int index_min_interval, float max_revisit_pos_distance,  float min_iou)
{
    vector<Edge_between_2Submaps> HDmap_revisit_edges;
	vector<Edge_between_2Submaps> HDmap_adjacent_edges;
	
	find_hdmap_adjacent_constraints_submap(transaction,HDmap_adjacent_edges);
	printf("The number of adjacent constraints : %d\n", HDmap_adjacent_edges.size());

	find_hdmap_revisit_constraints_submap(transaction,HDmap_revisit_edges, index_min_interval, max_revisit_pos_distance, min_iou);
	printf("The number of revisit constraints : %d\n", HDmap_revisit_edges.size());
	
	transaction.submap_edges.insert(transaction.submap_edges.end(),HDmap_adjacent_edges.begin(),HDmap_adjacent_edges.end());
	transaction.submap_edges.insert(transaction.submap_edges.end(),HDmap_revisit_edges.begin(),HDmap_revisit_edges.end());
	
	//output
	printf("The number of constraints : %d\n", transaction.submap_edges.size());
}

void Constraint_Finder::find_hdmap_adjacent_constraints_frame(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_adjacent_edges)
{
   //Fix it for multi-transaction cases
   for (int i = 0; i < HDmap_frames.size() - 1; i++)
	{
		Edge_between_2Frames edge_adjacent;
		edge_adjacent.frame1 = HDmap_frames[i];
		edge_adjacent.frame2 = HDmap_frames[i+1];
		edge_adjacent.type = ADJACENT;   
		HDmap_adjacent_edges.push_back(edge_adjacent);
	}
}


void Constraint_Finder::find_hdmap_revisit_constraints_frame(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_revisit_edges,  int index_min_interval, float max_revisit_pos_distance)
{
	pcl::PointCloud<pcl::PointXY>::Ptr cp_cloud(new pcl::PointCloud<pcl::PointXY>());
	
	for (int i = 0; i < HDmap_frames.size(); i++)
	{
		pcl::PointXY cp;
		cp.x = HDmap_frames[i].oxts_position(0);
		cp.y = HDmap_frames[i].oxts_position(1);
		//printf("[%d]: %lf,%lf\n", HDmap_frames[i].unique_id, cp.x,cp.y);
		cp_cloud->push_back(cp);
	}

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(cp_cloud);	

	for (int i = 0; i < HDmap_frames.size(); i++)
	{
		std::vector<int> pointIdx;
		std::vector<float> pointSquaredDistance;

		pcl::PointXY cp_search;
		cp_search.x = HDmap_frames[i].oxts_position(0);
		cp_search.y = HDmap_frames[i].oxts_position(1);

        //printf("[%d]: %lf,%lf\n", HDmap_frames[i].unique_id, cp_search.x,cp_search.y);
		
		int radius_search_num = kdtree.radiusSearch(cp_search, max_revisit_pos_distance, pointIdx, pointSquaredDistance);
		//printf("radius search number  : %d \n",radius_search_num);

		if(radius_search_num >0)
        {
		   int count=0;
		   for (int j = 0; j < pointIdx.size(); j++)
		   {
			    //printf("find [%d]: %lf,%lf\n", HDmap_frames[pointIdx[j]].unique_id, cp_cloud->points[pointIdx[j]].x,cp_cloud->points[pointIdx[j]].y);

			    bool isadjacent = judge_adjacent_hdmap_index(HDmap_frames[i],HDmap_frames[pointIdx[j]],index_min_interval);
			    if ( !isadjacent && pointIdx[j]>i )
			    {
				   Edge_between_2Frames revisit_edge;
				   revisit_edge.frame1 = HDmap_frames[i];
				   revisit_edge.frame2 = HDmap_frames[pointIdx[j]];
				   revisit_edge.type = REVISIT ; 
 
			       HDmap_revisit_edges.push_back(revisit_edge);
				   count++;	
			 }
		   }
		  //printf("revisit number : %d \n",count);
		}
	}
}


void Constraint_Finder::batch_find_hdmap_constraints_frame(vector<Frame> &HDmap_frames, vector<Edge_between_2Frames> &HDmap_edges, int index_min_interval, float max_revisit_pos_distance)
{
    vector<Edge_between_2Frames> HDmap_revisit_edges;
	vector<Edge_between_2Frames> HDmap_adjacent_edges;
	
	find_hdmap_adjacent_constraints_frame(HDmap_frames,HDmap_adjacent_edges);
	find_hdmap_revisit_constraints_frame(HDmap_frames,HDmap_revisit_edges, index_min_interval, max_revisit_pos_distance);
	
	HDmap_edges.insert(HDmap_edges.end(),HDmap_adjacent_edges.begin(),HDmap_adjacent_edges.end());
	HDmap_edges.insert(HDmap_edges.end(),HDmap_revisit_edges.begin(),HDmap_revisit_edges.end());
	
	//output
	printf("The number of constraints : %d\n", HDmap_edges.size());
	printf("The number of adjacent constraints : %d\n", HDmap_adjacent_edges.size());
	printf("The number of revisit constraints : %d\n", HDmap_revisit_edges.size());

}

void Constraint_Finder::divide_submap(Transaction & transaction, int max_frame_num, float max_tran_dis, float max_heading_angle)
{
	int temp_frame_num_in_frame=0;
	float temp_tran_dis=0;
	float temp_heading_angle=0;
    transaction.submap_number=0;
    
    float imu_time_step=0.01;
    
	//transaction.submaps.resize((int)(transaction.frame_number/(max_frame_num+1))+1);
    
	SubMap temp_submap;
	temp_submap.frame_number=0;

	for (int i=0; i< transaction.frame_number-1; i++)
	{	
		temp_submap.frames.push_back(transaction.frames[i]);
		temp_submap.frame_number++;
		
		temp_tran_dis += getDistance(transaction.frames[i].oxts_position, transaction.frames[i+1].oxts_position);	
		temp_heading_angle += getHeading(transaction.frames[i].imu_datas,imu_time_step);
		temp_frame_num_in_frame++;
        
		//Three Submap Terminal Condition
		if(temp_frame_num_in_frame>=max_frame_num || temp_tran_dis>=max_tran_dis || temp_heading_angle>=max_heading_angle)
		{	
			temp_submap.oxts_pose=temp_submap.frames[0].oxts_pose;
			temp_submap.oxts_position=temp_submap.frames[0].oxts_position;
			temp_submap.id_in_transaction=transaction.submap_number;
            temp_submap.type=transaction.type;

			//transaction.submaps[transaction.submap_number]=temp_submap;
			transaction.submaps.push_back(temp_submap);

			printf("Submap [%d] frame_num: %d  distance: %lf  heading: %lf \n", transaction.submap_number, temp_frame_num_in_frame, temp_tran_dis, temp_heading_angle);
			transaction.submap_number++;
			
            temp_tran_dis=0;
			temp_heading_angle=0;
			temp_frame_num_in_frame=0;

			temp_submap.frames.resize(0);
		    temp_submap.frame_number=0;
		}
		//cout<<i<<endl;
	} 
	temp_submap.oxts_pose=temp_submap.frames[0].oxts_pose;
	temp_submap.oxts_position=temp_submap.frames[0].oxts_position;
	temp_submap.id_in_transaction=transaction.submap_number;
    temp_submap.type=transaction.type;
    transaction.submaps.push_back(temp_submap);
	printf("Submap [%d] frame_num: %d  distance: %lf  heading: %lf \n", transaction.submap_number, temp_frame_num_in_frame, temp_tran_dis, temp_heading_angle);
	transaction.submap_number++;

	printf("The total submap number is %d \n",transaction.submap_number);
}

float Constraint_Finder::getDistance(Eigen::Vector3f &position1, Eigen::Vector3f &position2)
{
	float result= (position1(0)-position2(0))*(position1(0)-position2(0))+(position1(1)-position2(1))*(position1(1)-position2(1))
	+(position1(2)-position2(2))*(position1(2)-position2(2));
	return (sqrt(result));
}	

float Constraint_Finder::getHeading(vector<IMU_data> &imu_datas, float delta_time_second)
{
    float result=0;
	
	for (int i=0; i< imu_datas.size()-1; i++)
	{
		result += (180.0 / 3.14159) * 0.5 * (imu_datas[i].wz+ imu_datas[i+1].wz) * delta_time_second;
	}
	return abs(result);
}

void Constraint_Finder::front_end(Transaction &transaction)
{
	clock_t t0, t1;
	t0=clock();
	
	DataIo <Point_T> io;
	string display_name;
    
	ErrorCompute ec;
    std::vector<Eigen::Matrix4f> oxts_pose;
	std::vector<Eigen::Matrix4f> odom_pose;

	transaction.transaction_mae=0;
    for (int i=0; i< transaction.submap_number; i++)
	{
        //Lidar Odom Main Entrance
		submapOdom(transaction.submaps[i]);
		
		//For Error Evaluation
		for (int j=0; j< transaction.submaps[i].frame_number;j++)
		{   
            transaction.submaps[i].oxts_noise_pose=transaction.submaps[i].frames[0].oxts_noise_pose; //Add noise / transfer to submap 
			oxts_pose.push_back(transaction.submaps[i].frames[j].oxts_pose);
			odom_pose.push_back(transaction.submaps[i].frames[j].odom_pose);
		}
		
		printf("--------SubMap %d--------\n",i);
        
		ostringstream oss; 
		oss<<i;
        display_name="Submap: "+ oss.str()+ " LO(Red) OXTS-GT(Blue) OXTS-INPUT(YELLOW)";
        //if (i%10==0) io.display_submap(transaction.submaps[i], display_name,INTENSITY, 1); 

		transaction.transaction_mae+=transaction.submaps[i].submap_mae;
	}
    t1=clock();
    //float time_per_frame= (float(t1 - t0) / CLOCKS_PER_SEC)/transaction.submaps[0].frame_number;
	ec.compute(oxts_pose,odom_pose);
    ec.print_error();
    
	transaction.transaction_mae/=transaction.submap_number;
	printf("The transaction's Mean Abosulte Error is %lf. \n",transaction.transaction_mae);
	//printf("Average time for one frame's processing %lf\n", time_per_frame);
    
	io.display_submap(transaction.submaps[0], display_name, INTENSITY, 2); 
	io.display_trajectory(transaction,"LO(Red) OXTS_GT(Blue) OXTS_NOISE(YELLOW) StartingPoint(Green)");
}

bool Constraint_Finder::submapOdom(SubMap & submap)
{
    DataIo <Point_T> io;
	CFilter <Point_T> filter;
	CRegistration <Point_T> reg;
	PrincipleComponentAnalysis<Point_T> pca_estimator; 

	//Parameters
    //Distance Filter
    float xy_max = 40.0, z_min = -4.0 , z_max = 30.0;
    
	//Voxel Downsampling
	float voxel_filter_radius_frame = 0.35;
    float voxel_filter_radius_submap = 0.25;
	
	//Normal Downsampling
	float normal_search_radius = 0.35;
	int normal_search_K = 10;
	float normal_downsmaple_rate = 0.2;
	
	//Ground Filter
	int gf_min_grid_num =8;
	float gf_grid_resolution =0.8;
	float gf_max_grid_height_diff = 0.25;
	float gf_neighbor_height_diff = 1.2;
	float gf_max_ground_height = 2.8;
	int gf_downsample_rate_nonground = 2;
	int gf_downsample_rate_ground_first = 15;   //For reference ground points (more point number)
    int gf_downsample_rate_ground_second = 60;  //For search ground points (less point number)
    
	//Feature Points Detection
	float neighbor_search_radius = 0.4;
	int feature_min_neighbor_point_num = 8;
	float edge_point_thre_low = 0.6;           //For reference edge points (more point number)
	float planar_point_thre_low = 0.6;          //For reference planar points (more point number)
	float sphere_point_thre_low = 0.6;         //For reference sphere points (more point number)
    float edge_point_thre_high = 0.75;          //For search edge points (less point number)
	float planar_point_thre_high = 0.75;        //For search planar points (less point number)
	float sphere_point_thre_high = 0.75;         //For search sphere points (less point number)
    
	//Add OXTS Noise
	float noise_t = 0.2, noise_r = 0.2;         //Max random noise of translation(m) and rotation (degree)  
    

	//Begin 
	cout<<"Submap's frame number is "<<submap.frame_number<<endl;
    
	// Preprocessing
	for (int i=0;i< submap.frame_number;i++)
	{
		//Add noise for oxts data.
		if(i==0) add_noise(submap.frames[i], 0, 0);
		else add_noise(submap.frames[i], 2*noise_t, 2*noise_r);

		//Ptr initialization (Don't forget it !!!)
		submap.frames[i].pointcloud_lidar= boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_lidar_down= boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_odom= boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_odom_down= boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_oxts= boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_oxts_down= boost::make_shared <pcl::PointCloud<Point_T> > ();
        submap.frames[i].pointcloud_submap= boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_submap_down= boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_ground=boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_ground_down=boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_edge=boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_edge_down=boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_planar=boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_planar_down=boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_sphere=boost::make_shared <pcl::PointCloud<Point_T> > ();
		submap.frames[i].pointcloud_sphere_down=boost::make_shared <pcl::PointCloud<Point_T> > ();
        
		pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_raw (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<Point_T>::Ptr pointcloud_lidar_filtered (new pcl::PointCloud<Point_T>);
		pcl::PointCloud<Point_T>::Ptr pointcloud_lidar_unground (new pcl::PointCloud<Point_T>);
	   

		//Import cloud
		io.readPcdFile(submap.frames[i].pcd_file_name, submap.frames[i].pointcloud_lidar);
		//pcl::copycloud(,submap.frames[i].pointcloud_lidar);
		
		//Active Objects Filtering (@ Jingwei Li)

		//Dis Filter 
		filter.DisFilter(submap.frames[i].pointcloud_lidar, pointcloud_lidar_filtered, xy_max, z_min, z_max);  
        
		//Ground Filter
		filter.FastGroundFilter(pointcloud_lidar_filtered, submap.frames[i].pointcloud_ground, submap.frames[i].pointcloud_ground_down, pointcloud_lidar_unground,
		                        gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff, gf_neighbor_height_diff, gf_max_ground_height, 
		                        gf_downsample_rate_ground_first, gf_downsample_rate_ground_second, gf_downsample_rate_nonground);
	    
		//Detected Feature Points
        filter.RoughClassify(pointcloud_lidar_unground, submap.frames[i].pointcloud_edge, submap.frames[i].pointcloud_planar, submap.frames[i].pointcloud_sphere,
		                     submap.frames[i].pointcloud_edge_down, submap.frames[i].pointcloud_planar_down, submap.frames[i].pointcloud_sphere_down,
		                     neighbor_search_radius, feature_min_neighbor_point_num, 
							 edge_point_thre_low, planar_point_thre_low, sphere_point_thre_low,
							 edge_point_thre_high, planar_point_thre_high, sphere_point_thre_high);
        
		//Add to pointcloud_lidar_down
        submap.frames[i].pointcloud_lidar_down->points.insert(submap.frames[i].pointcloud_lidar_down->points.end(), submap.frames[i].pointcloud_ground_down->points.begin(), submap.frames[i].pointcloud_ground_down->points.end());
		submap.frames[i].pointcloud_lidar_down->points.insert(submap.frames[i].pointcloud_lidar_down->points.end(), submap.frames[i].pointcloud_edge_down->points.begin()  , submap.frames[i].pointcloud_edge_down->points.end()  );
		submap.frames[i].pointcloud_lidar_down->points.insert(submap.frames[i].pointcloud_lidar_down->points.end(), submap.frames[i].pointcloud_planar_down->points.begin(), submap.frames[i].pointcloud_planar_down->points.end());
        submap.frames[i].pointcloud_lidar_down->points.insert(submap.frames[i].pointcloud_lidar_down->points.end(), submap.frames[i].pointcloud_sphere_down->points.begin(), submap.frames[i].pointcloud_sphere_down->points.end());
        
		// Display the feature points extraction result
		if (i==100)
		{
        std::vector< pcl::PointCloud<Point_T> > feature_points_reference; //more points
        std::vector< pcl::PointCloud<Point_T> > feature_points_source;    //less points
        
		feature_points_reference.push_back(*submap.frames[i].pointcloud_ground);
		feature_points_reference.push_back(*submap.frames[i].pointcloud_edge);
	    feature_points_reference.push_back(*submap.frames[i].pointcloud_planar);
		feature_points_reference.push_back(*submap.frames[i].pointcloud_sphere);
		feature_points_source.push_back(*submap.frames[i].pointcloud_ground_down);
		feature_points_source.push_back(*submap.frames[i].pointcloud_edge_down);
	    feature_points_source.push_back(*submap.frames[i].pointcloud_planar_down);
		feature_points_source.push_back(*submap.frames[i].pointcloud_sphere_down);
		
		io.display1cloud(pointcloud_lidar_filtered,"All points",1);
		io.displaynclouds(feature_points_reference, "Feature points (Reference)",1 );
        io.displaynclouds(feature_points_source , "Feature points (Source)",1 );
		}

		//Downsample 
		//filter.VoxelDownsample(pointcloud_lidar_filtered, submap.frames[i].pointcloud_lidar_down, voxel_filter_radius_frame);
		//filter.VoxelDownsample(pointcloud_lidar_filtered, pointcloud_lidar_down, voxel_filter_radius_frame);
		//filter.NormalDownsample(pointcloud_lidar_filtered, submap.frames[i].pointcloud_lidar_down, normal_search_K, 0.1);
		//io.display2clouds(submap.frames[i].pointcloud_lidar_down,pointcloud_lidar_filtered,"Voxel Downsampling",1 );
		
        //Free the memory
        if(i>1) pcl::PointCloud<Point_T>().swap(*submap.frames[i].pointcloud_lidar);   
		pcl::PointCloud<Point_T>().swap(*pointcloud_lidar_filtered);                                   
		pcl::PointCloud<Point_T>().swap(*pointcloud_lidar_unground);           
		
	}

	//Preparation
    submap.frames[0].odom_pose=submap.frames[0].oxts_pose;
	pcl::transformPointCloud(*submap.frames[0].pointcloud_lidar_down, *submap.frames[0].pointcloud_odom_down, submap.frames[0].odom_pose);
	//pcl::transformPointCloud(*submap.frames[0].pointcloud_lidar, *submap.frames[0].pointcloud_odom, submap.frames[0].odom_pose);
	//reg.transformcloud(submap.frames[0].pointcloud_lidar_down, submap.frames[0].pointcloud_odom_down, submap.frames[0].odom_pose);
	//reg.transformcloud(submap.frames[0].pointcloud_lidar, submap.frames[0].pointcloud_odom, submap.frames[0].odom_pose);
    
	filter.getBoundAndCenter(*submap.frames[0].pointcloud_odom_down, submap.frames[0].boundingbox, submap.frames[0].centerpoint);

	for (int i=0;i< submap.frame_number-1;i++)
	{
		printf("---------------%d frame in the submap--------------\n",i+1);
		pairwiseReg(submap.frames[i],submap.frames[i+1]);
		
		if (i==0)
		{
          //For test
		  pcl::transformPointCloud(*submap.frames[i].pointcloud_lidar, *submap.frames[i].pointcloud_odom, submap.frames[i].odom_pose);
          pcl::transformPointCloud(*submap.frames[i+1].pointcloud_lidar, *submap.frames[i+1].pointcloud_oxts, submap.frames[i+1].oxts_noise_pose);
		  pcl::transformPointCloud(*submap.frames[i+1].pointcloud_lidar, *submap.frames[i+1].pointcloud_odom, submap.frames[i+1].odom_pose);
	      io.display2clouds (submap.frames[i].pointcloud_odom, submap.frames[i+1].pointcloud_oxts, "OXTS Initial Guess", 1);
	      io.display2clouds (submap.frames[i].pointcloud_odom, submap.frames[i+1].pointcloud_odom, "After Registration", 1);
		}

		//Get Bounding Box and Center Point
		filter.getBoundAndCenter(*submap.frames[i+1].pointcloud_odom_down, submap.frames[i+1].boundingbox, submap.frames[i+1].centerpoint);
        
		//printf("Submap bounding box: x [%lf - %lf] y [%lf - %lf] z [%lf - %lf]\n", submap.frames[i+1].boundingbox.min_x, submap.frames[i+1].boundingbox.max_x, submap.frames[i+1].boundingbox.min_y, 
	//submap.frames[i+1].boundingbox.max_y, submap.frames[i+1].boundingbox.min_z, submap.frames[i+1].boundingbox.max_z);
		//odom_clouds.push_back(*submap.frames[i].pointcloud_odom_down);
		//oxts_clouds.push_back(*submap.frames[i].pointcloud_oxts_down);
		//lidar_clouds.push_back(*submap.frames[i].pointcloud_lidar_down);
	}

	merge_frame_bounds_and_centers(submap);

#if 1
   
    //Merge the cloud and downsample again (saved the submap's pointcloud_lidar_down)
	pcl::PointCloud<Point_T>::Ptr submap_pointcloud_merged(new pcl::PointCloud<Point_T>());
	submap.pointcloud_lidar_down= boost::make_shared <pcl::PointCloud<Point_T> > ();
	
	for (int i=0;i< submap.frame_number;i++) //Fix for speed
	{
        Eigen::Matrix4f frame_pose_ref_submap = submap.frames[0].odom_pose.inverse()*submap.frames[i].odom_pose;
		pcl::transformPointCloud(*submap.frames[i].pointcloud_lidar_down, *submap.frames[i].pointcloud_submap_down, frame_pose_ref_submap); 
        
		//reg.transformcloud(submap.frames[i].pointcloud_lidar_down, submap.frames[i].pointcloud_submap_down, frame_pose_ref_submap);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
		submap_pointcloud_merged->points.insert(submap_pointcloud_merged->points.end(),submap.frames[i].pointcloud_submap_down->points.begin(),submap.frames[i].pointcloud_submap_down->points.end());
		
		//Free the Memory
		pcl::PointCloud<Point_T>().swap(*submap.frames[i].pointcloud_submap_down);
		pcl::PointCloud<Point_T>().swap(*submap.frames[i].pointcloud_lidar_down);
		//pcl::PointCloud<Point_T>().swap(*submap.frames[i].pointcloud_odom_down);
	}

	printf("Submap's downsmaple points number: %d \n",  submap_pointcloud_merged->points.size());

	//Downsampling again
	filter.VoxelDownsample(submap_pointcloud_merged, submap.pointcloud_lidar_down, voxel_filter_radius_submap);
	
	printf("Submap's double-downsmaple points number: %d \n", submap.pointcloud_lidar_down->points.size());
	
	pcl::PointCloud<Point_T>().swap(*submap_pointcloud_merged); // Free the Memory

#endif

	//Evaluate the accuracy
	submap.submap_mae = getMAE(submap);
    printf("The submap's Mean Absolute Error is %lf.\n", submap.submap_mae);

	//For Temp Display
    //io.displaynclouds(odom_clouds, "Lidar Odometry",1);
	//io.displaynclouds(oxts_clouds, "GNSSINS",1);
	//io.displaynclouds(lidar_clouds, "Before Registration",1);

	return 1;
}

//Pairwise registration between frames
bool Constraint_Finder::pairwiseReg(Frame &frame1, Frame &frame2)
{   
    int max_iter_num=5;
	float thre_dis_ground=0.7f;
	float thre_dis_edge=0.45f;
	float thre_dis_planar=0.45f;
	float thre_dis_sphere=0.4f;

    pcl::PointCloud<Point_T>::Ptr pointcloud_ground_g (new pcl::PointCloud<Point_T>());
	pcl::PointCloud<Point_T>::Ptr pointcloud_edge_g   (new pcl::PointCloud<Point_T>());
	pcl::PointCloud<Point_T>::Ptr pointcloud_planar_g (new pcl::PointCloud<Point_T>());
	pcl::PointCloud<Point_T>::Ptr pointcloud_sphere_g (new pcl::PointCloud<Point_T>());

	Eigen::Matrix4f transformation2to1f;

	//Provide initial guess by OXTS pose
	//Eigen::Matrix4f transformation2to1g= frame1.oxts_pose.inverse() * frame2.oxts_pose;
	Eigen::Matrix4f transformation2to1g= frame1.oxts_noise_pose.inverse() * frame2.oxts_noise_pose;

	//cout<<"Frame1's oxts pose" <<endl<< frame1->oxts_pose<<endl;
	//cout<<"Frame2's oxts pose" <<endl<< frame2->oxts_pose<<endl;
    
	cout<<"Initial Guess of OXTS"<<endl<< transformation2to1g<<endl;
	
    //Provide initial guess by uniform motion model

	//Provide initial guess by imu preintegration

	//Apply intial guess
	pcl::transformPointCloudWithNormals(*frame2.pointcloud_ground_down, *pointcloud_ground_g ,transformation2to1g);
	pcl::transformPointCloudWithNormals(*frame2.pointcloud_edge_down  , *pointcloud_edge_g   ,transformation2to1g);
	pcl::transformPointCloudWithNormals(*frame2.pointcloud_planar_down, *pointcloud_planar_g ,transformation2to1g);
	pcl::transformPointCloudWithNormals(*frame2.pointcloud_sphere_down, *pointcloud_sphere_g ,transformation2to1g);
	
	CRegistration <Point_T> reg;
    
	Eigen::Matrix<float, 6, 6>  reg_information_matrix;
    
	//Feature point efficient registration 
	reg.feature_pts_registration(pointcloud_ground_g, pointcloud_edge_g, pointcloud_planar_g, pointcloud_sphere_g,
		                         frame1.pointcloud_ground, frame1.pointcloud_edge, frame1.pointcloud_planar, frame1.pointcloud_sphere, 
								 transformation2to1f, reg_information_matrix, 
								  max_iter_num, thre_dis_ground, thre_dis_edge,thre_dis_planar, thre_dis_sphere);
									  
	//Registration (point to plane ICP)
	//reg.icp_registration(FrameCloudDown2_g, frame1.pointcloud_lidar_down, FrameCloudDown2_t,transformation2to1f,
	//         Point2Plane, NN, LLS, 0, 0, 5, 0.3f, 8);
    
	//Get final registration result
	Eigen::Matrix4f transformation2to1 = transformation2to1f * transformation2to1g;
	
	cout<<"Final Registration result"<<endl<< transformation2to1<<endl;
    
	//Get frame's lidar odometry pose
	frame2.odom_pose = frame1.odom_pose * transformation2to1;
    //Transform cloud
	pcl::transformPointCloud(*frame2.pointcloud_lidar_down, *frame2.pointcloud_odom_down, frame2.odom_pose);

	
	//For temp display
    // DataIo <Point_T> io;
	// io.display2clouds (frame1.pointcloud_lidar_down, frame2.pointcloud_lidar_down, "Before Registration", 1);
	// io.display2clouds (frame1.pointcloud_lidar_down, FrameCloudDown2_g, "OXTS Initial Guess", 1);
	// io.display2clouds (frame1.pointcloud_lidar_down, FrameCloudDown2_t, "After Registration", 1);
    
	// Free the Memory
	pcl::PointCloud<Point_T>().swap(*pointcloud_ground_g);   
	pcl::PointCloud<Point_T>().swap(*pointcloud_edge_g);  
	pcl::PointCloud<Point_T>().swap(*pointcloud_planar_g);
	pcl::PointCloud<Point_T>().swap(*pointcloud_sphere_g); 
    
	return 1;	
}

//Pairwise registration between submaps
bool Constraint_Finder::pairwiseReg(Edge_between_2Submaps &edge)
{	
    pcl::PointCloud<Point_T>::Ptr SubmapCloudDown2_t (new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr SubmapCloudDown2_g (new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr SubmapCloud2_t (new pcl::PointCloud<Point_T>());
    pcl::PointCloud<Point_T>::Ptr SubmapCloud2_g (new pcl::PointCloud<Point_T>());
	
	Eigen::Matrix4f transformation2to1f;
	Eigen::Matrix4f transformation2to1g= edge.submap1.oxts_noise_pose.inverse() * edge.submap2.oxts_noise_pose;
    
	cout<<"Initial Guess of OXTS"<<endl<< transformation2to1g<<endl;
    
	//Fix it later: Divide into ADJACENT AND REVISIT for speeding up
	CRegistration <Point_T> reg;
	pcl::transformPointCloud(*edge.submap2.pointcloud_lidar_down, *SubmapCloudDown2_g, transformation2to1g);
	//reg.transformcloud(edge.submap2.pointcloud_lidar_down, SubmapCloudDown2_g, transformation2to1g); //intial guess
	
	//Registration (point to plane Trimmed-ICP)
	//edge.confidence = 1.0 / reg.ptplicp_reg(SubmapCloudDown2_g, edge.submap1.pointcloud_lidar_down, SubmapCloudDown2_t, transformation2to1f, 15, 0, 1, 1.5f, 1.0f, LLS); 

    edge.confidence = 1.0 / reg.icp_registration(SubmapCloudDown2_g, edge.submap1.pointcloud_lidar_down, SubmapCloudDown2_t, transformation2to1f,
	                  Point2Plane, NN, LLS, 0, 0, 12, 0.5f, 10);

	edge.Trans1_2 =  transformation2to1f * transformation2to1g; //Trans1_2:  2 to 1
	
	cout<<"Final Registration result"<<endl<< edge.Trans1_2<<endl;
    
	//Assign Weight
	//(Fix this later)
    //edge.confidence= 1.0/fitness_score;
	//edge.confidence=1;
    switch (edge.type)
	{
		case ADJACENT:
			edge.confidence*=1.0;
			break;
		case REVISIT:
            edge.confidence*=1.0;
		    break;
		default:
			break;
	}

	// For display
    // DataIo <Point_T> io;
	// if(edge.type==REVISIT && edge.unique_id%10==0)
	// {
    //    io.display2clouds (edge.submap1.pointcloud_lidar_down,SubmapCloudDown2_g,"Before Registration", 1);
	//    io.display2clouds (edge.submap1.pointcloud_lidar_down,SubmapCloudDown2_t,"After Submap Registration", 1);
	// }
	
	
	pcl::PointCloud<Point_T>().swap(*SubmapCloudDown2_g);      // Free the Memory
	pcl::PointCloud<Point_T>().swap(*SubmapCloudDown2_t);      // Free the Memory
  
	return 1;
}


void Constraint_Finder::add_noise(Frame &frame, float noise_std_t, float noise_std_r)
{
	//const float mean_t=0.0, mean_r=0.0;
	//const float mean_tx=0.0, mean_ty=0.0, mean_tz=0.0;
    
	//Normal Distribution: Some Problem
	//std::default_random_engine noise_generator;
	//std::normal_distribution<float> dist (mean_t, noise_std_t);
    //std::normal_distribution<float> ang (mean_r, noise_std_r);

    // frame.oxts_noise_pose(0,3)=frame.oxts_pose(0,3)+ dist(noise_generator);
	// frame.oxts_noise_pose(1,3)=frame.oxts_pose(1,3)+ dist(noise_generator);
	// frame.oxts_noise_pose(2,3)=frame.oxts_pose(2,3)+ dist(noise_generator);
    
	frame.oxts_noise_pose(0,3)= frame.oxts_pose(0,3)+ 0.8 * (0.5 * noise_std_t - noise_std_t * ((double)rand() / RAND_MAX)); //Not too much horizontal error compared with vertical one
	frame.oxts_noise_pose(1,3)= frame.oxts_pose(1,3)+ 0.8 * (0.5 * noise_std_t - noise_std_t * ((double)rand() / RAND_MAX));
	frame.oxts_noise_pose(2,3)=frame.oxts_pose(2,3)+ 0.5 * noise_std_t - noise_std_t * ((double)rand() / RAND_MAX);

	float noise_roll, noise_pitch , noise_yaw;
	// noise_roll = ang(noise_generator)/180.0*M_PI;
	// noise_pitch = ang(noise_generator)/180.0*M_PI;
	// noise_yaw = ang(noise_generator)/180.0*M_PI;
    

	noise_roll = (0.5 * noise_std_r -noise_std_r * ((double)rand() / RAND_MAX))/180.0*M_PI;    //Not too much roll error  ?
	noise_pitch = (0.5 * noise_std_r - noise_std_r * ((double)rand() / RAND_MAX))/180.0*M_PI;  //Not too much pitch error ?
	noise_yaw = (0.5 * noise_std_r - noise_std_r * ((double)rand() / RAND_MAX))/180.0*M_PI;
    
	Eigen::Matrix3f Rx, Ry, Rz;
	Rx<<1,0,0,
	    0,cos(noise_roll),-sin(noise_roll),
		0,sin(noise_roll),cos(noise_roll);
	
	Ry<<cos(noise_pitch),0,sin(noise_pitch),
	    0,1,0,
		-sin(noise_pitch),0,cos(noise_pitch);
	
	Rz<<cos(noise_yaw),-sin(noise_yaw),0,
	    sin(noise_yaw),cos(noise_yaw),0,
		0,0,1;

	Eigen::Matrix3f rot_noise_Matrix= Rz*Ry*Rx;

	frame.oxts_noise_pose.block<3,3>(0,0) = rot_noise_Matrix * frame.oxts_pose.block<3,3>(0,0);
	frame.oxts_noise_pose.block<1,4>(3,0) = frame.oxts_pose.block<1,4>(3,0);
}

void Constraint_Finder::back_end(Transaction &transaction, int index_min_interval, float max_revisit_pos_distance, float min_iou)
{
    DataIo <Point_T> io;

	batch_find_hdmap_constraints_submap(transaction, index_min_interval, max_revisit_pos_distance, min_iou);   // interval frame , max_distance
    //Display the pose graph
	io.display_hdmap_edges(transaction.submap_edges);
    
	for (int i=0; i< transaction.submap_edges.size();i++)
	{
		transaction.submap_edges[i].unique_id=i;
		printf("-----------Map to Map registration----------\n");
		printf("Do the registration between submap [%d] and submap [%d].\n",transaction.submap_edges[i].submap1.id_in_transaction, transaction.submap_edges[i].submap2.id_in_transaction);
		pairwiseReg(transaction.submap_edges[i]);
	}
    
    // Free the memory
	for (int i=0; i<transaction.submap_number;i++)
	{
		pcl::PointCloud<Point_T>().swap(*transaction.submaps[i].pointcloud_lidar_down);
	}

    // Graph Optimization
	printf("Do Graph Optimization\n");
	
}

bool Constraint_Finder::merge_frame_bounds_and_centers(SubMap & submap)
{
    submap.boundingbox=submap.frames[0].boundingbox;
	for (int i=1; i < submap.frame_number;i++)
	{
         if (submap.frames[i].boundingbox.max_x>submap.boundingbox.max_x) submap.boundingbox.max_x=submap.frames[i].boundingbox.max_x;
		 if (submap.frames[i].boundingbox.max_y>submap.boundingbox.max_y) submap.boundingbox.max_y=submap.frames[i].boundingbox.max_y;
		 if (submap.frames[i].boundingbox.max_z>submap.boundingbox.max_z) submap.boundingbox.max_z=submap.frames[i].boundingbox.max_z;
         if (submap.frames[i].boundingbox.min_x<submap.boundingbox.min_x) submap.boundingbox.min_x=submap.frames[i].boundingbox.min_x;
		 if (submap.frames[i].boundingbox.min_y<submap.boundingbox.min_y) submap.boundingbox.min_y=submap.frames[i].boundingbox.min_y;
		 if (submap.frames[i].boundingbox.min_z<submap.boundingbox.min_z) submap.boundingbox.min_z=submap.frames[i].boundingbox.min_z;
	}
	submap.centerpoint.x=0.5*(submap.boundingbox.max_x+submap.boundingbox.min_x);
	submap.centerpoint.y=0.5*(submap.boundingbox.max_y+submap.boundingbox.min_y);
	submap.centerpoint.z=0.5*(submap.boundingbox.max_z+submap.boundingbox.min_z);
	printf("Submap bounding box: x [%lf - %lf] y [%lf - %lf] z [%lf - %lf]\n", submap.boundingbox.min_x, submap.boundingbox.max_x, submap.boundingbox.min_y, 
	submap.boundingbox.max_y, submap.boundingbox.min_z, submap.boundingbox.max_z);
}

float Constraint_Finder::getMAE(SubMap & submap)
{
	float dis_dif=0;
	for (int i=0; i< submap.frame_number; i++)
	{
		Eigen::Vector3f position_dif = submap.frames[i].oxts_pose.block<3,1>(0,3) - submap.frames[i].odom_pose.block<3,1>(0,3);
		dis_dif += sqrt(position_dif(0)*position_dif(0)+position_dif(1)*position_dif(1)+position_dif(2)*position_dif(2));
	}
	return (dis_dif/submap.frame_number);
}