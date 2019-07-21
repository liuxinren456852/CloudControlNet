#include "filter.h"

template<typename PointT>
bool CFilter<PointT>::VoxelDownsample(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, float voxel_size)
{
    clock_t t0, t1;
	t0=clock();

    float inverse_voxel_size = 1.0f / voxel_size;

	Eigen::Vector4f min_p, max_p;
	pcl::getMinMax3D(*cloud_in, min_p, max_p);

	Eigen::Vector4f gap_p;  //boundingbox gap;
	gap_p = max_p - min_p;
		
	unsigned long long max_vx = ceil(gap_p.coeff(0)*inverse_voxel_size)+1;
	unsigned long long max_vy = ceil(gap_p.coeff(1)*inverse_voxel_size)+1;
	unsigned long long max_vz = ceil(gap_p.coeff(2)*inverse_voxel_size)+1;
		
	if (max_vx*max_vy*max_vz >= std::numeric_limits<unsigned long long>::max())
	{
		std::cout << "Filtering Failed: The number of box exceed the limit."<<endl;
		return 0;
	}

	unsigned long long mul_vx = max_vy*max_vz;
	unsigned long long mul_vy = max_vz;
	unsigned long long mul_vz = 1;

	std::vector<IDPair> id_pairs(cloud_in->size());
	unsigned int idx = 0;
	for (typename pcl::PointCloud<PointT>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++)
	{

	   unsigned long long vx = floor((it->x - min_p.coeff(0))*inverse_voxel_size);
	   unsigned long long vy = floor((it->y - min_p.coeff(1))*inverse_voxel_size);
	   unsigned long long vz = floor((it->z - min_p.coeff(2))*inverse_voxel_size);

	   unsigned long long voxel_idx = vx*mul_vx + vy*mul_vy + vz*mul_vz;

	   IDPair pair;
	   pair.idx = idx;
	   pair.voxel_idx = voxel_idx;
	   id_pairs.push_back(pair);
	   idx++;
	}

	//Do sorting
	std::sort(id_pairs.begin(), id_pairs.end());

	unsigned int begin_id = 0;

	while (begin_id < id_pairs.size())
	{
		cloud_out->push_back(cloud_in->points[id_pairs[begin_id].idx]);

		unsigned int compare_id = begin_id + 1;
		while (compare_id < id_pairs.size() && id_pairs[begin_id].voxel_idx == id_pairs[compare_id].voxel_idx) compare_id++;
		begin_id = compare_id;
	}

    t1=clock();
    cout << "Voxel downsampling done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl; 
	return 1;
}

//Normal Space Downsampling with radius neighbor search
template<typename PointT>
bool CFilter<PointT>::NormalDownsample(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out,float neighbor_radius,float downsample_ratio)
{
    clock_t t0, t1;
	t0=clock();

    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	copyPointCloud(*cloud_in, *CloudXYZ);
	// In this case, The Cloud's Normal hasn't been calculated yet. 
    
	pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal_out(new pcl::PointCloud<pcl::PointNormal>());

	//Estimate Normal Multi-thread
	PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator; 
	
	//Radius search
	pca_estimator.CalculatePointCloudWithNormal_Radius(CloudXYZ, neighbor_radius, CloudNormal);  

    pcl::NormalSpaceSampling<pcl::PointNormal,pcl::PointNormal> nss;
    nss.setInputCloud(CloudNormal);
    nss.setNormals(CloudNormal);
    nss.setBins(4,4,4);
    nss.setSeed(0);
    nss.setSample(static_cast<unsigned int> (downsample_ratio * CloudNormal->size()));
     
    nss.filter(*CloudNormal_out);
    copyPointCloud(*CloudNormal_out, *cloud_out);
    
    t1=clock();
    cout << "Normal space downsampling done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl; 
	return 1;
}

//Normal Space Downsampling with KNN neighbor search
template<typename PointT>
bool CFilter<PointT>::NormalDownsample(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out,int K,float downsample_ratio)
{
    clock_t t0, t1;
	t0=clock();

    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	copyPointCloud(*cloud_in, *CloudXYZ);
	// In this case, The Cloud's Normal hasn't been calculated yet. 
    
	pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal_out(new pcl::PointCloud<pcl::PointNormal>());

	//Estimate Normal Multi-thread
	PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator; 
	 
    //K search 
    pca_estimator.CalculatePointCloudWithNormal_KNN(CloudXYZ, K, CloudNormal);  
    
    pcl::NormalSpaceSampling<pcl::PointNormal,pcl::PointNormal> nss;
    nss.setInputCloud(CloudNormal);
    nss.setNormals(CloudNormal);
    nss.setBins(4,4,4);
    nss.setSeed(0);
    nss.setSample(static_cast<unsigned int> (downsample_ratio * CloudNormal->size()));
     
    nss.filter(*CloudNormal_out);
    copyPointCloud(*CloudNormal_out, *cloud_out);
    
    t1=clock();
    cout << "Normal space downsampling done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl; 
	return 1;
}

//SOR (Statisics Outliers Remover);
template<typename PointT>
bool CFilter<PointT>::SORFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in,  typename pcl::PointCloud<PointT>::Ptr & cloud_out, int MeanK, double std)  
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<PointT> sor;
         
	sor.setInputCloud(cloud_in);
	sor.setMeanK(MeanK);         //50
	sor.setStddevMulThresh(std); //2.0
	sor.filter(*cloud_out);

    return 1;  
}

template<typename PointT>
bool CFilter<PointT>::RoughClassify(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_edge, typename pcl::PointCloud<PointT>::Ptr & cloud_planar, typename pcl::PointCloud<PointT>::Ptr & cloud_sphere,
                                    typename pcl::PointCloud<PointT>::Ptr & cloud_edge_down, typename pcl::PointCloud<PointT>::Ptr & cloud_planar_down, typename pcl::PointCloud<PointT>::Ptr & cloud_sphere_down,
                                    float neighbor_radius, int neigh_num_thre, float edge_thre, float planar_thre, float sphere_thre, float edge_thre_down, float planar_thre_down, float sphere_thre_down)
{
	clock_t t0, t1;
	t0=clock();
	
	//Estimate Normal Multi-thread
	PrincipleComponentAnalysis<PointT> pca_estimator; 
	vector<pcaFeature> cloud_features;
	pca_estimator.CalculatePcaFeaturesOfPointCloud(cloud_in,neighbor_radius,cloud_features);

	for (int i=0; i< cloud_in->points.size();i++)
	{
		if (cloud_features[i].ptNum > neigh_num_thre)
		{
			if(cloud_features[i].linear_2 > edge_thre)
			{
                cloud_edge->points.push_back(cloud_in->points[i]);
				if(cloud_features[i].linear_2 > edge_thre_down)
				{
					cloud_edge_down->points.push_back(cloud_in->points[i]);
				}
			}
			else if(cloud_features[i].planar_2 > planar_thre)
			{
                cloud_planar->points.push_back(cloud_in->points[i]);
				if(cloud_features[i].planar_2 > planar_thre_down)
				{
					cloud_planar_down->points.push_back(cloud_in->points[i]);
				}
			}
			else if(cloud_features[i].spherical_2 > sphere_thre)
			{
                cloud_sphere->points.push_back(cloud_in->points[i]);
				if(cloud_features[i].spherical_2 > sphere_thre_down)
				{
					cloud_sphere_down->points.push_back(cloud_in->points[i]);
				}
			}			
		}
	}

    t1=clock(); 
	printf("Edge [%d | %d] Planar [%d | %d] Sphere [%d | %d]\n", cloud_edge->points.size(), cloud_edge_down->points.size(), cloud_planar->points.size(), cloud_planar_down->points.size(),cloud_sphere->points.size(), cloud_sphere_down->points.size());
    printf("Feature points extracted done in %lf s\n", (float(t1 - t0) / CLOCKS_PER_SEC));
    return 1;
}

//Filter the point cloud according to the horizontal and vertical distance to the lidar center
template<typename PointT>
bool CFilter<PointT>::DisFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, double xy_dis_max, double z_min, double z_max)  
{
	clock_t t0, t1;
	t0=clock();
    double dis_square;
	for (int i=0; i< cloud_in->points.size();i++)
	{
		dis_square=cloud_in->points[i].x*cloud_in->points[i].x + cloud_in->points[i].y + cloud_in->points[i].y;
		if (dis_square < xy_dis_max*xy_dis_max /* && cloud_in->points[i].z<z_max */ && cloud_in->points[i].z>z_min)
		{cloud_out->points.push_back(cloud_in->points[i]);}
	}
    t1=clock();
    cout << "Distance Filter done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl; 
    return 1;
}

template<typename PointT>
bool CFilter<PointT>::ActiveObjectFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, std::vector<Bounds> & active_bbxs)  
{
    vector<bool> is_static(cloud_in->points.size(),1);
    for (int i=0; i<cloud_in->points.size();i++)
    {
       for (int j=0; j<active_bbxs.size();j++)
       {
           //In the bounding box
           if(cloud_in->points[i].x > active_bbxs[j].min_x && cloud_in->points[i].x < active_bbxs[j].max_x &&
              cloud_in->points[i].y > active_bbxs[j].min_y && cloud_in->points[i].y < active_bbxs[j].max_y &&
              cloud_in->points[i].z > active_bbxs[j].min_z && cloud_in->points[i].z < active_bbxs[j].max_z )
           {is_static[i]=0; break;}
       }
       if(is_static[i]) cloud_out->points.push_back(cloud_in->points[i]);   
    }
    
	return 1;
}


template<typename PointT>
bool CFilter<PointT>::FastGroundFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_ground,
                                       typename pcl::PointCloud<PointT>::Ptr & cloud_ground_down, typename pcl::PointCloud<PointT>::Ptr & cloud_unground,
                                       int min_grid_num, float grid_resolution,float max_height_difference, float neighbor_height_diff, float max_ground_height, 
	                                   int ground_random_downsample_rate_first, int ground_random_downsample_rate_second,  int nonground_random_downsample_rate)
{
    clock_t t0, t1, t2;
	t0=clock();
    
    PrincipleComponentAnalysis<PointT> pca_estimator; 

    Bounds bounds;
    CenterPoint center_pt;
    this->getBoundAndCenter(*cloud_in , bounds , center_pt); //Inherited from its parent class, use this->
    
    //Construct Grid
    int row, col, num_grid;  
	row = ceil((bounds.max_y - bounds.min_y) / grid_resolution);
	col = ceil((bounds.max_x - bounds.min_x) / grid_resolution);
	num_grid = row*col;
	
    Grid* grid = new Grid[num_grid];

    //Each grid
    for (int i = 0; i < num_grid; i++) 
    {
        grid[i].min_z = FLT_MAX;
        grid[i].NeighborMin_z = FLT_MAX;
    }
    
    //Each point
    for (int j = 0; j < cloud_in->points.size(); j++)
	{
		int temp_row, temp_col, temp_id;
		temp_col = floor((cloud_in->points[j].x - bounds.min_x) / grid_resolution);
		temp_row = floor((cloud_in->points[j].y - bounds.min_y) / grid_resolution);
		temp_id = temp_row * col + temp_col;
		if (temp_id >= 0 && temp_id < num_grid)
		{
			grid[temp_id].PointsNumber++;
			if (cloud_in->points[j].z > max_ground_height)
            {
               cloud_unground->points.push_back(cloud_in->points[j]);
            }
            else{
                 grid[temp_id].point_id.push_back(j);
			     if (cloud_in->points[j].z < grid[temp_id].min_z)
			     {
				    grid[temp_id].min_z = cloud_in->points[j].z;
				    grid[temp_id].NeighborMin_z = cloud_in->points[j].z;
			     }
            }      
		}
	}
    
    //Each grid
    for (int i = 0; i < num_grid; i++)
	{   
        int temp_row, temp_col;
        temp_row = i / col;
		temp_col = i % col;
        if (temp_row >= 1 && temp_row <= row-2  && temp_col>=1 && temp_col <= col-2)
        {
           for (int j = -1; j <= 1; j++) //row
		   {
			  for (int k = -1; k <= 1 ; k++) //col
			  {
				if (grid[i].NeighborMin_z > grid[i + j * col + k].min_z) grid[i].NeighborMin_z = grid[i + j * col + k].min_z;
			  }
		   }
        }
	}
    //Each grid
    for (int i = 0; i < num_grid; i++)
	{
		//Filtering some grids with too little points
		if (grid[i].PointsNumber >= min_grid_num)
		{
            if (grid[i].min_z - grid[i].NeighborMin_z < neighbor_height_diff) 
            {
               for (int j = 0; j < grid[i].point_id.size(); j++)
		       {
			      if (cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_difference) 
			      {
				     if(j%ground_random_downsample_rate_first==0) // for example 10
					 {
						 
						//  cloud_in->points[grid[i].point_id[j]].normal_x=0.0;
						//  cloud_in->points[grid[i].point_id[j]].normal_y=0.0;
						//  cloud_in->points[grid[i].point_id[j]].normal_z=1.0;

						 cloud_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to ground points
						 if(j%ground_random_downsample_rate_second==0)  // for example 40
						 {
							 cloud_ground_down->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to down ground points 
						 }
					 }
			      }
			      else 
			      {
				     if(j%nonground_random_downsample_rate==0) cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
			      }
		        }
            }
            else 
            {
               for (int j = 0; j < grid[i].point_id.size(); j++)  
			   {
				    if(j%nonground_random_downsample_rate==0) cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
			   } //Add to nonground points
            }
		}	
	}

	delete[]grid;
    
    t1=clock();
    
	pcl::PointCloud<pcl::Normal>::Ptr ground_normal(new pcl::PointCloud<pcl::Normal>);
	pca_estimator.CalculateNormalVector_KNN(cloud_ground, 8, ground_normal);
	for (int i=0; i< cloud_ground->points.size();i++)
	{
		cloud_ground->points[i].normal_x=ground_normal->points[i].normal_x;
		cloud_ground->points[i].normal_y=ground_normal->points[i].normal_y;
		cloud_ground->points[i].normal_z=ground_normal->points[i].normal_z;
		//if (i%500==0) printf("normal: %lf, %lf, %lf\n", cloud_ground->points[i].normal_x, cloud_ground->points[i].normal_y, cloud_ground->points[i].normal_z ); 
	}     
    
	t2=clock();

    printf("Ground [%d | %d] UnGround [%d] \n", cloud_ground->points.size(), cloud_ground_down->points.size(), cloud_unground->points.size());
    printf("Ground Filter done in %lf s\n", (float(t1 - t0) / CLOCKS_PER_SEC));
	printf("Ground Normal Estimation done in %lf s\n", (float(t2 - t1) / CLOCKS_PER_SEC));
    
    return 1;
}


template<typename PointT>
bool CFilter<PointT>::GroundFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_ground, typename pcl::PointCloud<PointT>::Ptr & cloud_unground ,float grid_resolution,float max_height_difference)
{
    clock_t t0, t1;
	t0=clock();

    Bounds bounds;
    CenterPoint center_pt;
    this->getBoundAndCenter(*cloud_in , bounds , center_pt); //Inherited from its parent class, use this->
    
    int row, col, num_grid;  
	row = ceil((bounds.max_y - bounds.min_y) / grid_resolution);
	col = ceil((bounds.max_x - bounds.min_x) / grid_resolution);
	num_grid = row*col;
    
	Grid* grid = new Grid[num_grid];
	for (int i = 0; i < num_grid; i++) 
    {
        grid[i].min_z = FLT_MAX;
        grid[i].NeighborMin_z = FLT_MAX;
    }
    
    //Preprocessing
	preprocessing(cloud_in, bounds, row, col, num_grid, grid, grid_resolution);
    
    //Processing
	processing(cloud_in, cloud_ground, cloud_unground, grid, num_grid, grid_resolution, max_height_difference);
    
    //Post processing
	postprocessing(cloud_in, cloud_ground, cloud_unground);

	delete[]grid;
    
    t1=clock();

    cout << "Ground Filter done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl; 
    
    return 1;
}

template<typename PointT>
void CFilter<PointT>::preprocessing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, Bounds & bounds, int row, int col, int num_grid, Grid* grid, float grid_resolution)
{
    int temp_num_voxel, ExpandGrid_Col, ExpandGrid_Row;
	ExpandGrid_Col = col + 2;
	ExpandGrid_Row = row + 2;
	temp_num_voxel = ExpandGrid_Row*ExpandGrid_Col;
	Grid *temp_grid = new Grid[temp_num_voxel];

	for (int i = 0; i < cloud_in->points.size(); i++)
	{
		int temp_row, temp_list, temp_id;
		temp_list = floor((cloud_in->points[i].x - bounds.min_x) / grid_resolution);
		temp_row = floor((cloud_in->points[i].y - bounds.min_y) / grid_resolution);
		temp_id = temp_row * col + temp_list;
		if (temp_id >= 0 && temp_id < num_grid)
		{
			grid[temp_id].point_id.push_back(i);
			grid[temp_id].PointsNumber++;
			if (cloud_in->points[i].z < grid[temp_id].min_z)
			{
				grid[temp_id].min_z = cloud_in->points[i].z;
				grid[temp_id].NeighborMin_z = cloud_in->points[i].z;
			}
		}
	}

	for (int i = 0; i < num_grid; i++)
	{
		//Ok .fix it later. It's too ...
        int ExpandGrid_TempRow, ExpandGrid_TempCol, ExpandGrid_Temp_id;
		ExpandGrid_TempRow = i / col + 1;
		ExpandGrid_TempCol =i % col + 1;
		ExpandGrid_Temp_id = ExpandGrid_TempRow*ExpandGrid_Col + ExpandGrid_TempCol;
		temp_grid[ExpandGrid_Temp_id].min_z = grid[i].min_z;
		if (ExpandGrid_TempCol == 1 || ExpandGrid_TempRow == 1 || ExpandGrid_TempCol == col || ExpandGrid_TempRow == row)
		{
			if (ExpandGrid_TempCol == 1)
			{
				temp_grid[ExpandGrid_Temp_id - 1].min_z = grid[i].min_z;
				if (ExpandGrid_TempRow == 1)
					temp_grid[ExpandGrid_Temp_id - 1 - ExpandGrid_Col].min_z = grid[i].min_z;
			}
			else
			{
				if (ExpandGrid_TempCol == col)
				{
					temp_grid[ExpandGrid_Temp_id + 1].min_z = grid[i].min_z;
					if (ExpandGrid_TempRow == row)
						temp_grid[ExpandGrid_Temp_id + 1 + ExpandGrid_Col].min_z = grid[i].min_z;
				}
			}
			if (ExpandGrid_TempRow == 1)
			{
				temp_grid[ExpandGrid_Temp_id - ExpandGrid_Col].min_z = grid[i].min_z;
				if (ExpandGrid_TempCol == col)
					temp_grid[ExpandGrid_Temp_id + 1 - ExpandGrid_Col].min_z = grid[i].min_z;
			}
			else
			{
				if (ExpandGrid_TempRow == row)
				{
					temp_grid[ExpandGrid_Temp_id + ExpandGrid_Col].min_z = grid[i].min_z;
					if (ExpandGrid_TempCol == 1)
						temp_grid[ExpandGrid_Temp_id - 1 + ExpandGrid_TempCol].min_z = grid[i].min_z;

				}
			}
		}
	}
	
    for (int i = 0; i < num_grid; i++)
	{
		int ExpandGrid_TempRow, ExpandGrid_TempCol, ExpandGrid_Temp_id;
		ExpandGrid_TempRow = i / col + 1;
		ExpandGrid_TempCol = i % col + 1;
		ExpandGrid_Temp_id = ExpandGrid_TempRow * ExpandGrid_Col + ExpandGrid_TempCol;
		for (int j = -1; j < 2; j++) //Col
		{
			for (int k = -1; k<2; k++) //row
			{
				if (grid[i].NeighborMin_z > temp_grid[ExpandGrid_Temp_id + j*ExpandGrid_Col + k].min_z && (j != 0 || k != 0))
					grid[i].NeighborMin_z = temp_grid[ExpandGrid_Temp_id + j*ExpandGrid_Col + k].min_z;
			}
		}
	}
	delete[] temp_grid;
}

template<typename PointT>
void CFilter<PointT>::processing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_ground, typename pcl::PointCloud<PointT>::Ptr & cloud_unground,
			Grid* grid, int num_grid, float grid_resolution, float max_height_difference)
{
	for (int i = 0; i < num_grid; i++)
	{
		if (grid[i].min_z - grid[i].NeighborMin_z < 2 * grid_resolution) 
        {
            for (int j = 0; j < grid[i].point_id.size(); j++)
		    {
			   if (cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_difference) //Add to ground points
			   {
				cloud_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
			   }
			   else //Add to nonground points
			   {
				cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
			   }
		    }
        }
        else //Add to nonground points
        {
            for (int j = 0; j < grid[i].point_id.size(); j++)  {cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]);}
        }
	}
}


template<typename PointT>
void CFilter<PointT>::postprocessing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in,
			typename pcl::PointCloud<PointT>::Ptr & cloud_ground,
			typename pcl::PointCloud<PointT>::Ptr & cloud_unground)

{
    typename pcl::PointCloud<PointT>::Ptr temp_ground_cloud(new typename pcl::PointCloud<PointT>());
	std::vector<int> is_ground_points;
	
    PointT groundpointsTotempgroudpoints;

	for (int i = 0; i < cloud_ground->points.size(); i++)
	{
		groundpointsTotempgroudpoints = cloud_ground->points[i];
		temp_ground_cloud->push_back(groundpointsTotempgroudpoints);
		is_ground_points.push_back(i);
	}
	cloud_ground->clear();
	
    std::set<int, less<int> > Unsearch;
	std::set<int, less<int> >::iterator iterUnsearch;

	std::vector<int> isearse;
	
    for (int i = 0; i < temp_ground_cloud->points.size(); i++)
	{
		isearse.push_back(i);
		Unsearch.insert(i);
	}

	pcl::KdTreeFLANN<PointT> Kdtree_search_ground_cloud;
	pcl::KdTreeFLANN<PointT> Kdtree_search_cloud;

	Kdtree_search_ground_cloud.setInputCloud(temp_ground_cloud);
	Kdtree_search_cloud.setInputCloud(cloud_in);
	
    //Radius search
    float radius = 1.0;
	std::vector<int>PointIdSearch_ground_cloud;
	std::vector<float>PointDistanceSearch_ground_cloud;
	std::vector<int>PointIdSearch_cloud;
	std::vector<float>PointDistanceSearch_cloud;
	
    PointT Searchpoint;
	int Pointsub;

	while (!Unsearch.empty()) //Still some points not searched
	{
		iterUnsearch = Unsearch.begin();
		Pointsub = *iterUnsearch;
		Searchpoint = temp_ground_cloud->points[Pointsub];
		Kdtree_search_ground_cloud.radiusSearch(Searchpoint, radius, PointIdSearch_ground_cloud, PointDistanceSearch_ground_cloud); //Radius search
		Unsearch.erase(Pointsub); // Erase from the set

        //Empricial Settings
        if (PointIdSearch_ground_cloud.size() < 5)  //Less than 5 ground points in 1 meter neighborhood
		{
			if (isearse[Pointsub] != -1)  //->non ground 
			{
				is_ground_points[Pointsub] = -2;  
				isearse[Pointsub] = -1;
			}
		}           
		else
		{
			if (PointIdSearch_ground_cloud.size() > 10) //More than 10 ground points in 1 meter neighborhood // keep ground 
			{
				for (int i = 0; i < PointIdSearch_ground_cloud.size(); i++)
				{
					if (isearse[PointIdSearch_ground_cloud[i]] != -1) 
					{
						Unsearch.erase(PointIdSearch_ground_cloud[i]); // Neighborhood ground points erased from the search set
						isearse[PointIdSearch_ground_cloud[i]] = -1;
					}
				}
			}
			else //   >=5 && <=10 
			{
				Kdtree_search_cloud.radiusSearch(Searchpoint, radius, PointIdSearch_cloud, PointDistanceSearch_cloud); //Search All the points (ground and unground)
				if (PointIdSearch_cloud.size() > 2 * PointIdSearch_ground_cloud.size()) // Non-ground more than ground
				{
					for (int i = 0; i < PointIdSearch_ground_cloud.size(); i++)
					{
						if (isearse[PointIdSearch_ground_cloud[i]] != -1)
						{
							is_ground_points[PointIdSearch_ground_cloud[i]] = -1;   // Neighborhood ground points -> non ground      
							Unsearch.erase(PointIdSearch_ground_cloud[i]);          // Neighborhood ground points erased from the search set
							isearse[PointIdSearch_ground_cloud[i]] = -1;
						}
					}
				}
				else //Ground more than non-ground   
				{
					if (isearse[Pointsub] != -1)  // keep ground 
					{
						//Unsearch.erase(Pointsub);  
						isearse[Pointsub] = -1;     
					}
				}
            }
		}
	}

    //Free the memory
	isearse.clear();
	vector<int>().swap(isearse);
	PointIdSearch_cloud.clear();
	vector<int>().swap(PointIdSearch_cloud);
	PointDistanceSearch_cloud.clear();
	vector<float>().swap(PointDistanceSearch_cloud);
	PointIdSearch_ground_cloud.clear();
	vector<int>().swap(PointIdSearch_ground_cloud);
	PointDistanceSearch_ground_cloud.clear();
	vector<float>().swap(PointDistanceSearch_ground_cloud);
	
    for (int i = 0; i < temp_ground_cloud->points.size(); i++)
	{
		if (is_ground_points[i]<0) // -2 definitly -1 bordeline
          {cloud_unground->points.push_back(temp_ground_cloud->points[i]);}
        else 
          {cloud_ground->push_back(temp_ground_cloud->points[i]);}
	}
	temp_ground_cloud->clear();
	pcl::PointCloud<PointT>().swap(*temp_ground_cloud);
	is_ground_points.clear();
	vector<int>().swap(is_ground_points);
}