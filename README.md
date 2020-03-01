# Cloud Control Net

### How to use 

1. Install dependent 3rd libraries 

[PCL(>=1.7)](https://github.com/PointCloudLibrary/pcl), [LibLas(Optional for las data IO)](https://github.com/libLAS/libLAS), [Proj4(Optional for geo-projection)](https://proj.org/), [g2o](https://github.com/RainerKuemmerle/g2o) or [ceres](http://ceres-solver.org/)


2. Compile
```
mkdir build
cd build
cmake ..
make 
```

3. Run
```
cd ..
# configure the script/run.sh file for editting the data path and key parameters
sh script/run.sh
```


### Application 1： ALS Point Cloud Refinement
 ALS refinement using TLS or MLS data for highway expansion and reconstruction egineering

 ![alt text](img/1.jpg)
 Generated Pose Graph (Red: ALS block, Green: TLS station, Blue: MLS block, Yellow: BPLS block, Cyan: Adjacent Smooth Edge, Purple: Overlapping Registration Edge)
 
 ![alt text](img/2.jpg)
 ALS correction result: (a) Before optimization (b) After optimization
 

 ### Application 2：Multi-Source Point Cloud Fusion
  ![alt text](img/4.jpg)
  Multi-template point cloud data fusion result (a) TLS (b) TLS+BPLS (c)TLS+BPLS+MLS (d)TLS+BPLS+MLS+ALS
 
 ### Application 3: HD-Map Point Cloud Joint Correction and Mapping (dev branch)
 
  ![alt text](img/Graph2.png)
 
 ### 'Control Cloud' multi-template point cloud pose graph optimization framework
  ![alt text](img/framework.jpg)
