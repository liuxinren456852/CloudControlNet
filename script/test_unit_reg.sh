#data path
#target_point_cloud_path=/home/edward/testdata/1-3-11-ALS.las;
#source_point_cloud_path=/home/edward/testdata/2-1-11-ALS.las;
#output_point_cloud_path=/home/edward/testdata/2-1-11-ALS-t.las;

target_point_cloud_path=/home/edward/testdata/4-1-TLS.las;
source_point_cloud_path=/home/edward/testdata/1-3-2-ALS.las;
output_point_cloud_path=/home/edward/testdata/1-3-2-ALS_t.las;

# key parameters
downsample_resolution_s=0.2;
downsample_resolution_t=0.1;
gf_grid_resolution=2.0;
gf_max_grid_height_diff=0.3;
gf_neighbor_height_diff=2.5;
neighbor_search_radius=1.5;

#run
./bin/test_reg ${target_point_cloud_path} ${source_point_cloud_path} ${output_point_cloud_path} \
${downsample_resolution_s} ${downsample_resolution_t} \
${gf_grid_resolution} ${gf_max_grid_height_diff} ${gf_neighbor_height_diff} \
${neighbor_search_radius} 