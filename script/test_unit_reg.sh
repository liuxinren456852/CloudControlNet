#data path
#tpc_path=/home/edward/testdata/1-3-11-ALS.las;
#spc_path=/home/edward/testdata/2-1-11-ALS.las;
#opc_path=/home/edward/testdata/2-1-11-ALS-t.las;

tpc_path=/home/edward/testdata/4-1-TLS.las;
spc_path=/home/edward/testdata/1-3-2-ALS.las;
opc_path=/home/edward/testdata/1-3-2-ALS_t.las;

#run
#gdb --args \
./bin/test_reg \
--colorlogtostderr=true \
-stderrthreshold 0 \
-log_dir ./log/test \
--v=10 \
--target_point_cloud_path=${tpc_path} \
--source_point_cloud_path=${spc_path} \
--output_point_cloud_path=${opc_path} \
--target_down_res=0.1 \
--source_down_res=0.1 \
--target_pca_neigh_radius=1.5 \
--source_pca_neigh_radius=1.5 \
--gf_grid_size=2.0 \
--gf_in_grid_h_thre=0.3 \
--gf_neigh_grid_h_thre=2.5 \
--gf_max_h=25.0 \
--linearity_thre=0.6 \
--planarity_thre=0.6 \
--spherity_thre=0.55

