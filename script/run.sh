#data path
ALS_data_path=/media/edward/Seagate/Data/Highway_dataset/ALS/blocks;
TLS_data_path=/media/edward/Seagate/Data/Highway_dataset/TLS/utm48-las;
MLS_data_path=/media/edward/Seagate/Data/Highway_dataset/MLS/UTM48_tran-las;
BPLS_data_path=/media/edward/Seagate/Data/Highway_dataset/BPLS/UTM48_tran-las;

#parameters
find_constraint_knn=10;              
find_constraint_overlap_ratio=0.2;
downsample_voxels_size_als=0.2;
downsample_voxels_size_tls=0.1;
downsample_voxels_size_mls=0.1;
downsample_voxels_size_bpls=0.1;
registration_overlap_search_radius=0.5;
registration_min_overlap_ratio=0.2;
registration_use_reciprocal_corres=0;
registration_use_use_trimmed_rejector=1;
registration_perturbate_dis=0.0;


#run
#gdb --args \ 
./bin/ccn ${ALS_data_path} ${TLS_data_path} ${MLS_data_path} ${BPLS_data_path} \
${find_constraint_knn} ${find_constraint_overlap_ratio} \
${downsample_voxels_size_als} ${downsample_voxels_size_tls} ${downsample_voxels_size_mls} ${downsample_voxels_size_bpls} \
${registration_overlap_search_radius} ${registration_min_overlap_ratio} ${registration_perturbate_dis} \
${registration_use_reciprocal_corres} ${registration_use_use_trimmed_rejector} 

