BAG_NAME=$1
OUT_ADDR=$2


#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/try_coarse_gps/bag/07-25-15-44-57.bag
#OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test
EXE_ROOT=/home/chamo/Documents/work/chamo_vps

ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/mono_kitti
BAG_TOOL_ADDR=${EXE_ROOT}/devel/lib/bag_tool/bag_tool_exe

${ORB_SLAM_ADDR} \
    --bag_addr=${BAG_NAME} \
    --output_addr=${OUT_ADDR}/ \
    --voc_addr=${OUT_ADDR}/FreakAll.bin \
    --camera_config=${OUT_ADDR}/camera_config.txt \
    --image_topic=img \
    --min_frame=0 \
    --max_frame=30000 \
    --step_frame=3 \
    --use_orb=false \
    --feature_count=2000 \
    --feature_scale_factor=1.2 \
    --feature_level=8 \
    --min_match_count=100 \
    --max_step_KF=25 \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true

${BAG_TOOL_ADDR} --bag_addr=${BAG_NAME} --out_dir=${OUT_ADDR} --img_topic=img --imu_topic=imu --gps_topic=gps --isExtractImage=false
