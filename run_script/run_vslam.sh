BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_19_cloudy/6_19_cloudy_building_near.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test
EXE_ROOT=/home/chamo/Documents/work/chamo_vps

ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/mono_kitti

${ORB_SLAM_ADDR} \
    --bag_addr=${BAG_NAME} \
    --output_addr=${OUT_ADDR}/ \
    --voc_addr=${OUT_ADDR}/FreakAll.bin \
    --camera_config=${OUT_ADDR}/camera_config.txt \
    --image_topic=img \
    --min_frame=0 \
    --max_frame=10000 \
    --step_frame=3 \
    --use_orb=false \
    --feature_count=2000 \
    --feature_scale_factor=1.2 \
    --feature_level=8 \
    --min_match_count=200 \
    --max_step_KF=15 \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
