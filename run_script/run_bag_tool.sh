BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/try_coarse_gps/bag/07-25-15-33-12.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test
EXE_ROOT=/home/chamo/Documents/work/chamo_vps

BAG_TOOL_ADDR=${EXE_ROOT}/devel/lib/bag_tool/bag_tool_exe

${BAG_TOOL_ADDR} --bag_addr=${BAG_NAME} --out_dir=${OUT_ADDR} --img_topic=img --imu_topic=imu --gps_topic=gps --isExtractImage=false
    
    
