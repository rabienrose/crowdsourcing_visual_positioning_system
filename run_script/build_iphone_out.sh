OUT_ADDR=$1
BAG_NAME=$2


#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/office_2.bag
#OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/test_android
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

echo working directory ${OUT_ADDR}
echo bag address ${BAG_NAME}
bash ./run_script/run_vslam.sh ${BAG_NAME} ${OUT_ADDR}
mkdir ${OUT_ADDR}/local
bash ./run_script/run_conv_map.sh ${OUT_ADDR} ${OUT_ADDR}/local
#bash ./run_script/run_backend.sh ${OUT_ADDR}/local ${OUT_ADDR}/local "BA"
#bash ./run_script/run_backend.sh ${OUT_ADDR}/local ${OUT_ADDR}/local "Reset"
