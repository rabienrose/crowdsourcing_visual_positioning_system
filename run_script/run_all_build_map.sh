ROOT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all
CONF_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_720_configs
BAG_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/build_right

EXE_ROOT=/home/chamo/Documents/work/chamo_vps
BACKEND_ADDR=${EXE_ROOT}/devel/lib/backend/backend

rm -rf ${ROOT_ADDR}
mkdir ${ROOT_ADDR}

map_bag_list=`ls ${BAG_ROOT}`
mkdir ${ROOT_ADDR}/global
cp ${CONF_ADDR}/words_projmat.fstream ${ROOT_ADDR}/global
for map_bag_addr in ${map_bag_list}
do
    map_name=`basename "${map_bag_addr}"`
    echo create map: ${map_name}
    mkdir ${ROOT_ADDR}/${map_name}
    cp ${CONF_ADDR}/* ${ROOT_ADDR}/${map_name}
    bash ./run_script/build_iphone_out.sh ${ROOT_ADDR}/${map_name} ${BAG_ROOT}/${map_bag_addr}
    bash ./run_script/run_merge_new.sh ${ROOT_ADDR}/global ${ROOT_ADDR}/${map_name}/local ${ROOT_ADDR}/global
    MAP_IDS=$(python ./run_script/get_mapids.py ${ROOT_ADDR}/${map_name}/local)
    ${BACKEND_ADDR} --res_root=${ROOT_ADDR}/global --out_root=${ROOT_ADDR}/global --op_type=Match --map_ids=${MAP_IDS} --project_mat_file=${ROOT_ADDR}/global/words_projmat.fstream
    ${BACKEND_ADDR} --res_root=${ROOT_ADDR}/global --out_root=${ROOT_ADDR}/global --op_type=BA --map_ids=${MAP_IDS} --max_repro_err=100 --gps_weight=0.01 --v=0
    ${BACKEND_ADDR} --res_root=${ROOT_ADDR}/global --out_root=${ROOT_ADDR}/global --op_type=BA --map_ids=${MAP_IDS} --max_repro_err=50 --gps_weight=0.01 --v=0
    ${BACKEND_ADDR} --res_root=${ROOT_ADDR}/global --out_root=${ROOT_ADDR}/global --op_type=BA --map_ids=${MAP_IDS} --max_repro_err=10 --gps_weight=0.01 --v=0
    ${BACKEND_ADDR} --res_root=${ROOT_ADDR}/global --out_root=${ROOT_ADDR}/global --op_type=CullingFrame --map_ids=${MAP_IDS} --cull_frame_rate=0.9 --v=0
    ${BACKEND_ADDR} --res_root=${ROOT_ADDR}/global --out_root=${ROOT_ADDR}/global --op_type=BA --map_ids=${MAP_IDS} --max_repro_err=10 --gps_weight=0.01 --v=0
    ${BACKEND_ADDR} --res_root=${ROOT_ADDR}/global --out_root=${ROOT_ADDR}/global --op_type=Reset --map_ids=${MAP_IDS} --reset_type=all --reset_val=false --v=0
done

