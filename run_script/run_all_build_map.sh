ROOT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all
CONF_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_720_configs
BAG_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/build_right
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
    #bash ./run_script/build_iphone_out.sh ${ROOT_ADDR}/${map_name} ${BAG_ROOT}/${map_bag_addr}
    bash ./run_script/run_merge_new.sh ${ROOT_ADDR}/global ${ROOT_ADDR}/${map_name}/local ${ROOT_ADDR}/global
    MAP_IDS=$(python ./run_script/get_mapids.py ${ROOT_ADDR}/${map_name}/local)
    bash ./run_script/run_backend.sh ${ROOT_ADDR}/global ${ROOT_ADDR}/global Match ${MAP_IDS}
    bash ./run_script/run_backend.sh ${ROOT_ADDR}/global ${ROOT_ADDR}/global BA ${MAP_IDS}
    bash ./run_script/run_backend.sh ${ROOT_ADDR}/global ${ROOT_ADDR}/global Match ${MAP_IDS}
    bash ./run_script/run_backend.sh ${ROOT_ADDR}/global ${ROOT_ADDR}/global BA ${MAP_IDS}
    bash ./run_script/run_backend.sh ${ROOT_ADDR}/global ${ROOT_ADDR}/global CullingFrame ${MAP_IDS}
    bash ./run_script/run_backend.sh ${ROOT_ADDR}/global ${ROOT_ADDR}/global BA ${MAP_IDS}
    bash ./run_script/run_backend.sh ${ROOT_ADDR}/global ${ROOT_ADDR}/global Reset ${MAP_IDS}
done

