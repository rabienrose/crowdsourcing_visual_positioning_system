ROOT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/globalmap_api_test
MAP_ADDR=${ROOT_ADDR}/global
BAG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_bag/build_near_10_circle
CONFIG_ADDR=${ROOT_ADDR}/config
CACHE_ROOT=${ROOT_ADDR}/cache
LOCAL_ROOT=${ROOT_ADDR}/local

EXE_ROOT=/home/chamo/Documents/work/chamo_vps
GLOBAL_MAP_ADDR=${EXE_ROOT}/devel/lib/global_map_api/global_map_api 

#rm -rf ${CACHE_ROOT}
#rm -rf ${LOCAL_ROOT}
mkdir ${CACHE_ROOT}
mkdir ${LOCAL_ROOT}
mkdir ${MAP_ADDR}

map_bag_list=`ls ${BAG_ADDR}`
for map_bag_addr in ${map_bag_list}
do
    map_name=`basename "${map_bag_addr}"`
    echo process bag: ${map_name}
    ${GLOBAL_MAP_ADDR} --map_root=${MAP_ADDR} --bag_addr=${BAG_ADDR}/${map_bag_addr} --cache_root=${CACHE_ROOT} --config_root=${CONFIG_ADDR} --localmap_root=${LOCAL_ROOT}
done
