ROOT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/globalmap_api_test
MAP_ADDR=${ROOT_ADDR}/global
RELEASE_ADDR=${ROOT_ADDR}/release
REJECT_ADDR=${ROOT_ADDR}/reject
BAG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_bag/office_9/09-05-15-02-10
CONFIG_ADDR=${ROOT_ADDR}/config
CACHE_ROOT=${ROOT_ADDR}/cache
LOCAL_ROOT=${ROOT_ADDR}/local

EXE_ROOT=/home/chamo/Documents/work/chamo_vps
GLOBAL_MAP_ADDR=${EXE_ROOT}/devel/lib/global_map_api/global_map_api 

mkdir ${MAP_ADDR}
mkdir ${RELEASE_ADDR}
mkdir ${REJECT_ADDR}

map_bag_list=`ls ${BAG_ADDR}`
for map_bag_addr in ${map_bag_list}
do
    rm -rf ${CACHE_ROOT}
    rm -rf ${LOCAL_ROOT}
    mkdir ${CACHE_ROOT}
    mkdir ${LOCAL_ROOT}
    map_name=`basename "${map_bag_addr}"`
    echo process bag: ${map_name}
    SIZE_1=$(du -B 1 ${MAP_ADDR} | cut -f 1)
    rm -rf ${MAP_ADDR}
    mkdir ${MAP_ADDR}
    cp ${RELEASE_ADDR}/* ${MAP_ADDR}
    echo 'before: '$SIZE_1
    ${GLOBAL_MAP_ADDR} --map_root=${MAP_ADDR} --bag_addr=${BAG_ADDR}/${map_bag_addr} --cache_root=${CACHE_ROOT} --config_root=${CONFIG_ADDR} --localmap_root=${LOCAL_ROOT} --v=0
    SIZE_2=$(du -B 1 ${MAP_ADDR} | cut -f 1)    
    echo 'after: '$SIZE_2
    if [[ $SIZE_2 -gt $SIZE_1 ]]; then
        echo 'pass'
        rm -rf ${RELEASE_ADDR}
        mkdir ${RELEASE_ADDR}
        cp ${MAP_ADDR}/* ${RELEASE_ADDR}
    else
        echo 'failed'
        cp ${BAG_ADDR}/${map_bag_addr} ${REJECT_ADDR}
    fi
done
