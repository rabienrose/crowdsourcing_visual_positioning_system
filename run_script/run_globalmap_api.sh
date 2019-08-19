ROOT_ADDR=/home/chamo/Documents/data/test_iphone_app
MAP_ADDR=${ROOT_ADDR}/global
BAG_ADDR=${ROOT_ADDR}/bag/08-08-09-02-12.bag
CONFIG_ADDR=${ROOT_ADDR}/config
CACHE_ROOT=${ROOT_ADDR}/cache
LOCAL_ROOT=${ROOT_ADDR}/local

EXE_ROOT=/home/chamo/Documents/chamo_vps
GLOBAL_MAP_ADDR=${EXE_ROOT}/devel/lib/global_map_api/global_map_api 

#rm -rf ${CACHE_ROOT}
rm -rf ${LOCAL_ROOT}
mkdir ${CACHE_ROOT}
mkdir ${LOCAL_ROOT}
mkdir ${MAP_ADDR}
${GLOBAL_MAP_ADDR} --map_root=${MAP_ADDR} --bag_addr=${BAG_ADDR} --cache_root=${CACHE_ROOT} --config_root=${CONFIG_ADDR} --localmap_root=${LOCAL_ROOT}
