MAP_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/globalmap_api_test/global
BAG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_bag/build_right/07-22-12-19-37.bag
CONFIG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/globalmap_api_test/config
CACHE_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/globalmap_api_test/cache
LOCAL_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/globalmap_api_test/local

EXE_ROOT=/home/chamo/Documents/work/chamo_vps
GLOBAL_MAP_ADDR=${EXE_ROOT}/devel/lib/global_map_api/global_map_api 

rm -rf ${CACHE_ROOT}
rm -rf ${LOCAL_ROOT}
mkdir ${CACHE_ROOT}
mkdir ${LOCAL_ROOT}
mkdir ${MAP_ADDR}
${GLOBAL_MAP_ADDR} --map_root=${MAP_ADDR} --bag_addr=${BAG_ADDR} --cache_root=${CACHE_ROOT} --config_root=${CONFIG_ADDR} --localmap_root=${LOCAL_ROOT}
