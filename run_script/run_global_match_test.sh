MAP_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/try_coarse_gps/global
BAG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_bag/build_near_10_circle/07-25-15-51-37.bag
CONFIG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/try_coarse_gps/config
RES_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/try_coarse_gps/output

EXE_ROOT=/home/chamo/Documents/work/chamo_vps
GLOBAL_MATCH_ADDR=${EXE_ROOT}/devel/lib/global_match_test/global_match_test

${GLOBAL_MATCH_ADDR} --map_name=${MAP_ADDR} --bag_addr=${BAG_ADDR} --res_root=${RES_ROOT} --config_root=${CONFIG_ADDR}
