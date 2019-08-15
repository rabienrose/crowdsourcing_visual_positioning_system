MAP_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all/test_opti
BAG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/build_right/07-22-11-46-43.bag
CONFIG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_720_configs
RES_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all

EXE_ROOT=/home/chamo/Documents/work/chamo_vps
GLOBAL_MATCH_ADDR=${EXE_ROOT}/devel/lib/global_match_test/global_match_test

${GLOBAL_MATCH_ADDR} --map_name=${MAP_ADDR} --bag_addr=${BAG_ADDR} --res_root=${RES_ROOT} --config_root=${CONFIG_ADDR}
