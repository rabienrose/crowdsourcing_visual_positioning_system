MAP_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/office_merge/global
BAG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/office_merge_1/bag/08-26-13-28-16.bag
CONFIG_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/office_merge_1/config
RES_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/office_merge_1/output

EXE_ROOT=/home/chamo/Documents/work/chamo_vps
GLOBAL_MATCH_ADDR=${EXE_ROOT}/devel/lib/global_match_test/global_match_test

${GLOBAL_MATCH_ADDR} --map_name=${MAP_ADDR} --bag_addr=${BAG_ADDR} --res_root=${RES_ROOT} --config_root=${CONFIG_ADDR}
