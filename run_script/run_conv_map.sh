IN_ADDR=$1
OUT_ADDR=$2

#IN_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test
#OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test/new
EXE_ROOT=/home/chamo/Documents/work/chamo_vps

CONV_MAP_ADDR=${EXE_ROOT}/devel/lib/convert_to_visual_map/convert_to_visual_map

${CONV_MAP_ADDR} --res_root=${IN_ADDR} --global_root=${OUT_ADDR}
