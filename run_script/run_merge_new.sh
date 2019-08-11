BASE_MAP=$1
NEW_MAP=$2
OUT_MAP=$3

#BASE_MAP=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all/global
#NEW_MAP=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all/07-22-11-50-51.bag/local
#OUT_MAP=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all/temp
EXE_ROOT=/home/chamo/Documents/work/chamo_vps

MERGE_ADDR=${EXE_ROOT}/devel/lib/merge_new/merge_new

${MERGE_ADDR} --base_map_root=${BASE_MAP} --new_map_root=${NEW_MAP} --out_map_root=${OUT_MAP} --map_ids=$(python ./run_script/get_mapids.py ${NEW_MAP})
#${MERGE_ADDR} --base_map_root=${BASE_MAP} --new_map_root=${NEW_MAP} --out_map_root=${OUT_MAP} --map_ids="112224160,112224161,112260160,112260161"
