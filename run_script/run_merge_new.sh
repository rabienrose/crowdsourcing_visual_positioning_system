BASE_MAP=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test/global
NEW_MAP=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test/new
OUT_MAP=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test/out
EXE_ROOT=/home/chamo/Documents/work/chamo_vps

MERGE_ADDR=${EXE_ROOT}/devel/lib/merge_new/merge_new

${MERGE_ADDR} --base_map_root=${BASE_MAP} --new_map_root=${NEW_MAP} --out_map_root=${OUT_MAP} --map_ids="112260160"
