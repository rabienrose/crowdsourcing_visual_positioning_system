IN_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test/out
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test/graph_out
EXE_ROOT=/home/chamo/Documents/work/chamo_vps

BACKEND_ADDR=${EXE_ROOT}/devel/lib/backend/backend

${BACKEND_ADDR} --res_root=${IN_ADDR} --out_root=${OUT_ADDR} --map_ids="112260160" --project_mat_file=${IN_ADDR}/words_projmat.fstream --gps_weight=0.00000001 --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
