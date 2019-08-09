IN_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test/opti
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/test/opti
EXE_ROOT=/home/chamo/Documents/work/chamo_vps

BACKEND_ADDR=${EXE_ROOT}/devel/lib/backend/backend

${BACKEND_ADDR} \
    --res_root=${IN_ADDR} \
    --out_root=${OUT_ADDR} \
    --map_ids="112260160" \
    --project_mat_file=${IN_ADDR}/words_projmat.fstream \
    --op_type="BA" \
    --param1="" \
    --max_repro_err=5 \
    --gps_weight=1 \
    --reset_type=all \
    --reset_val=false \
    --cull_frame_rate=0.8 \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
