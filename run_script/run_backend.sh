IN_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/globalmap_api_test/matched
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/globalmap_api_test/opt
MAP_IDS=112224160
OP_TYPE=BA
EXE_ROOT=/home/chamo/Documents/work/chamo_vps

BACKEND_ADDR=${EXE_ROOT}/devel/lib/backend/backend

${BACKEND_ADDR} \
    --res_root=${IN_ADDR} \
    --out_root=${OUT_ADDR} \
    --map_ids=${MAP_IDS}\
    --project_mat_file=${IN_ADDR}/../config/words_projmat.fstream \
    --op_type=${OP_TYPE} \
    --max_repro_err=100 \
    --gps_weight=0.000001 \
    --reset_type=doMatch \
    --reset_val=true \
    --cull_frame_rate=0.8 \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
