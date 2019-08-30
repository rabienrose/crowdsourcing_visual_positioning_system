IN_ADDR=$1
OUT_ADDR=$2
OP_TYPE=$3
MAP_IDS=$4

#IN_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all/global
#OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all/global
#OP_TYPE=BA
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
    --reset_type=all \
    --reset_val=false \
    --cull_frame_rate=0.9 \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
