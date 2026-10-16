set -eu
python "$(dirname "$0")/make_checkpoint.py" checkpoints_from_gpu_box/act_aloha_transfer/checkpoints/100000/pretrained_model > /dev/null 2>&1
