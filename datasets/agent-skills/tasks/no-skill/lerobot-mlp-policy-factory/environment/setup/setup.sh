set -eu
python .skills_setup/make_so101_dataset.py recordings/so101_reach_state mira/so101_reach_state 100,120,90,110,95,105,100,115 --no-camera --task "Reach the marker" > /dev/null 2>&1
rm -rf .skills_setup
