set -eu
python .skills_setup/make_so101_dataset.py recordings/so101_pick_cube mira/so101_pick_cube 212,187,240,199,226,205,178 > /dev/null 2>&1
rm -rf .skills_setup
