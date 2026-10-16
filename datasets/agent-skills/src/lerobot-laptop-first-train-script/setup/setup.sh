set -eu
python .skills_setup/make_so101_dataset.py recordings/so101_stack_blocks mira/so101_stack_blocks 301,288,315,297,276,309,322,290,284,305 --task "Stack the green block on the blue block" > /dev/null 2>&1
rm -rf .skills_setup
