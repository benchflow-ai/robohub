python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab_assets.robots.anymal import ANYMAL_C_CFG\n", "from isaaclab_assets.robots.anymal import ANYMAL_C_CFG\n\nfrom . import rewards as tw_rewards\n")
s = s.replace('''    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)
''', '''    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)
    hip_deviation = RewTerm(
        func=tw_rewards.joint_deviation_l1, weight=-0.1, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*HAA")}
    )
    base_height = RewTerm(func=tw_rewards.base_height_l2, weight=-1.0, params={"target_height": 0.55})
    joint_vel = RewTerm(func=tw_rewards.joint_vel_l2_clipped, weight=-1.0e-4, params={"max_vel": 10.0})
    stand_still = RewTerm(func=tw_rewards.stand_still_deviation, weight=-0.5, params={"command_name": "base_velocity"})
''')
open(p, "w").write(s)
PY
