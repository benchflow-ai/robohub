# Starting state: the 2.3 project with WXYZ quaternions hard-coded in the env config.
python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab_assets.robots.anymal import ANYMAL_C_CFG\n", "from isaaclab_assets.robots.anymal import ANYMAL_C_CFG\n\nfrom .heading import GOAL_MARKER_RGBA, heading_error\n")
s = s.replace('''    robot: ArticulationCfg = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
''', '''    robot: ArticulationCfg = ANYMAL_C_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        # spawn upright with no rotation
        init_state=ANYMAL_C_CFG.init_state.replace(pos=(0.0, 0.0, 0.6), rot=(1.0, 0.0, 0.0, 0.0)),
    )
    # goal marker: a red cone 2 m ahead, turned 90 degrees about z
    goal_marker = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/GoalMarker",
        spawn=sim_utils.ConeCfg(
            radius=0.1,
            height=0.3,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=GOAL_MARKER_RGBA[:3], opacity=GOAL_MARKER_RGBA[3]),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(2.0, 0.0, 0.15), rot=(0.7071068, 0.0, 0.0, 0.7071068)),
    )
''')
s = s.replace('''        actions = ObsTerm(func=mdp.last_action)
''', '''        actions = ObsTerm(func=mdp.last_action)
        heading_error = ObsTerm(func=heading_error)
''')
open(p, "w").write(s)
PY
