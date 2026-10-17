# Starting state: the legs use an ideal (implicit) PD actuator.
python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab.assets import ArticulationCfg, AssetBaseCfg\n", "from isaaclab.actuators import ImplicitActuatorCfg\nfrom isaaclab.assets import ArticulationCfg, AssetBaseCfg\n")
s = s.replace('''    robot: ArticulationCfg = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
''', '''    robot: ArticulationCfg = ANYMAL_C_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        actuators={
            # ideal joint-space PD control for now
            "legs": ImplicitActuatorCfg(
                joint_names_expr=[".*HAA", ".*HFE", ".*KFE"], stiffness=80.0, damping=2.0, effort_limit_sim=80.0
            )
        },
    )
''')
open(p, "w").write(s)
PY
