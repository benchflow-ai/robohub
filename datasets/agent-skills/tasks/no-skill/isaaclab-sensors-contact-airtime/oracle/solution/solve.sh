python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab.scene import InteractiveSceneCfg\n", "from isaaclab.scene import InteractiveSceneCfg\nfrom isaaclab.sensors import ContactSensorCfg\n")
s = s.replace('''    robot: ArticulationCfg = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
''', '''    robot: ArticulationCfg = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/[^/]*", history_length=3, track_air_time=True)
''')
s = s.replace('''    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)
''', '''    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=0.5,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*FOOT"),
            "command_name": "base_velocity",
            "threshold": 0.5,
        },
    )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*THIGH"), "threshold": 1.0},
    )
''')
s = s.replace('''    bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 1.0})
''', '''    bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 1.0})
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )
''')
s = s.replace('''        self.sim.physics_material = self.scene.terrain.physics_material
''', '''        self.sim.physics_material = self.scene.terrain.physics_material
        # update the contact sensor every physics step
        self.scene.contact_forces.update_period = self.sim.dt
''')
open(p, "w").write(s)
PY
