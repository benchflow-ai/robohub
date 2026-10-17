python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab.scene import InteractiveSceneCfg\n", "from isaaclab.scene import InteractiveSceneCfg\nfrom isaaclab.sensors import RayCasterCfg, patterns\n")
s = s.replace("from isaaclab.terrains import TerrainImporterCfg\n", "from isaaclab.terrains import TerrainImporterCfg\nfrom isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG\n")
s = s.replace('''        terrain_type="plane",
''', '''        terrain_type="generator",
        terrain_generator=ROUGH_TERRAINS_CFG,
        max_init_terrain_level=5,
''')
s = s.replace('''    robot: ArticulationCfg = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
''', '''    robot: ArticulationCfg = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )
''')
s = s.replace('''        actions = ObsTerm(func=mdp.last_action)
''', '''        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
        )
''')
s = s.replace('''        self.sim.physics_material = self.scene.terrain.physics_material
''', '''        self.sim.physics_material = self.scene.terrain.physics_material
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt
''')
open(p, "w").write(s)
PY
