"""Hidden verifier: every ANYmal-C leg joint is driven by the ANYdrive 3.0 LSTM actuator network."""

import re

from ss_isaac import resolve

TASK = "IsaacContrib-TrailWalker-Flat"
JOINTS = [f"{leg}_{j}" for leg in ("LF", "LH", "RF", "RH") for j in ("HAA", "HFE", "KFE")]


def actuators():
    e, _ = resolve(TASK, [])
    return e.scene.robot.actuators


def covered(a):
    return {j for j in JOINTS if any(re.fullmatch(x, j) for x in a.joint_names_expr)}


def test_all_learned_lstm():
    from isaaclab.actuators import ActuatorNetLSTMCfg

    acts = actuators()
    assert acts, "no actuators"
    for name, a in acts.items():
        assert isinstance(a, ActuatorNetLSTMCfg), f"{name}: {type(a).__name__}"
        assert a.network_file.endswith("anydrive_3_lstm_jit.pt"), a.network_file


def test_every_leg_joint_once():
    seen = []
    for a in actuators().values():
        seen += sorted(covered(a))
    assert sorted(seen) == sorted(JOINTS)


def test_matches_isaaclab_anydrive_model():
    from isaaclab_assets.robots.anymal import ANYDRIVE_3_LSTM_ACTUATOR_CFG as ref

    for a in actuators().values():
        assert a.saturation_effort == ref.saturation_effort
        assert a.actuator_effort_limit == ref.actuator_effort_limit
        assert a.actuator_velocity_limit == ref.actuator_velocity_limit
