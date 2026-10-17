# Valid alternative (fix round 3): the reference migration + a unit test that resets ProxyArray's warn-once flag so the "no DeprecationWarning" check is meaningful.
O="$(dirname "$0")/../solution/files"
cp -a "$O/." .
mkdir -p source/isaaclab_tasks/test
cat > source/isaaclab_tasks/test/test_trail_walker_rewards.py <<'PY'
"""Trail Walker reward terms must not use the implicit-tensor deprecation path."""

import warnings
from types import SimpleNamespace

import pytest
import torch
import warp as wp

from isaaclab.utils.warp import ProxyArray

from isaaclab_tasks.contrib.trail_walker import rewards


@pytest.fixture(autouse=True)
def fresh_warning():
    # ProxyArray warns only once per process; re-arm it so this test sees any implicit use.
    ProxyArray._deprecation_warned = False
    yield


def test_no_deprecation_warning():
    jp = torch.randn(4, 12)
    data = SimpleNamespace(joint_pos=ProxyArray(wp.from_torch(jp)), default_joint_pos=ProxyArray(wp.from_torch(jp.clone())))
    env = SimpleNamespace(scene={"robot": SimpleNamespace(data=data)})
    with warnings.catch_warnings():
        warnings.simplefilter("error", DeprecationWarning)
        out = rewards.joint_deviation_l1(env, SimpleNamespace(name="robot", joint_ids=slice(None)))
    torch.testing.assert_close(out, torch.zeros(4))
PY
