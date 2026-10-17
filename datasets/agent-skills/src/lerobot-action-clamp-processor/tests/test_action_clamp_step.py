"""lerobot-action-clamp-processor: the step through ProcessorStepRegistry and a PolicyProcessorPipeline
save_pretrained / from_pretrained round trip across fresh processes."""
import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

from lr_helpers import W

HEADER = """
import torch
from lerobot.processor import PolicyProcessorPipeline
from lerobot.processor.converters import policy_action_to_transition, transition_to_policy_action
KW = dict(to_transition=policy_action_to_transition, to_output=transition_to_policy_action)
"""


def _run(code):
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    r = subprocess.run([sys.executable, "-c", HEADER + textwrap.dedent(code)], cwd="/", env=env,
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-4000:]
    return r.stdout


@pytest.fixture(scope="module")
def saved(tmp_path_factory):
    d = tmp_path_factory.mktemp("clamp_pipeline")
    _run(f"""
        from lerobot.processor import ActionClampProcessorStep
        step = ActionClampProcessorStep(min_action=torch.tensor([-1.0, -2.0, -3.0]),
                                        max_action=torch.tensor([1.0, 2.0, 3.0]), margin=0.25)
        pipe = PolicyProcessorPipeline(steps=[step], name="clamp_post", **KW)
        out = pipe(torch.tensor([[5.0, -5.0, 0.5], [-0.5, 2.1, -9.0]]))
        assert torch.allclose(out, torch.tensor([[1.25, -2.25, 0.5], [-0.5, 2.1, -3.25]])), out
        pipe.save_pretrained({str(d)!r})
    """)
    return d


def _config(d):
    files = sorted(Path(d).glob("*.json"))
    assert len(files) == 1, files
    return files[0], json.loads(files[0].read_text())


def test_registered_and_exported():
    out = _run("""
        from lerobot.processor import ActionClampProcessorStep, ProcessorStepRegistry
        cls = ProcessorStepRegistry.get("action_clamp_processor")
        assert cls is ActionClampProcessorStep
        print(cls.__module__)
    """)
    assert out.strip().splitlines()[-1] == "lerobot.processor.action_clamp_processor"


def test_saved_by_registry_name_with_bounds_as_state(saved):
    from safetensors.torch import load_file
    import torch

    _, cfg = _config(saved)
    steps = cfg["steps"]
    assert len(steps) == 1
    entry = steps[0]
    assert entry.get("registry_name") == "action_clamp_processor", entry
    conf = entry.get("config", {})
    assert conf.get("margin") == 0.25
    assert "min_action" not in conf and "max_action" not in conf, conf
    assert entry.get("state_file"), entry
    state = load_file(str(saved / entry["state_file"]))
    vals = [t.float() for t in state.values()]
    assert any(t.shape == (3,) and torch.allclose(t, torch.tensor([-1.0, -2.0, -3.0])) for t in vals), state
    assert any(t.shape == (3,) and torch.allclose(t, torch.tensor([1.0, 2.0, 3.0])) for t in vals), state


def test_round_trip_in_fresh_process(saved):
    name, _ = _config(saved)
    _run(f"""
        pipe = PolicyProcessorPipeline.from_pretrained({str(saved)!r}, config_filename={name.name!r}, **KW)
        a = torch.tensor([[5.0, -5.0, 0.5], [-0.5, 2.1, -9.0]])
        assert torch.allclose(pipe(a), torch.tensor([[1.25, -2.25, 0.5], [-0.5, 2.1, -3.25]]))
        one = pipe(torch.tensor([9.0, 9.0, 9.0], dtype=torch.float64))
        assert one.shape == (3,) and one.dtype == torch.float64
        assert torch.allclose(one, torch.tensor([1.25, 2.25, 3.25], dtype=torch.float64))
        tight = PolicyProcessorPipeline.from_pretrained({str(saved)!r}, config_filename={name.name!r},
                                                        overrides={{"action_clamp_processor": {{"margin": 0.0}}}}, **KW)
        assert torch.allclose(tight(a), torch.tensor([[1.0, -2.0, 0.5], [-0.5, 2.0, -3.0]]))
        print("OK")
    """)


def test_features_unchanged():
    _run("""
        from lerobot.configs import FeatureType, PipelineFeatureType, PolicyFeature
        from lerobot.processor import ActionClampProcessorStep
        step = ActionClampProcessorStep(min_action=torch.zeros(2), max_action=torch.ones(2))
        feats = {PipelineFeatureType.ACTION: {"action": PolicyFeature(type=FeatureType.ACTION, shape=(2,))},
                 PipelineFeatureType.OBSERVATION: {"observation.state": PolicyFeature(type=FeatureType.STATE, shape=(4,))}}
        import copy
        out = PolicyProcessorPipeline(steps=[step], **KW).transform_features(copy.deepcopy(feats))
        assert out == feats, out
    """)
