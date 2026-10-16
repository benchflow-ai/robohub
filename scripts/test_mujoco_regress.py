"""Tests of scripts/mujoco_regress.py that need no sandbox: the plan over this checkout, the Dockerfile reader, the
runtime variant, divergence and classification, and diff + report over a small synthetic job.

    python3 -m unittest scripts/test_mujoco_regress.py
"""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mujoco_regress as M  # noqa: E402

HUB = Path(__file__).resolve().parents[1]


def jl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))


class Plan(unittest.TestCase):
    def test_plan_lists_the_mujoco_runtimes_and_their_pins(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "plan.json"
            M.main(["--hub", str(HUB), "plan", "--mujoco", "3.16.0", "--out", str(out), "--no-releases", "--shards", "base=3"])
            p = json.loads(out.read_text())
        rts = p["runtimes"]
        self.assertEqual(rts["dmcontrol"]["sim_mujoco"], "3.14.0")
        self.assertEqual(rts["humanoidbench"]["sim_mujoco"], "3.1.6")
        self.assertEqual(rts["base"]["sim_mujoco"], "3.3.0")
        self.assertEqual(rts["dmcontrol"]["pythons"], {"ROBOUSE_DMCONTROL_PYTHON": "/opt/dmcontrol-venv/bin/python"})
        # seven pinned versions across the MuJoCo runtimes; light, driving and maniskill pin MuJoCo but do not use it
        self.assertEqual(sorted({r["sim_mujoco"] for r in rts.values()}, key=M.vkey),
                         ["3.1.6", "3.3.0", "3.3.1", "3.4.0", "3.5.0", "3.6.0", "3.14.0"])
        for rt in ("light", "driving", "maniskill", "gpu", "behavior", "robodojo"):
            self.assertIn(rt, p["skipped_runtimes"])
        # a task shared unchanged by several datasets runs once; the no-op control's reference is the no-op
        core = [t for t in p["tasks"].values() if "benchflow/robouse-core@0.4" in t["datasets"]]
        self.assertTrue(core)
        noop = [t for t in p["tasks"].values() if t["datasets"] == ["benchflow/robouse-noop-control@0.4"]]
        self.assertTrue(noop and all(t["harnesses"] == ["reference"] and t["expect"] == {"reference": 0} for t in noop))
        self.assertEqual(sum(len(u["tasks"]) for u in p["units"].values()), len(p["tasks"]))
        self.assertEqual(len([u for u in p["units"] if u.startswith("base.")]), 3)


class Dockerfile(unittest.TestCase):
    def test_instructions_join_continuations_and_drop_comments(self) -> None:
        text = "FROM x\n# a comment\nRUN a \\\n# inside\n && b\nENV A=1 B=\"two words\"\nARG C=3\n"
        self.assertEqual(M.instructions(text), [("FROM", "x"), ("RUN", "a  && b"), ("ENV", 'A=1 B="two words"'), ("ARG", "C=3")])
        self.assertEqual(M.env_pairs('A=1 B="two words"'), {"A": "1", "B": "two words"})
        self.assertEqual(M.expand("$A/${B}/$C", {"A": "x", "B": "y"}), "x/y/$C")

    def test_content_digest_matches_the_pinned_runtime(self) -> None:
        compose = next((HUB / "datasets" / "robouse-dmcontrol").glob("*/environment/docker-compose.yaml")).read_text()
        self.assertIn(M.content_digest(HUB / "runtimes" / "dmcontrol"), compose)

    def test_variant_switches_every_interpreter(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "v"
            M.main(["--hub", str(HUB), "variant", "dmcontrol", "--mujoco", "3.16.0", "--out", str(out)])
            df = (out / "Dockerfile").read_text()
            meta = json.loads((out / "MUJOCO_VARIANT.json").read_text())
        self.assertLess(df.index("mujoco_regress.py variant"), df.index("ARG ROBOHUB_RUNTIME_DIGEST"))
        for py in ("/usr/local/bin/python", "/opt/dmcontrol-venv/bin/python"):
            self.assertIn(f'--python {py} "mujoco==3.16.0"', df)
        self.assertEqual(meta["pinned"], {"requirements-dmcontrol.txt": "3.14.0", "requirements.txt": "3.3.0"})


class Evidence(unittest.TestCase):
    def test_first_divergence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp) / "a", Path(tmp) / "b"
            jl(a / "episode" / "physics.jsonl", [{"i": i, "ee": [0.0, 0.0, 0.001 * i]} for i in range(10)])
            jl(b / "episode" / "physics.jsonl", [{"i": i, "ee": [0.0, 0.0, 0.001 * i + (0.00002 if i >= 4 else 0) + (0.002 if i >= 7 else 0)]} for i in range(10)])
            jl(a / "episode" / "trace.jsonl", [{"t": 1.0, "steps": 0, "req": {"op": "observe"}, "resp": {"ok": True, "x": 1.0}}])
            jl(b / "episode" / "trace.jsonl", [{"t": 9.0, "steps": 0, "req": {"op": "observe"}, "resp": {"ok": True, "x": 1.0}}])
            d = M.first_divergence(a, b)
        self.assertEqual((d["physics_step"], d["physics_step_1mm"]), (4, 7))
        self.assertNotIn("trace_request", d)  # times differ, nothing else

    def test_error_signature_finds_the_suite_package(self) -> None:
        err = ('Traceback (most recent call last):\n  File "/usr/local/lib/python3.12/site-packages/robosuite/utils/binding_utils.py", line 521, in f\n'
               "    assert joint_type in (a, b)\nAssertionError\n")
        self.assertEqual(M.error_sig(err)["package"], "robosuite")
        row = {"new_mujoco": "3.16.0", "pin_conflict": [], "reference": {"changed": True, "pinned": 1, "new": "error", "new_error_sig": M.error_sig(err)}}
        self.assertEqual(M.classify(row)[0], "our pin")
        own = M.error_sig('  File "/opt/robohub-runtime/robouse/src/robouse/backends/x.py", line 3, in g\nAttributeError: no attribute\n')
        row["reference"]["new_error_sig"] = own
        self.assertEqual(M.classify(row)[0], "simulator change")
        flip = {"reference": {"changed": True, "pinned": 1, "new": 0, "repeat": 0}}
        self.assertEqual(M.classify(flip)[0], "task fragility")
        steady = {"reference": {"changed": True, "pinned": 1, "new": 0, "repeat": 1}, "seeds": {"reference": {"n": 2, "pinned_pass": 2, "new_pass": 0}}}
        self.assertEqual(M.classify(steady)[0], "simulator change")

    def test_verdict(self) -> None:
        self.assertEqual([M.verdict(None), M.verdict({"reward": None}), M.verdict({"reward": 1.0}), M.verdict({"reward": 0.0})],
                         [None, "error", 1, 0])


class DiffAndReport(unittest.TestCase):
    def test_diff_and_report_over_a_synthetic_job(self) -> None:
        plan = {"mujoco": "3.16.0", "hub_commit": "abc", "created_utc": "now", "releases": [], "skipped_runtimes": {},
                "runtimes": {"rt": {"pins": {"requirements.txt": "3.3.0"}, "sim_mujoco": "3.3.0", "tasks": ["k1", "k2"]}},
                "tasks": {"k1": {"id": "t1", "runtime": "rt", "suite": "s", "backend": "b", "datasets": ["o/d@1"], "harnesses": ["reference", "noop"], "expect": {"reference": 1, "noop": 0}},
                          "k2": {"id": "t2", "runtime": "rt", "suite": "s", "backend": "b", "datasets": ["o/d@1"], "harnesses": ["reference", "noop"], "expect": {"reference": 1, "noop": 0}}},
                "units": {"rt.0": {"runtime": "rt", "tasks": ["k1", "k2"]}}}
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "plan.json").write_text(json.dumps(plan))
            job = root / "rt.0"
            job.mkdir()
            (job / "meta.json").write_text(json.dumps({"unit": "rt.0", "runtime": "rt", "pinned": {"/py": "3.3.0"}, "simulator_python": "/py", "resolver": {}}))
            ok = lambda k, h, r: {"key": k, "harness": h, "seed": None, "reward": r, "outcome": "done", "steps_used": 5}  # noqa: E731
            jl(job / "pinned.jsonl", [ok("k1", "reference", 1.0), ok("k1", "noop", 0.0), ok("k2", "reference", 1.0), ok("k2", "noop", 0.0)])
            jl(job / "new.jsonl", [ok("k1", "reference", 1.0), ok("k1", "noop", 0.0), ok("k2", "reference", 0.0), ok("k2", "noop", 0.0)])
            jl(job / "pinned-repeat.jsonl", [ok("k2", "reference", 1.0)])
            (job / "divergence.json").write_text(json.dumps({"k2|reference": {"new": {"physics_step": 12}}}))
            (job / "bisect.json").write_text(json.dumps({"k2": {"first_changed": "3.12.0", "last_unchanged": "3.11.0", "checked": {}, "bisected": True}}))
            jl(job / "progress.jsonl", [{"phase": "pinned", "utc": "a"}, {"phase": "done", "utc": "b"}])
            M.diff(argparse.Namespace(plan=str(root / "plan.json"), results=[str(job)], out=str(root / "diff.json")))
            d = json.loads((root / "diff.json").read_text())
            M.report(argparse.Namespace(diff=str(root / "diff.json"), out=str(root / "R.md")))
            md = (root / "R.md").read_text()
        self.assertEqual(d["summary"]["changed"], 1)
        t2 = next(r for r in d["tasks"] if r["task"] == "t2")
        self.assertEqual((t2["reference"]["pinned"], t2["reference"]["new"], t2["class"]), (1, 0, "simulator change"))
        self.assertIn("| rt | 3.3.0 | 2 | 2 / 1 of 2 | 2 / 2 of 2 | 1 | 3.12.0 (1) |", md)
        self.assertIn("| `t2` | o/d@1 | 3.3.0 | 3.16.0 | 1 → 0 | 0 → 0 | 12 | simulator change |", md)


if __name__ == "__main__":
    unittest.main()
