"""`robouse estop`, `operator`, `hil-check` and `robots`: the human side of real-robot runs. Not listed in `robouse
--help` until the live-hardware path has run through Robo Use; the commands work.
"""

from __future__ import annotations

import argparse
import json


def register(sub: argparse._SubParsersAction) -> None:
    e = sub.add_parser("estop")
    e.add_argument("reason", nargs="?", default="operator e-stop", help="why (default: operator e-stop)")
    e.add_argument("--clear", action="store_true", help="release the e-stop")
    e.set_defaults(func=estop)

    o = sub.add_parser("operator")
    o.add_argument("action", choices=["status", "answer", "say"], help="status, answer or say")
    o.add_argument("value", nargs="?", default="", help="answer: yes, no, partial or skip; say: the remark")
    o.add_argument("--note", default="", help="a note recorded with the answer")
    o.set_defaults(func=operator)

    hc = sub.add_parser("hil-check")
    hc.add_argument("fixtures", nargs="*", help="hardware-in-the-loop fixture names (default: all)")
    hc.set_defaults(func=hil_check)

    r = sub.add_parser("robots")
    r.set_defaults(func=robots)


def estop(a: argparse.Namespace) -> int:
    from ..real import safety

    if a.clear:
        print("e-stop cleared" if safety.clear_estop() else "e-stop was not engaged")
    else:
        print(f"e-stop latched: {safety.set_estop(a.reason)}")
    return 0


def operator(a: argparse.Namespace) -> int:
    from ..real import operator as op
    from ..real import safety

    if a.action == "status":
        p = safety.OPERATOR_DIR / "pending.json"
        print(p.read_text() if p.exists() else "no pending operator request")
        print("e-stop:", safety.estop_engaged() or "clear")
    elif a.action == "answer":
        print(op.answer(a.value, a.note))
    else:
        print(f"remark queued for the policy: {op.say(a.value)}")
    return 0


def hil_check(a: argparse.Namespace) -> int:
    from ..real.replay import HIL_DIR, Fixture, conformance
    from ..real.robots import profile

    names = a.fixtures or sorted(p.name for p in HIL_DIR.iterdir() if (p / "fixture.json").exists())
    for n in names:
        fx = Fixture(n)
        if len(fx.steps) < 2:
            print(
                json.dumps({"fixture": n, "robot": fx.robot, "steps": len(fx.steps), "note": fx.meta.get("note", "")})
            )
            continue
        print(json.dumps(conformance(fx, profile(fx.robot), fx.meta.get("session_envelope"))))
    return 0


def robots(a: argparse.Namespace) -> int:
    from ..real.replay import HIL_DIR
    from ..real.robots import PROFILES

    seen: dict[int, str] = {}
    for name, p in PROFILES.items():
        if id(p) in seen:
            print(f"{name:14} the {seen[id(p)]} profile")
            continue
        seen[id(p)] = name
        kin = "kinematics" if p.chain_file else "joint limits only"
        checked = (
            "profile checked against hardware sessions" if p.tested_on_hardware else "profile not checked on hardware"
        )
        print(f"{name:14} {p.display}: {len(p.joints)} joints, {p.control_hz:g} Hz, {kin}; {checked}")
    print(
        "live hardware through Robo Use: not run yet; real tasks run on the hardware-in-the-loop mock unless --rig is given"
    )
    print(
        "drivers: mock (hardware-in-the-loop replay), lerobot (Metal, SO-100/SO-101), piper (piper_sdk), "
        "rosbridge (ROS arms), plus robouse.drivers entry points"
    )
    for f in sorted(HIL_DIR.glob("*/fixture.json")):
        d = json.loads(f.read_text())
        print(
            f"fixture {f.parent.name}: {d['robot']}, {len(d.get('steps', []))} recorded steps, "
            f"{len(d.get('keyframes', []))} keyframes, cameras {d.get('cameras')}, source {d.get('source')}"
        )
    return 0
