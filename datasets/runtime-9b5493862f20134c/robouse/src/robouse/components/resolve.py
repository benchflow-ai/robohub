"""Compatibility: does an embodiment fit a scene and a task (with these modifiers)? And if not, why not.

Checks, in order, from the manifests alone (no simulator):
  1. placement   the scene offers the placement the embodiment needs (arm mount, bimanual mount, floor, airspace)
  2. task        one of the task's variants accepts this placement and the embodiment has all its capabilities
  3. scene needs the scene stocks what the task needs (containers, object kinds, surfaces, tags, floor space), and
                 objects this embodiment can grasp (front grasps need objects at least 7 cm tall)
  4. modifiers   the perturbation and safety overlay apply to this task
`generate` then instantiates the task (object placement, reachability) and reports a failure there as a reason too.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .registry import Registry

MOD_DEFAULTS = {"obs": "state", "budget": "normal", "perturbation": "none", "safety": "none", "roles": "single"}


@dataclass
class Report:
    ok: bool
    reasons: list[str] = field(default_factory=list)
    variant: dict | None = None

    def as_dict(self) -> dict:
        return {"ok": self.ok, "reasons": self.reasons, **({"variant": self.variant} if self.variant else {})}


def modifiers(mods: dict | None) -> dict:
    out = dict(MOD_DEFAULTS)
    for k, v in (mods or {}).items():
        if v is not None:
            out[k] = v
    return out


def check(reg: Registry, emb: str, scene: str, task: str, mods: dict | None = None) -> Report:
    e, s, t = reg.get("embodiments", emb), reg.get("scenes", scene), reg.get("tasks", task)
    mods = modifiers(mods)
    reasons: list[str] = []
    for kind, m in (("embodiment", e), ("scene", s), ("task", t)):
        if not m.get("remixable"):
            reasons.append(f"{kind} {m['id']} is a fixed component (it describes an existing suite) and cannot be remixed")
    if reasons:
        return Report(False, reasons)
    # 1. placement
    if e["placement"] not in s["placements"]:
        need = {"arm": "a surface mount for one arm", "bimanual": "a surface mount for two arms", "floor": "floor space",
                "airspace": "airspace and a take-off pad"}[e["placement"]]
        reasons.append(f"{e['title']} needs {need}; {s['title']} offers {', '.join(s['placements'])}")
        return Report(False, reasons)
    # 2. task variant
    caps = set(e["capabilities"])
    variant = None
    missing = []
    for v in t["requires"]:
        if e["placement"] not in v["placements"]:
            missing.append(f"{'an' if v['placements'][0][0] in 'ai' else 'a'} {'/'.join(v['placements'])} placement")
            continue
        lack = [c for c in v["capabilities"] if c not in caps]
        if lack:
            missing.append(f"capabilities {', '.join(lack)}")
            continue
        variant = v
        break
    if variant is None:
        reasons.append(f"{t['title'].lower()} needs {' or '.join(dict.fromkeys(missing))}; {e['title']} is placed on "
                       f"{e['placement']} and has {', '.join(e['capabilities'])}")
        return Report(False, reasons)
    # 3. scene needs
    needs = t.get("needs", {})
    kinds = s.get("object_kinds", {})
    conts = s.get("containers", {})
    if needs.get("containers"):
        on_surface = [c for c in conts if c != "bin"]
        if not on_surface:
            reasons.append(f"{s['title']} has no container that stands on a work surface")
        elif e.get("grasp") == "front" and not [c for c in on_surface if conts[c]["wall_height_m"] <= .05]:
            walls = ", ".join(f"{c} ({conts[c]['wall_height_m'] * 100:g} cm)" for c in on_surface)
            reasons.append(f"{e['title']} grasps from the front with a level gripper and can only drop things into containers with "
                           f"walls up to 5 cm; {s['title']} has {walls}")
    if needs.get("object_kinds_any") and not set(needs["object_kinds_any"]) & set(kinds):
        want = [x for x in needs["object_kinds_any"]]
        if variant.get("object"):
            want = [variant["object"]]
        reasons.append(f"{s['title']} stocks no {' or '.join(want)}")
    if variant.get("object") and variant["object"] not in kinds:
        reasons.append(f"{s['title']} stocks no {variant['object']} (this variant pushes one)")
    if needs.get("tags") and s.get("tag_spots", 0) < needs["tags"]:
        reasons.append(f"{s['title']} has no place for an inspection tag")
    if needs.get("floor_slots") and s.get("floor_slots", 0) < needs["floor_slots"]:
        reasons.append(f"{s['title']} has no free floor area")
    if "grasp" in variant["capabilities"]:
        from ..backends.remix_sim.templates import grasp_fits

        g = e.get("grasp") or "top"
        from ..backends.remix_sim.templates import CONTAINER_MAX_H

        graspable = [k for k, v in kinds.items() if v["pick_and_place"]
                     and grasp_fits(v["grip_width_m"], v["height_m"], g, e.get("max_grip_m", 0))
                     and not (needs.get("containers") and g == "top" and v["height_m"] > CONTAINER_MAX_H)]
        if not graspable:
            reasons.append(f"{s['title']} stocks nothing {e['title']} can pick up with its {g} grasp (at most "
                           f"{e.get('max_grip_m', 0) * 100:g} cm wide)" +
                           (" (a level front grasp needs objects at least 7 cm tall and 4 cm wide)" if g == "front" else ""))
        if e["placement"] == "floor":
            lo, hi = e.get("reach", {}).get("grasp_heights_m", [0, 9])
            reach = [k for k, v in s["surfaces"].items() if v["reachable_from"] and lo <= v["height_m"] <= hi]
            need_n = needs.get("surfaces", 1)
            if len(reach) < need_n:
                reasons.append(f"{e['title']} reaches surfaces {lo:g} to {hi:g} m high from the floor; {s['title']} has "
                               f"{len(reach)} such surface{'s' if len(reach) != 1 else ''}" + (f", the task needs {need_n}" if need_n > 1 else ""))
    elif needs.get("surfaces", 0) > len(s.get("surfaces", {})):
        reasons.append(f"{s['title']} has fewer than {needs['surfaces']} work surfaces")
    # 4. modifiers
    if mods["perturbation"] == "push" and not t.get("perturbable", True):
        reasons.append(f"the push perturbation needs a task object; {t['title'].lower()} has none")
    if mods["perturbation"] in ("dynamics", "clutter") and not t.get("perturbable", True):
        reasons.append(f"the {mods['perturbation']} perturbation changes task objects; {t['title'].lower()} has none")
    if mods["safety"] not in ("none",) and mods["safety"] not in t.get("safety_levels", []):
        reasons.append(f"the {mods['safety']} safety overlay does not apply to {t['title'].lower()}")
    for k, v in mods.items():
        allowed = reg.modifiers.get(k, {}).get("values")
        if k != "seed" and allowed and v not in allowed:
            reasons.append(f"modifier {k} has no value {v!r} (values: {', '.join(allowed)})")
    return Report(not reasons, reasons, variant)
