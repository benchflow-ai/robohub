"""Reference solutions of the so101 suite. Each drives the arm(s) only through `robo` (the episode socket), like an
agent: `move_to` and `grip` sequences computed from `robo observe` (above the object, descend, close, lift, move,
lower, open), re-observing between grasps; scripted pulls and presses for the drawer and the buttons; for the ruler
handover the left arm presents the ruler at a fixed handover pose and the right arm grasps it before the left opens.

The only robot facts they use besides the observation: the jaws close toward the fixed jaw (an object ends with its
near face on the fixed jaw, `jaw_tips.fixed`), and `robo act` takes absolute joint targets with the wrist roll fifth.
"""

from __future__ import annotations

import math

import numpy as np

OPEN, CLOSED = 60, 2  # the real reference solutions' grip values
# The Koch's moving finger swings in a wide arc: opened to 60 it comes down on top of a 2.5 cm cube, so its picks open
# to 35 (3.6 cm between the pads)
KOCH_OPEN = 35
ROLL = 4  # wrist roll's index in an arm's six action values
MAX_DQ = 3.0
RULER_OPEN = 35
BOX_OPEN = 85  # the 6 cm box: wide enough, and the moving jaw still meets the far face below its top edge
S_RULER_HALF = 0.075  # half the ruler's length (robo observe: objects.ruler.size_m)


class Arm:
    """One arm of the rig, through the socket."""

    def __init__(self, o, arm: str | None = None):
        self.o, self.arm = o, arm

    def _pick(self, v):
        return v[self.arm] if self.arm else v

    def state(self) -> dict:
        return self.o.state()

    def tool(self, s=None) -> np.ndarray:
        return np.asarray(self._pick((s or self.state())["tool_pos"]), float)

    def jaw(self, s=None) -> np.ndarray:
        return np.asarray(self._pick((s or self.state())["jaw_axis"]), float)

    def fixed_tip(self, s=None) -> np.ndarray:
        return np.asarray(self._pick((s or self.state())["jaw_tips"])["fixed"], float)

    def tips_mid(self, s=None) -> np.ndarray:
        t = self._pick((s or self.state())["jaw_tips"])
        return (np.asarray(t["fixed"], float) + np.asarray(t["moving"], float)) / 2

    def holding(self, s=None):
        return self._pick((s or self.state())["holding"])

    def move(self, x, y, z, speed: float = 1.0) -> dict:
        args = [round(float(x), 4), round(float(y), 4), round(float(z), 4)]
        if self.arm:
            args.append(self.arm)
        return self.o.skill("move_to", *args, round(float(speed), 3))

    def open_value(self) -> float:
        return KOCH_OPEN if "Koch" in str(self.state().get("robot", "")) else OPEN

    def grip(self, v) -> dict:
        return self.o.skill("grip", v, *([self.arm] if self.arm else []))

    def turn_roll(self, deg: float) -> None:
        """Turn the wrist roll by `deg` with a low-level absolute target (the other joints keep theirs)."""
        n = int(math.ceil(abs(deg) / MAX_DQ))
        if n == 0:
            return
        a = targets(self.state())
        a[(6 if self.arm == "right" else 0) + ROLL] += deg
        n += 2  # the speed limit spreads the turn over n steps; two more to arrive
        while n > 0:  # at most 50 repeats per request
            self.o.act(a, min(n, 50))
            n -= 50

    def align_jaw(self, yaw_deg: float, period: float = 180.0) -> None:
        """Turn the wrist roll until the jaws close along world yaw `yaw_deg` (modulo `period`), closed loop."""
        sign = 1.0
        for _ in range(4):
            j = self.jaw()
            err = _wrap(yaw_deg - math.degrees(math.atan2(j[1], j[0])), period)
            if abs(err) < 1.5:
                return
            before = abs(err)
            self.turn_roll(sign * err)
            j = self.jaw()
            after = abs(_wrap(yaw_deg - math.degrees(math.atan2(j[1], j[0])), period))
            if after > before:
                sign = -sign

    def turn_held(self, name: str, yaw_deg: float, period: float = 180.0) -> None:
        """Turn the wrist roll until the held object `name` points along world yaw `yaw_deg` (modulo `period`)."""
        for _ in range(3):
            err = _wrap(yaw_deg - float(_obj(self.state(), name)["yaw_deg"]), period)
            if abs(err) < 1.5:
                return
            self.turn_roll(err)

    def grasp_point(self, center: np.ndarray, half: float, clearance: float = 0.002) -> np.ndarray:
        """Where the tool point goes so the fixed jaw stands `clearance` off the near face of an object `half` wide
        (along the jaws) centred at `center`; the closing jaw then pushes the object onto the fixed jaw."""
        s = self.state()
        j = self.jaw(s)
        j = j / max(1e-9, float(np.linalg.norm(j)))
        lead = float(np.dot(self.tool(s) - self.fixed_tip(s), j))  # tool point ahead of the fixed jaw's inner face
        return np.asarray(center, float) + (lead - half - clearance) * j


def _wrap(a: float, period: float) -> float:
    return (a + period / 2) % period - period / 2


def _obj(s: dict, name: str) -> dict:
    return s["objects"][name]


def pick(
    arm: Arm,
    name: str,
    *,
    half: float,
    z_grasp: float,
    z_above: float,
    yaw_period: float = 90.0,
    jaw_yaw: float | None = None,
) -> np.ndarray:
    """Open, go above the object, turn the jaws square to it, descend, close, lift to z_above. Returns the object's
    centre relative to the tool point after the lift (used to place it)."""
    arm.grip(arm.open_value())
    s = arm.state()
    p = np.asarray(_obj(s, name)["pos"], float)
    arm.move(p[0], p[1], z_above)
    yaw = jaw_yaw if jaw_yaw is not None else float(_obj(arm.state(), name).get("yaw_deg", 0.0))
    arm.align_jaw(yaw, yaw_period)
    s = arm.state()
    p = np.asarray(_obj(s, name)["pos"], float)
    g = arm.grasp_point(p, half)
    arm.move(g[0], g[1], z_above)
    arm.move(g[0], g[1], z_grasp + 0.03, 0.6)  # two short moves: the joint-space path stays close to a straight line
    arm.move(g[0], g[1], z_grasp, 0.4)
    arm.grip(CLOSED)
    arm.move(g[0], g[1], z_above, 0.5)
    s = arm.state()
    return np.asarray(_obj(s, name)["pos"], float) - arm.tool(s)


def place(arm: Arm, name: str, dest_xy, center_z: float, z_above: float) -> None:
    """Carry the held object over dest_xy, lower it until its centre is at center_z, open, back up."""
    s = arm.state()
    off = np.asarray(_obj(s, name)["pos"], float) - arm.tool(s)
    arm.move(dest_xy[0] - off[0], dest_xy[1] - off[1], z_above)
    s = arm.state()
    off = np.asarray(_obj(s, name)["pos"], float) - arm.tool(s)  # re-measured over the target
    t = np.array([dest_xy[0] - off[0], dest_xy[1] - off[1], center_z - off[2]])
    arm.move(t[0], t[1], t[2], 0.4)
    arm.grip(arm.open_value())
    arm.move(t[0], t[1], z_above, 0.5)


def targets(s: dict) -> list[float]:
    """Every joint's current target (the left arm's six, then the right arm's, on the two-arm rig): `robo act` with
    these holds the arm(s)."""
    t = s["target"]
    return [float(v) for v in (t["left"] + t["right"] if isinstance(t, dict) else t)]


def settle(o, steps: int = 5) -> None:
    o.act(targets(o.state()), steps)


# ---- tasks -----------------------------------------------------------------------------------------------------------


def reach(o) -> None:
    arm = Arm(o)
    p = o.state()["fixtures"]["marker"]["pos"]
    arm.move(*p)
    r = arm.tool()
    miss = np.asarray(p) - r
    if np.linalg.norm(miss) > 0.005:
        arm.move(*(np.asarray(p) + miss), 0.5)


def block_lift(o) -> None:
    # the real reference solution's sequence (grip 60, above, down, grip 2, up), from the observed cube
    arm = Arm(o)
    pick(arm, "cube", half=0.0125, z_grasp=0.014, z_above=0.08)
    settle(o, 3)


def _cube_into(o, arm: Arm, name: str, dest_xy, center_z: float, z_above: float = 0.085) -> None:
    pick(arm, name, half=0.0125, z_grasp=0.014, z_above=z_above)
    place(arm, name, dest_xy, center_z, z_above)


def cube_in_cup(o) -> None:
    arm = Arm(o)
    s = o.state()
    cup = np.asarray(_obj(s, "cup")["pos"], float)
    rim = cup[2] + _obj(s, "cup")["height_m"]
    _cube_into(o, arm, "cube", cup[:2], rim + 0.0125 + 0.008, z_above=0.09)


def stack(o, top: str = "red_cube", bottom: str = "blue_cube") -> None:
    arm = Arm(o)
    pick(arm, top, half=0.0125, z_grasp=0.014, z_above=0.08)
    s = o.state()
    b = np.asarray(_obj(s, bottom)["pos"], float)
    place(arm, top, b[:2], b[2] + 0.025 + 0.003, 0.08)


def sort_by_color(o) -> None:
    arm = Arm(o)
    slots = {"red": [-0.03, 0.03], "blue": [-0.03, 0.03]}
    s = o.state()
    for name in sorted(s["objects"]):
        ob = s["objects"][name]
        if ob["kind"] != "cube" or f"{ob['color']}_tray" not in s["objects"]:
            continue  # the green cube has no tray: it stays where it is
        tray = np.asarray(s["objects"][f"{ob['color']}_tray"]["pos"], float)
        yaw = math.radians(s["objects"][f"{ob['color']}_tray"]["yaw_deg"])
        dx = slots[ob["color"]].pop(0)
        dest = tray[:2] + dx * np.array([math.cos(yaw), math.sin(yaw)])
        _cube_into(o, arm, name, dest, tray[2] + 0.004 + 0.0125 + 0.004, z_above=0.08)
        s = o.state()


def push_puck(o) -> None:
    arm = Arm(o)
    arm.grip(CLOSED)
    s = o.state()
    zone = np.asarray(s["fixtures"]["zone"]["pos"][:2], float)
    R = s["objects"]["puck"]["diameter_m"] / 2
    z_push = 0.0105  # tool height: the closed fingertips at the puck's mid-height
    p0 = np.asarray(s["objects"]["puck"]["pos"][:2], float)
    arm.move(p0[0], p0[1], 0.05)  # gripper pointing down over the puck first
    for _ in range(8):
        s = o.state()
        p = np.asarray(s["objects"]["puck"]["pos"][:2], float)
        d = zone - p
        dist = float(np.linalg.norm(d))
        if dist < 0.006:
            break
        u = d / dist
        arm.align_jaw(math.degrees(math.atan2(u[1], u[0])) + 90.0, 180.0)  # fingers' flat side toward the push
        s = o.state()
        off = arm.tool(s)[:2] - arm.tips_mid(s)[:2]  # tool point relative to the closed fingertips
        behind = p - (R + 0.02) * u + off
        arm.move(behind[0], behind[1], 0.05)
        arm.move(behind[0], behind[1], z_push, 0.5)
        # push in short straight moves along u until the puck's centre is on the zone's
        reach = dist + 0.02 - 0.004  # fingertips from 2 cm behind the puck's edge to the contact point + dist
        n = max(1, int(math.ceil(reach / 0.02)))
        for k in range(1, n + 1):
            t = behind + u * reach * k / n
            arm.move(t[0], t[1], z_push, 0.4)
        s = o.state()
        back = arm.tool(s)[:2] - 0.01 * u
        arm.move(back[0], back[1], z_push, 0.5)
        arm.move(back[0], back[1], 0.05)


def open_drawer(o) -> None:
    arm = Arm(o)
    arm.grip(OPEN)
    s = o.state()
    dr = s["fixtures"]["drawer"]
    k = np.asarray(dr["knob_pos"], float)
    pull = np.asarray(dr["opens_toward"], float)
    arm.move(k[0], k[1], 0.08)
    # jaws across the pull, around the stem, with the jaw axis the pull direction turned 90 degrees counter-clockwise
    # seen from above (wrist roll near -90 on this layout): with the roll the other way round, the wrist camera mount
    # meets the forearm as the arm folds to pull the drawer toward its base, and the elbow stalls before the drawer is
    # 4 cm open
    arm.align_jaw(math.degrees(math.atan2(pull[1], pull[0])) + 90.0, 360.0)
    g = arm.grasp_point(k, 0.0045, clearance=0.0015)
    arm.move(g[0], g[1], 0.08)
    arm.move(g[0], g[1], k[2], 0.4)
    arm.grip(CLOSED)
    for step in (0.025, 0.05):
        t = g + pull * step
        arm.move(t[0], t[1], k[2], 0.3)
    arm.grip(OPEN)
    s = o.state()
    t = arm.tool(s)
    arm.move(t[0], t[1], 0.08, 0.5)


CODE = ["green_button", "green_button", "red_button", "blue_button"]  # the order the task's prompt gives


def press_buttons(o) -> None:
    arm = Arm(o)
    arm.grip(CLOSED)
    for _ in range(2):  # a missed press leaves the code unfinished: enter it again from the start
        for name in CODE:
            press_once(o, arm, name)
        if all(b["lit"] for b in o.state()["fixtures"].values()):
            break


def press_once(o, arm: Arm, name: str) -> None:
    """One full press of a button with the closed fingertips: down in 2 mm steps until the cap is in 5 mm (a press
    counts past 4.5 mm of its 8), then up 4 cm, which lets it up."""
    s = o.state()
    cap = np.asarray(s["fixtures"][name]["pos"], float)
    if np.linalg.norm(arm.tool(s)[:2] - cap[:2]) > 0.03:
        arm.move(cap[0], cap[1], cap[2] + 0.05)  # gripper pointing down over the button first
        s = o.state()
    off = arm.tool(s) - arm.tips_mid(s)  # from the closed fingertips to the tool point (measured pointing down)
    above = cap + off + np.array([0, 0, 0.04])
    arm.move(*above)
    z = cap[2] + off[2] + 0.004
    for _ in range(14):
        arm.move(cap[0] + off[0], cap[1] + off[1], z, 0.3)
        if o.state()["fixtures"][name]["pressed_in_m"] >= 0.005:
            break
        z -= 0.002
    arm.move(*above, 0.5)


def _ruler(o) -> tuple[np.ndarray, float, np.ndarray]:
    """The ruler's centre, yaw (deg) and long axis pointing to the rig's left (+y)."""
    r = _obj(o.state(), "ruler")
    yaw = math.radians(r["yaw_deg"])
    along = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    if along[1] < 0:
        along = -along
    return np.asarray(r["pos"], float), math.degrees(yaw), along


def ruler_handover(o) -> None:
    left, right = Arm(o, "left"), Arm(o, "right")
    # jaws opened to 35, not 60: the moving jaw then closes level with the 1 cm ruler instead of coming down on its edge
    left.grip(RULER_OPEN)
    right.grip(RULER_OPEN)
    # the left arm picks the ruler 5.5 cm left of its centre, jaws across it
    r, yaw, along = _ruler(o)
    gp = r + 0.055 * along
    left.move(gp[0], gp[1], 0.07)
    left.align_jaw(yaw + 90.0, 180.0)
    g = left.grasp_point(gp, 0.012)
    left.move(g[0], g[1], 0.07)
    left.move(g[0], g[1], 0.007, 0.5)
    left.grip(CLOSED)
    left.move(g[0], g[1], 0.07, 0.5)
    # the fixed handover pose: the ruler along y in front of the rig, its right part over the centre line
    left.move(0.20, 0.06, 0.09, 0.6)
    left.turn_held("ruler", 90.0)  # the base turned with the move: square the ruler to the rig again (along y)
    # the right arm lines up beyond the ruler's right end at its height, jaws across it, and slides in along it
    r, yaw, along = _ruler(o)
    far = r - (S_RULER_HALF + 0.065) * along
    right.move(far[0], far[1], r[2] + 0.001)
    right.align_jaw(yaw + 90.0, 180.0)
    r, yaw, along = _ruler(o)
    start = right.grasp_point(r - (S_RULER_HALF + 0.035) * along, 0.012)
    g = right.grasp_point(r - 0.055 * along, 0.012)
    right.move(start[0], start[1], r[2] + 0.001, 0.5)
    right.move(g[0], g[1], r[2] + 0.001, 0.4)
    right.grip(CLOSED)  # the right arm holds the ruler before the left lets go
    left.grip(OPEN)
    t = left.tool()
    left.move(t[0], t[1] + 0.05, t[2] - 0.01, 0.5)  # off the ruler's left end
    o.skill("home", "left")
    # into the tray: the ruler's centre over the tray's centre, just above its walls
    s = o.state()
    tray = np.asarray(_obj(s, "tray")["pos"], float)
    off = np.asarray(_obj(s, "ruler")["pos"], float) - right.tool(s)
    right.move(tray[0] - off[0], tray[1] - off[1], 0.08)
    right.turn_held("ruler", float(_obj(s, "tray")["yaw_deg"]))  # along the tray's long side
    s = o.state()
    off = np.asarray(_obj(s, "ruler")["pos"], float) - right.tool(s)
    z = tray[2] + 0.016 + 0.005 + 0.006 - off[2]
    right.move(tray[0] - off[0], tray[1] - off[1], z, 0.4)
    right.grip(OPEN)
    t = right.tool()
    right.move(t[0], t[1], 0.07, 0.5)


def lift_box(o) -> None:
    left, right = Arm(o, "left"), Arm(o, "right")
    s = o.state()
    bx = _obj(s, "box")
    c = np.asarray(bx["pos"], float)
    yaw = math.radians(bx["yaw_deg"])
    along = np.array([-math.sin(yaw), math.cos(yaw), 0.0])  # the box's long side (15 cm) is its local y
    if along[1] < 0:
        along = -along
    half_w = bx["size_m"][0] / 2
    z_g = c[2] + 0.008  # above the box's centre of mass, so it hangs level from the jaw tips
    z_up = z_g + 0.055
    grasp, up = {}, {}
    for arm, sgn in ((left, 1.0), (right, -1.0)):
        arm.grip(BOX_OPEN)
        p = c + sgn * 0.055 * along
        arm.move(p[0], p[1], z_up)
        arm.align_jaw(math.degrees(yaw), 180.0)  # jaws across the box's 6 cm width
        g = arm.grasp_point(p, half_w)
        arm.move(g[0], g[1], z_up)
        up[arm.arm] = list(o.state()["target"][arm.arm][:5])  # this arm's joints with the box lifted
        grasp[arm.arm] = g
    for arm in (left, right):
        g = grasp[arm.arm]
        arm.move(g[0], g[1], z_g, 0.5)
    left.grip(CLOSED)
    right.grip(CLOSED)
    # lift with both arms at once: back to the joint poses recorded above the grasps (one interpolated move)
    o.skill("move_joints", *[round(v, 2) for v in up["left"] + up["right"]], 0.4)
    settle(o, 5)


def koch_cube_in_cup(o) -> None:
    cube_in_cup(o)


def koch_stack(o) -> None:
    stack(o)


ORACLES = {
    "so101-reach": reach,
    "so101-block-lift": block_lift,
    "so101-cube-in-cup": cube_in_cup,
    "so101-stack-two-cubes": stack,
    "so101-sort-by-color": sort_by_color,
    "so101-push-puck-to-zone": push_puck,
    "so101-open-drawer": open_drawer,
    "so101-press-buttons-in-order": press_buttons,
    "so101-bimanual-ruler-handover": ruler_handover,
    "so101-bimanual-lift-box": lift_box,
    "koch-cube-in-cup": koch_cube_in_cup,
    "koch-stack-two-cubes": koch_stack,
}
