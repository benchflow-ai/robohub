"""Reference solutions for the humanoid suite. They use only the episode socket (`Oracle`: robo observe / skill / act)
and the public observation fields an agent sees; they prove each task is solvable through the public interface."""
from __future__ import annotations

import numpy as np


def _obj(o, name) -> np.ndarray:
    return np.array(o.state()["objects"][name]["pos"], float)


def _hand(o, side) -> np.ndarray:
    return np.array(o.state()["robot"]["hands"][side]["pos"], float)


# ---- G1 (Dex3 hands) -------------------------------------------------------------------------------------------------

def g1_pick(o, side: str, name: str, lift: float = .12, dz: float = -.005) -> float:
    """Approach from behind at grasp height (the object slides in between the thumb and the fingers), wrap grasp
    around the object's middle, lift. Returns the hand-object height offset."""
    p = _obj(o, name)
    o.skill("release", side)
    h = _hand(o, side)
    o.skill("reach", side, h[0], h[1], max(h[2], p[2] + .13))
    o.skill("reach", side, p[0] - .14, p[1], p[2] + .13)
    o.skill("reach", side, p[0] - .14, p[1], p[2] + dz)
    o.skill("reach", side, p[0], p[1], p[2] + dz, "tol=0.01")
    o.skill("grasp", side)
    o.skill("reach", side, p[0], p[1], p[2] + lift)
    return float(_hand(o, side)[2] - _obj(o, name)[2])


def g1_place(o, side: str, name: str, x: float, y: float, z_center: float, carry_z: float) -> None:
    """Carry at carry_z, lower until the object's centre is at z_center (+4 mm), open, lift straight up."""
    off = float(_hand(o, side)[2] - _obj(o, name)[2])
    o.skill("reach", side, x, y, carry_z)
    o.skill("reach", side, x, y, z_center + .004 + off, "tol=0.01")
    o.skill("release", side)
    h = _hand(o, side)
    o.skill("reach", side, h[0] - .13, h[1], h[2] + .005)   # back out the way the object came in, then up
    o.skill("reach", side, h[0] - .13, h[1], h[2] + .1)


def g1_can_to_plate(o) -> None:
    st = o.state()
    plate = st["scene"]["plate"]
    can = _obj(o, "can")
    g1_pick(o, "right", "can")
    half_h = can[2] - st["scene"]["table"]["top_height"]
    g1_place(o, "right", "can", plate["center"][0], plate["center"][1], plate["center"][2] + half_h, can[2] + .12)
    o.skill("home")


def g1_pass_across(o) -> None:
    st = o.state()
    table_h = st["scene"]["table"]["top_height"]
    b = _obj(o, "bottle")
    half_h = b[2] - table_h
    # the right hand cannot reach the coaster and the left hand cannot reach the bottle: put it down in the middle,
    # left of the glass, carrying it high enough to clear the glass
    g = st["objects"]["glass"]
    carry = table_h + g["height"] + half_h + .06
    g1_pick(o, "right", "bottle")
    g1_place(o, "right", "bottle", .29, .07, table_h + half_h, carry)
    o.skill("home")
    g1_pick(o, "left", "bottle")
    c = st["scene"]["coaster"]["center"]
    g1_place(o, "left", "bottle", c[0], c[1], c[2] + half_h, carry)
    o.skill("home")


def g1_drawer_stow(o) -> None:
    st = o.state()
    h = st["scene"]["drawer"]["handle"]["center"]
    # 1. right hand: slide the open hand forward around the vertical handle bar, close, pull the drawer 13 cm open
    o.skill("reach", "right", h[0] - .14, h[1], h[2] + .02)
    o.skill("reach", "right", h[0] - .14, h[1], h[2])
    o.skill("reach", "right", h[0], h[1], h[2], "tol=0.01")
    o.skill("grasp", "right")
    o.skill("reach", "right", h[0] - .15, h[1], h[2])
    o.skill("release", "right")
    hr = _hand(o, "right")
    o.skill("reach", "right", hr[0] - .1, hr[1], hr[2])
    o.skill("reach", "right", .2, -.2, .94)
    # 2. left hand: the soap into the open part of the tray
    st = o.state()
    tray = st["scene"]["drawer"]["tray_inside"]
    soap = _obj(o, "soap")
    half_h = soap[2] - st["scene"]["table"]["top_height"]
    g1_pick(o, "left", "soap")
    # the hand cannot go below the drawer front's top edge: hold the soap just above it and let it drop in
    x = st["scene"]["cabinet"]["front_x"] - .045
    g1_place(o, "left", "soap", x, .04, st["scene"]["cabinet"]["top_height"] + half_h + .025, soap[2] + .14)
    o.skill("home")
    # 3. right hand: push the handle bar with the palm until the drawer is shut
    h = o.state()["scene"]["drawer"]["handle"]["center"]
    o.skill("reach", "right", h[0] - .14, h[1], h[2])
    o.skill("reach", "right", h[0] + .2, h[1], h[2])   # the bar meets the palm about 4 cm behind the control point
    hr = _hand(o, "right")
    o.skill("reach", "right", hr[0] - .12, hr[1], hr[2])
    o.skill("home")


# ---- H1 (fists) ------------------------------------------------------------------------------------------------------

H1_FIST_R = .033


def squeeze_carry(o, name: str, target_xy, target_top: float, fist_r: float, squeeze: float = .03, clear: float = .08,
                  lift: float = .14, over: float = .12) -> None:
    """Bimanual squeeze: hands beside the box at its mid height, press inward, lift, carry, lower onto the target
    surface (top at target_top), open the hands, back off."""
    st = o.state()
    b = np.array(st["objects"][name]["pos"], float)
    size = st["objects"][name]["size"]
    hy = size[1] / 2
    zc = b[2]
    yl, yr = b[1] + hy + fist_r, b[1] - hy - fist_r
    o.skill("reach_both", b[0], yl + clear, zc + over, b[0], yr - clear, zc + over)
    o.skill("reach_both", b[0], yl + clear, zc, b[0], yr - clear, zc)
    o.skill("reach_both", b[0], yl - squeeze, zc, b[0], yr + squeeze, zc)
    o.skill("wait", 8)
    o.skill("reach_both", b[0], yl - squeeze, zc + lift, b[0], yr + squeeze, zc + lift)
    tx, ty = target_xy
    dy = ty - b[1]
    o.skill("reach_both", tx, yl - squeeze + dy, zc + lift, tx, yr + squeeze + dy, zc + lift)
    zt = target_top + size[2] / 2
    o.skill("reach_both", tx, yl - squeeze + dy, zt + .005, tx, yr + squeeze + dy, zt + .005)
    o.skill("reach_both", tx, yl + clear + dy, zt + .005, tx, yr - clear + dy, zt + .005)
    o.skill("reach_both", tx - .06, yl + clear + dy, zt + .1, tx - .06, yr - clear + dy, zt + .1)
    o.skill("home")


def h1_box_to_shelf(o) -> None:
    sh = o.state()["scene"]["shelf"]
    squeeze_carry(o, "box", sh["top_center"][:2], sh["top_center"][2], H1_FIST_R)


def press_sequence(o, seq, reach_r: float, depth: float = .012) -> None:
    """Press each button with the hand on its side: line up in front of it, push in along +x, back off."""
    st = o.state()
    btns = st["scene"]["panel"]["buttons"]
    for n in seq:
        c = btns[n]["center"]
        side = "left" if c[1] > 0 else "right"
        o.skill("reach", side, c[0] - reach_r - .08, c[1], c[2])
        o.skill("reach", side, c[0] - reach_r + depth, c[1], c[2], "tol=0.006")
        o.skill("reach", side, c[0] - reach_r - .08, c[1], c[2])
    o.skill("home")


def h1_panel_sequence(o) -> None:
    press_sequence(o, ["yellow", "green", "white", "blue"], H1_FIST_R)


# ---- Apollo (flat hand plates, palms facing each other) -----------------------------------------------------------------

APOLLO_HALF_PALM = .02   # half thickness of the hand plate: the control point is at the plate's centre


def apollo_box_into_bin(o) -> None:
    st = o.state()
    b = st["scene"]["bin"]
    # lift high enough to clear the bin wall with the box bottom
    box = st["objects"]["box"]
    lift = b["wall_height"] + .06
    squeeze_carry(o, "box", b["center"][:2], b["floor_height"], APOLLO_HALF_PALM, squeeze=.02, clear=.06, lift=lift, over=.1)


def apollo_carry_tray(o) -> None:
    st = o.state()
    tray = st["objects"]["tray"]
    t = np.array(tray["pos"], float)
    hy = tray["size"][1] / 2
    table_h = st["scene"]["table"]["top_height"]
    z = table_h + .085                   # the hand plate (8 cm tall) overlaps the upper part of the tray's side
    yl, yr = t[1] + hy + APOLLO_HALF_PALM, t[1] - hy - APOLLO_HALF_PALM
    sq, cl = .015, .06
    o.skill("reach_both", t[0], yl + cl, z + .12, t[0], yr - cl, z + .12)
    o.skill("reach_both", t[0], yl + cl, z, t[0], yr - cl, z)
    o.skill("reach_both", t[0], yl - sq, z, t[0], yr + sq, z)
    o.skill("wait", 6)
    o.skill("reach_both", t[0], yl - sq, z + .08, t[0], yr + sq, z + .08)
    m = st["scene"]["mat"]["center"]
    dy = m[1] - t[1]
    o.skill("reach_both", m[0], yl - sq + dy, z + .08, m[0], yr + sq + dy, z + .08)
    o.skill("reach_both", m[0], yl - sq + dy, z + .004, m[0], yr + sq + dy, z + .004)
    o.skill("reach_both", m[0], yl + cl + dy, z + .004, m[0], yr - cl + dy, z + .004)
    o.skill("reach_both", m[0] - .08, yl + cl + dy, z + .12, m[0] - .08, yr - cl + dy, z + .12)
    o.skill("home")


# ---- T1 (rigid cylindrical hands) -----------------------------------------------------------------------------------

T1_HAND_R = .03


def push_to(o, side: str, name: str, target, hand_r: float, obj_r: float, tol: float = .012, z_off: float = .045,
            max_rounds: int = 8) -> None:
    """Closed-loop pushing: line the hand up behind the object (opposite the target), push it part of the way,
    re-observe, repeat."""
    table_h = o.state()["scene"]["table"]["top_height"]
    z = table_h + z_off + hand_r * 0
    target = np.array(target[:2], float)
    for _ in range(max_rounds):
        p = _obj(o, name)[:2]
        d = target - p
        dist = float(np.linalg.norm(d))
        if dist < tol:
            break
        u = d / dist
        pre = p - u * (obj_r + hand_r + .03)
        h = _hand(o, side)
        if float(np.linalg.norm(h[:2] - pre)) > .02:
            o.skill("reach", side, h[0], h[1], z + .08)
            o.skill("reach", side, pre[0], pre[1], z + .08)
            o.skill("reach", side, pre[0], pre[1], z)
        step = min(dist, .07)
        end = p + u * step - u * (obj_r + hand_r - .004)
        o.skill("reach", side, end[0], end[1], z, "tol=0.006")
    h = _hand(o, side)
    back = h[:2] - (target - h[:2]) / max(float(np.linalg.norm(target - h[:2])), 1e-6) * .03
    o.skill("reach", side, back[0], back[1], h[2])
    o.skill("reach", side, back[0], back[1], h[2] + .08)


def t1_push_pucks(o) -> None:
    st = o.state()
    zones = st["scene"]["zones"]
    r = st["objects"]["red_can"]["diameter"] / 2
    push_to(o, "right", "red_can", zones["red"]["center"], T1_HAND_R, r)
    o.skill("home")
    push_to(o, "left", "blue_can", zones["blue"]["center"], T1_HAND_R, r)
    o.skill("home")


def t1_box_onto_step(o) -> None:
    sh = o.state()["scene"]["step"]
    squeeze_carry(o, "box", sh["top_center"][:2], sh["top_center"][2], T1_HAND_R, squeeze=.025, clear=.06, lift=.1, over=.1)


ORACLES = {
    "g1-can-to-plate": g1_can_to_plate,
    "g1-pass-across": g1_pass_across,
    "g1-drawer-stow": g1_drawer_stow,
    "h1-box-to-shelf": h1_box_to_shelf,
    "h1-panel-sequence": h1_panel_sequence,
    "apollo-box-into-bin": apollo_box_into_bin,
    "apollo-carry-tray": apollo_carry_tray,
    "t1-push-cans": t1_push_pucks,
    "t1-box-onto-step": t1_box_onto_step,
}
