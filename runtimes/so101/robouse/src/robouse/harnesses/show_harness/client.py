"""`show-harness` harness: NUS Show Lab's Show-Harness (arXiv 2609.10522; github.com/showlab/Show-Harness, Apache-2.0)
driving a Robo Use episode. Standard library only: the runner copies this file into the trial workspace and runs it
with `python3` inside the agent sandbox, like every harness; it reaches the robot only through the `robo` socket.

A VLM chooses one parameter-free action unit per decision (MV_FWD, MV_BACK, MV_LEFT, MV_RIGHT, MV_UP, MV_DOWN,
GRASP, RELEASE, DONE) and an embodiment interpreter grounds it into motion: a move drives the hand a fixed step
(2 cm; 4 cm coarse) closed-loop, one `act` per control step, aimed at a lattice node so steps do not drift (Show-
Harness's simulator executor, `AtomicExec`); GRASP and RELEASE first wait for the hand to stop, then hold the gripper
command. Prompts, parsing and plugin logic follow the repository at commit 403f068; the texts are quoted verbatim.

Two modes, as in the paper:
  zs  zero-shot: a subtask planner call, then one controller call per decision with the plugins that apply in
      simulation (proprioception, action history, adaptive step, action chunking, failure recovery; multi-view when
      the task has a wrist camera). Used for models behind the trial's model gateway.
  ft  fine-tuned: the lite v3 prompt (task sentence + recent moves), one bare token per call, thinking off, the
      LIBERO runner's stuck guard. Used for a self-hosted fine-tuned model (e.g. the Qwen3.5-2B LoRA on vLLM).

Environment (set by the harness builder):
  ROBOUSE_SOCKET               the episode
  ROBOUSE_SHOW_HARNESS_MODEL   the model name to send
  ROBOUSE_GATEWAY_URL/_KEY     the trial's model gateway (provider-prefixed models), or
  ROBOUSE_SHOW_HARNESS_URL     an OpenAI-compatible server (base URL ending in /v1), with ROBOUSE_SHOW_HARNESS_TOKEN
  ROBOUSE_SHOW_HARNESS_SETTINGS  JSON: mode, the task sentence for ft, and overrides of DEFAULTS
argv[1]: the task text (the task.md body). Output: one JSON line per event on stdout (agent/stdout.jsonl).
"""

from __future__ import annotations

import base64
import json
import math
import os
import re
import socket
import struct
import sys
import time
import urllib.error
import urllib.request
import zlib

# ======================================================================================================================
# The action units (core/action_units.py)
# ======================================================================================================================

MOVE_ATOMS = ("MV_FWD", "MV_BACK", "MV_LEFT", "MV_RIGHT", "MV_UP", "MV_DOWN")
GRASP, RELEASE, DONE, STOP = "GRASP", "RELEASE", "DONE", "STOP"
CONTROLLER_TOKENS = MOVE_ATOMS + (GRASP, RELEASE, DONE)
EMPTY_GRASP_LABEL = "GRASP(empty)"  # core/runners/real.py

# ======================================================================================================================
# Prompts, verbatim from the repository (403f068)
# ======================================================================================================================

# plugins/subgoal/subgoal_planner.txt
PLANNER_PROMPT = """ROLE: SubgoalPlanner
TASK: {task}

{video_ref}

Return an ordered JSON plan:
{{
  "subgoals": [
    {{
      "id": "short_snake_case_id",
      "target": "object or destination",
      "affordance": "visible part or placement region",
      "motion": "semantic stage label",
      "description": "visual strategy for this stage",
      "completion": "visible condition that means this stage is complete"
    }}
  ]
}}

### STRICT RULES

1. Stage Segmentation

-Break the task down into meaningful visual milestones (e.g., GRASP, LIFT, MOVE, PLACE, RELEASE, RETREAT)
-MERGE: Do NOT split immediate pre-grasp steps. Combine approach, align, lower, and close into a single `GRASP` stage
-SEPARATE: Keep lift/clearance after a successful grasp as a separate `LIFT` stage
-RETREAT: After every `RELEASE`, add a `RETREAT` stage that lifts the gripper up

2. Affordance Selection

-ONE specific part -- never alternatives like "left end or middle"
-Must be visible in AgentView & bracketable by open fingers
-Containers/Hollow objects (cups, bowls): Target left/right rim or edge.
-Simple Solid objects (blocks): main body

3. Completion Criteria

-ALL completion conditions MUST be strictly judgeable from raw 2D images
-Movement stages: End with a stable visual spatial relation, NOT a gripper event
-Set-aside placements: If moving an object to another location, ensure it is placed away from the origin point on the table
-MUST distinguish among similar objects

Return JSON ONLY."""

# prompts/controller.txt
CONTROLLER_PROMPT = """TASK: {task}
STAGE: {stage}
TARGET: {target}
AFFORD: {affordance}
Stage goal: {description}
DONE WHEN: {completion}
Gripper now: {gripper_state}
{mem_text}
{recovery}
{proprio}

DIRECTION:
Is STAGE for grasping AND TARGET inside the wrist view?
A) YES -> wrist is the primary guide. Judge AFFORD's position vs the {gripper_color} gripper and take the direction of LARGEST deviation:
- AFFORD to the gripper's left  -> MV_LEFT
- AFFORD to the gripper's right -> MV_RIGHT
- AFFORD near image bottom and far from the grippers -> MV_FWD
- AFFORD between the image top and the grippers -> MV_BACK
- AFFORD roughly centered between the grippers -> MV_DOWN
{rotation}
B) NO -> AgentView is the primary guide. Judge TARGET's position vs the end effector (before GRASP) or holding object (after GRASP), and take the direction of LARGEST deviation:
- TARGET near the image left  -> MV_LEFT
- TARGET near the image right -> MV_RIGHT
- TARGET near the image bottom -> MV_FWD
- TARGET near the image top -> MV_BACK
C) MV_UP when:
- Need to lift the object
- too low to reach TARGET
- retreating after a RELEASE

ATTENTION:
{mem_text_rules}
- DONE only when "DONE WHEN" is already visible in the images

GRIPPER:
- GRASP when BOTH AgentView and Wrist view confirm the {affordance} is clearly between the center of two grippers
- RELEASE only when the held object is above its destination and lowered onto it
{variable_step}
{action_chunk}
Think one visual sentence, then commit.
{output_contract}
"""

# prompts/v3/mvtoken_generator_lite.txt (the prompt every released adapter was trained on; no trailing newline)
FT_PROMPT = """You are controlling a robot arm with two cameras:
- Agentview: overhead view of the robot and workspace
- Wristview: close-up view from the gripper

Task: {task}
Recent moves, newest first: {recent_moves}

Output exactly one action token:
MV_FWD, MV_BACK, MV_LEFT, MV_RIGHT, MV_UP, MV_DOWN, GRASP, RELEASE, DONE

Check both camera views and choose the next action:
- Use AgentView to locate the target when it is not in wrist view
- Use wrist view to fine-align when target is visible up close
- GRASP when gripper fingers are aligned around the object
- RELEASE when object is above the destination
- DONE when the task is complete: the object is at its destination and the gripper is clear
- Avoid repeating a direction that conflicts with the most recent move

Return the single token only, no punctuation, no explanation:"""

# plugin fragments (plugins/*/*.txt, core/prompting/wrist_marker.txt)
PROPRIO_BLOCK = "The gripper is {gap_cm} cm above the table; {step_sizes}. {hint}"
PROPRIO_STEP_FINE = "each step moves ~{fine_cm} cm"
PROPRIO_STEP_COARSE = "each step moves ~{fine_cm} cm ({coarse_cm} cm)"
PROPRIO_HINT_DESCEND = "If height > {high_cm} cm, MV_DOWN first."
PROPRIO_HINT_HOLDING = "Holding an object: lift until clear of the table; descend only to place."
PROPRIO_DESCEND_STALL = (
    "Last MV_DOWN lowered {moved_cm} of {commanded_cm} cm -> already in contact, do NOT MV_DOWN again"
)
MEM_RECENT = "Recent moves, newest first: {moves}"
MEM_RULES = (
    "- If recent moves show GRASP(empty), do not GRASP in place again; prioritize MV_UP, MV_BACK, MV_DOWN, or MV_FWD\n"
    "- NEVER OSCILLATE: Do NOT choose the opposite of the newest recent move (Pairs: MV_LEFT/MV_RIGHT, MV_FWD/MV_BACK)\n"
    "- When opposite directions appear in recent moves, prioritize MV_DOWN or MV_UP"
)
CHUNK_PLAN = (
    "ACTION PLAN -- only when WRIST: NO (TARGET far): plan your next {step_num} moves as `PLAN: M1, M2, ...` (MV_ "
    "tokens only) and set decision to M1. Choose each move from the height and step size so the plan does not "
    "overshoot (e.g. never plan more MV_DOWN than the height above the table allows). When WRIST: YES, decide a "
    "single move."
)
WRIST_MARKER = (
    "WRIST CHECK: begin your reasoning with `WRIST: YES` if the TARGET is visible in the wrist view, else `WRIST: NO`."
)
RECOVERY_CONTEXT = "Recovery: {note}"
RECOVERY_NOTES = {
    "empty_grasp": "Empty close; do not retry on an edge/corner. Recenter body and confirm depth.",
    "unverified_grasp": "Grasp not verified; continue GRASP until width and images show a real hold.",
    "lost_grasp": "Grasp lost; return to GRASP, recenter the object body, then confirm depth.",
    "unsettled": "Closed width is unsettled; wait before moving.",
}
IMAGE_ROLES = (
    "LIVE AgentView: authoritative global scene and object positions.",
    "LIVE Wrist view: local gripper detail only; do not infer hidden global positions.",
)

# Robo Use's adapter text: what the task text's `robo` commands mean here, and this scene's unit conventions
ADAPTER = (
    "This task runs in Robo Use. The task text was written for an agent that types `robo` commands; here you "
    "control the robot only by choosing one action unit per step, and the harness executes it (and ends the episode "
    "when you finish). {conventions}"
)

# ======================================================================================================================
# Settings: the repository's defaults (configs/robot_franka.yaml for zs; scripts/run_libero_mvtoken.py for ft)
# ======================================================================================================================

DEFAULTS = {
    "zs": {
        "max_decisions": 200,
        "max_stage_decisions": 40,
        "fine_step_m": 0.02,
        "coarse_step_m": 0.04,
        "up_step_m": 0.04,
        "high_above_table_m": 0.08,
        "chunk_steps": 3,
        "history": 5,
        "max_tokens": 2048,
        "planner_max_tokens": 6144,
        "temperature": 0.0,
        "plugins": {
            "subgoal": True,
            "proprioception": True,
            "recovery": True,
            "variable_step": True,
            "action_chunk": True,
            "mem_text": True,
        },
    },
    "ft": {
        "max_decisions": 400,
        "fine_step_m": 0.02,
        "history": 5,
        "max_tokens": 8,
        "temperature": 0.0,
        "stuck_guard_mm": 5.0,
        "stuck_repeat": 2,
    },
}
TOL_M = 0.001  # AtomicExec: a move stops within 1 mm of its target ...
MAX_CTRL_STEPS = 24  # ... or after 24 control steps (the LIBERO runner's value)
QUIESCE_TOL_M = 0.0003  # "stopped": under 0.3 mm per control step
QUIESCE_MAX_STEPS = 40
# A unit whose motion is blocked (the hand on the table or pressing on an object) ends once the hand has stopped for
# this many control steps, instead of running out the executor's 24: Robo Use charges every control step to the
# episode's budget, and Show-Harness's real-arm interpreter spends a fixed 4 re-commands per unit, blocked or not
STALL_STEPS = 3
EMPTY_WIDTH_M = 0.005  # the simulator configs' empty-grasp width
OPEN_WIDTH_M = 0.07
CLOSED_BELOW_M = 0.07  # gripper "CLOSED" in the prompt when the measured width is below this
HTTP_RETRIES = 10

# ======================================================================================================================
# Embodiments: how the units ground on each Robo Use backend
# ======================================================================================================================

PROFILES = {
    # floating two-finger gripper; front camera at -y looking at the table; x right in its image; 1 cm per unit
    "tabletop": {
        "dirs": {
            "MV_FWD": (0.0, -1.0, 0.0),
            "MV_BACK": (0.0, 1.0, 0.0),
            "MV_LEFT": (-1.0, 0.0, 0.0),
            "MV_RIGHT": (1.0, 0.0, 0.0),
            "MV_UP": (0.0, 0.0, 1.0),
            "MV_DOWN": (0.0, 0.0, -1.0),
        },
        "pos": (0, 1, 2),
        "grip": 3,
        "unit_m": 0.01,
        "max_cmd_m": 0.02,
        "table_z": 0.0,
        # the backend's tool-point limits (scene.HAND_LO, HAND_HI): the setpoint is projected into them (the paper's
        # Pi_E, as the real interpreters' workspace box and Z floor do)
        "workspace": ((-0.34, -0.27, 0.012), (0.34, 0.28, 0.35)),
        "width": "fraction_10cm",
        "gripper_steps": 20,
        "wrist": ("wrist",),
        "flip": {},
        "gripper_color": "dark grey",
        "conventions": (
            "Each MV_* unit moves the gripper about {fine_cm} cm ({coarse_cm} cm when coarse) in the task's table "
            "frame: MV_FWD = -y (toward the camera, image bottom), MV_BACK = +y (image top), MV_LEFT = -x (image "
            "left), MV_RIGHT = +x (image right), MV_UP = +z, MV_DOWN = -z. GRASP closes the fingers, RELEASE opens "
            "them."
        ),
    },
    # Franka Panda in LIBERO, OSC_POSE: 1.0 asks for 5 cm (0.5 rad) per control step. Robo Use's LIBERO images are
    # the cameras' true views (they match MuJoCo's own render of the same camera). In the agentview +x (MV_FWD)
    # shows toward the image bottom and +y (MV_RIGHT) toward the right, as the controller's rules read the AgentView;
    # the wrist view shows both reversed, so it is turned 180 degrees to read the same way (measured with the
    # cameras' calibration). Show-Harness's own LIBERO contract (configs/robot_libero.yaml: agentview turned 180
    # degrees, wrist native) was calibrated on their LIBERO-plus renders, which come out mirrored relative to these.
    "libero": {
        "dirs": {
            "MV_FWD": (1.0, 0.0, 0.0),
            "MV_BACK": (-1.0, 0.0, 0.0),
            "MV_LEFT": (0.0, -1.0, 0.0),
            "MV_RIGHT": (0.0, 1.0, 0.0),
            "MV_UP": (0.0, 0.0, 1.0),
            "MV_DOWN": (0.0, 0.0, -1.0),
        },
        "pos": (0, 1, 2),
        "grip": 6,
        "rot": (3, 4, 5),
        "unit_m": 0.05,
        "rot_unit_rad": 0.5,
        "max_cmd_m": 0.02,
        "table_z": 0.0,
        "width": "metres",
        "gripper_steps": 25,
        "wrist": ("robot0_eye_in_hand",),
        "flip": {"wrist": "hv"},
        "gripper_color": "",
        "conventions": (
            "Each MV_* unit moves the gripper about {fine_cm} cm ({coarse_cm} cm when coarse) in the task's world "
            "frame: MV_FWD = +x (away from the robot base, toward the agentview camera, image bottom), MV_BACK = -x "
            "(image top), MV_LEFT = -y (image left), MV_RIGHT = +y (image right), MV_UP = +z, MV_DOWN = -z. GRASP "
            "closes the fingers, RELEASE opens them."
        ),
    },
    # inspect-robots' 2-D reach world: a point in the unit square, top image with +x right and +y down. The square is
    # taken as a 50 cm table (the scale of the paper's workspace), so a 2 cm unit is 0.04 of its side
    "cubepick": {
        "dirs": {
            "MV_FWD": (0.0, 1.0, 0.0),
            "MV_BACK": (0.0, -1.0, 0.0),
            "MV_LEFT": (-1.0, 0.0, 0.0),
            "MV_RIGHT": (1.0, 0.0, 0.0),
        },
        "pos": (0, 1, None),
        "grip": None,
        "pos_scale_m": 0.5,
        "unit_m": 0.5,
        "max_cmd_m": 0.05,
        "workspace": ((0.0, 0.0, -1.0), (0.5, 0.5, 1.0)),  # the unit square
        "table_z": None,
        "width": None,
        "gripper_steps": 0,
        "wrist": (),
        "flip": {},
        "gripper_color": "red",
        "conventions": (
            "The end effector is the red square, seen from above on a square table taken as 50 cm wide (the task's unit "
            "square); it moves only in the plane of the image, about {fine_cm} cm ({coarse_cm} cm when coarse) per MV_* "
            "unit: MV_FWD = +y (image bottom), MV_BACK = -y (image top), MV_LEFT = -x (image left), MV_RIGHT = +x "
            "(image right). There is no MV_UP, MV_DOWN, GRASP or RELEASE here; finish with DONE when the red square "
            "touches the green cube."
        ),
    },
}


def executable_units(prof: dict) -> tuple[str, ...]:
    """The units this robot can carry out: its translations, GRASP/RELEASE when it has a gripper, and DONE."""
    return tuple(
        t
        for t in CONTROLLER_TOKENS
        if (t in MOVE_ATOMS and t in prof["dirs"])
        or (t in (GRASP, RELEASE) and prof.get("grip") is not None)
        or t == DONE
    )


def pick_profile(info: dict) -> tuple[str | None, str]:
    """The profile for this episode's robot, or (None, why not)."""
    backend = str(info.get("backend") or "")
    names = [str(n).lower() for n in (info.get("action") or {}).get("names") or []]
    if backend == "tabletop" and names == ["dx", "dy", "dz", "grip"]:
        return "tabletop", ""
    if backend == "libero" and names[:3] == ["dx", "dy", "dz"] and len(names) == 7:
        return "libero", ""
    if backend == "cubepick" and names == ["dx", "dy"]:
        return "cubepick", ""
    return None, (
        f"show-harness has no unit conventions for the {backend or 'unknown'} backend (action {names}); "
        "supported: tabletop, libero, cubepick"
    )


# ======================================================================================================================
# Output
# ======================================================================================================================


def emit(**kw) -> None:
    print(json.dumps(kw, default=str), flush=True)


def clip_text(text: str, limit: int = 6000) -> str:
    text = str(text or "")
    return text if len(text) <= limit else text[: limit // 2] + "\n...\n" + text[-limit // 2 :]


def now() -> float:
    return round(time.time(), 3)


# ======================================================================================================================
# The episode socket
# ======================================================================================================================

STEPPING = ("act", "move_to", "grip", "skill", "done", "give_up")


class EpisodeOver(Exception):
    """The episode has ended (success under success_mode first, budget, wall clock)."""


class Robot:
    """The `robo` socket, with every request recorded in order (the trajectory converter aligns them 1:1 with the
    episode trace)."""

    def __init__(self, sock: str):
        self.sock = sock
        self.requests: list[str] = []
        self.finished = ""  # why the episode ended, once it has
        self.state: dict = {}
        self.steps_used = 0

    def send(self, req: dict) -> dict:
        self.requests.append(label(req))
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(3600 if req.get("op") in STEPPING else 120)
        try:
            s.connect(self.sock)
            s.sendall((json.dumps(req) + "\n").encode())
            buf = b""
            while not buf.endswith(b"\n"):
                chunk = s.recv(1 << 20)
                if not chunk:
                    break
                buf += chunk
            resp = json.loads(buf.decode() or '{"ok": false, "error": "empty response"}')
        except (FileNotFoundError, ConnectionRefusedError):
            resp = {"ok": False, "error": "episode server is not running"}
        finally:
            s.close()
        r = resp.get("result") if isinstance(resp.get("result"), dict) else {}
        if isinstance(r.get("state"), dict):
            self.state = r["state"]
        if isinstance(r.get("steps_used"), int):
            self.steps_used = r["steps_used"]
        if "episode" in r:
            self.finished = str(r["episode"])
        elif not resp.get("ok") and re.search(r"episode (finished|server is not running)", str(resp.get("error"))):
            self.finished = str(resp.get("error"))
        return resp

    def take_requests(self) -> list[str]:
        out, self.requests = self.requests, []
        return out


def label(req: dict) -> str:
    """The `robo` command line a request corresponds to (for the trajectory)."""
    op = req.get("op")
    if op == "act":
        a = " ".join(f"{float(x):.4g}" for x in req.get("action") or [])
        return f"robo act {a}" + (f" --repeat {req['repeat']}" if req.get("repeat", 1) != 1 else "")
    if op == "observe":
        return (
            "robo observe"
            + (" --image" if req.get("image") else "")
            + (f" --camera {req['camera']}" if req.get("camera") else "")
        )
    if op in ("done", "give_up"):
        return f"robo {op.replace('_', '-')} {json.dumps(req.get('text', ''))}"
    return f"robo {op}"


# ======================================================================================================================
# PNG (8-bit, non-interlaced): enough to apply a camera contract's flips without a third-party package
# ======================================================================================================================


def png_read(path: str) -> tuple[int, int, int, list[bytearray]]:
    """(width, height, bytes per pixel, rows) of an 8-bit non-interlaced PNG."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    pos, idat, w = 8, b"", 0
    h = bpp = 0
    while pos < len(data):
        n = struct.unpack(">I", data[pos : pos + 4])[0]
        kind, body = data[pos + 4 : pos + 8], data[pos + 8 : pos + 8 + n]
        pos += 12 + n
        if kind == b"IHDR":
            w, h, depth, ctype, _, _, interlace = struct.unpack(">IIBBBBB", body)
            if depth != 8 or interlace != 0 or ctype not in (0, 2, 4, 6):
                raise ValueError(f"unsupported PNG (depth {depth}, colour type {ctype}, interlace {interlace})")
            bpp = {0: 1, 2: 3, 4: 2, 6: 4}[ctype]
        elif kind == b"IDAT":
            idat += body
        elif kind == b"IEND":
            break
    raw = zlib.decompress(idat)
    stride = w * bpp
    rows: list[bytearray] = []
    prev = bytearray(stride)
    for y in range(h):
        ft = raw[y * (stride + 1)]
        row = bytearray(raw[y * (stride + 1) + 1 : (y + 1) * (stride + 1)])
        if ft == 1:
            for i in range(bpp, stride):
                row[i] = (row[i] + row[i - bpp]) & 0xFF
        elif ft == 2:
            for i in range(stride):
                row[i] = (row[i] + prev[i]) & 0xFF
        elif ft == 3:
            for i in range(stride):
                left = row[i - bpp] if i >= bpp else 0
                row[i] = (row[i] + ((left + prev[i]) >> 1)) & 0xFF
        elif ft == 4:
            for i in range(stride):
                a = row[i - bpp] if i >= bpp else 0
                b = prev[i]
                c = prev[i - bpp] if i >= bpp else 0
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                row[i] = (row[i] + (a if pa <= pb and pa <= pc else (b if pb <= pc else c))) & 0xFF
        elif ft != 0:
            raise ValueError(f"bad PNG filter {ft}")
        rows.append(row)
        prev = row
    return w, h, bpp, rows


def png_write(path: str, w: int, h: int, bpp: int, rows: list[bytearray]) -> None:
    ctype = {1: 0, 3: 2, 2: 4, 4: 6}[bpp]

    def chunk(kind: bytes, body: bytes) -> bytes:
        return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    with open(path, "wb") as f:
        f.write(
            b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, ctype, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6))
            + chunk(b"IEND", b"")
        )


def flip_png(src: str, dst: str, how: str) -> None:
    """Flip an image horizontally ("h"), vertically ("v") or both ("hv", a 180-degree turn)."""
    w, h, bpp, rows = png_read(src)
    if "h" in how:
        out = []
        for r in rows:
            m = bytearray(len(r))
            for c in range(bpp):
                m[c::bpp] = r[c::bpp][::-1]
            out.append(m)
        rows = out
    if "v" in how:
        rows = rows[::-1]
    png_write(dst, w, h, bpp, rows)


def data_uri(path: str) -> str:
    with open(path, "rb") as f:
        return "data:image/png;base64," + base64.b64encode(f.read()).decode()


# ======================================================================================================================
# The model: one OpenAI-compatible chat-completions endpoint
# ======================================================================================================================


class SpendCap(Exception):
    """The trial's model gateway refused the request: the trial's spend cap is reached."""


class Model:
    """The trial's model gateway (`hosted`: provider dialect, no vLLM-only fields) or a self-hosted vLLM server
    (`vllm`: chat_template_kwargs and structured outputs)."""

    def __init__(self, token_header: str = "") -> None:
        self.name = os.environ.get("ROBOUSE_SHOW_HARNESS_MODEL", "")
        served = os.environ.get("ROBOUSE_SHOW_HARNESS_URL", "")
        self.headers = {"Content-Type": "application/json"}
        if served:
            self.url = served.rstrip("/") + "/chat/completions"
            key = os.environ.get("ROBOUSE_SHOW_HARNESS_TOKEN", "") or "EMPTY"
            self.dialect = "vllm"
            # a bearer token (vLLM --api-key), or the token under another header (a private Daytona preview:
            # x-daytona-preview-token)
            if token_header and token_header.lower() != "authorization":
                self.headers[token_header] = key
            else:
                self.headers["Authorization"] = "Bearer " + key
        else:
            self.url = os.environ["ROBOUSE_GATEWAY_URL"].rstrip("/") + "/v1/chat/completions"
            self.headers["Authorization"] = "Bearer " + os.environ["ROBOUSE_GATEWAY_KEY"]
            self.dialect = "hosted"

    def complete(
        self,
        prompt: str,
        images: list[str],
        max_tokens: int,
        temperature: float,
        no_think: bool = False,
        json_schema: dict | None = None,
        choice: list[str] | None = None,
        reasoning_effort: str = "",
    ) -> dict:
        """One user turn (images first, then the text) -> {text, reasoning, usage, latency_s}."""
        content: list[dict] = [{"type": "image_url", "image_url": {"url": data_uri(p)}} for p in images]
        content.append({"type": "text", "text": prompt})
        body: dict = {
            "model": self.name,
            "messages": [{"role": "user", "content": content}],
            "max_tokens": int(max_tokens),
            "temperature": float(temperature),
        }
        if self.dialect == "vllm":
            if no_think:
                body["chat_template_kwargs"] = {"enable_thinking": False, "thinking": False}
            if json_schema is not None:
                body["guided_json"] = json_schema
                body["structured_outputs"] = {"json": json_schema}
            if choice:
                body["guided_choice"] = list(choice)
                body["structured_outputs"] = {"choice": list(choice)}
        else:
            if json_schema is not None:
                body["response_format"] = {"type": "json_object"}
            if reasoning_effort:
                body["reasoning_effort"] = reasoning_effort
        t0 = time.time()
        data = self._post(body)
        msg = (data.get("choices") or [{}])[0].get("message") or {}
        content_text = msg.get("content") or ""
        if isinstance(content_text, list):
            content_text = "".join(str(p.get("text", "")) for p in content_text if isinstance(p, dict))
        reasoning = msg.get("reasoning_content") or msg.get("reasoning") or ""
        if isinstance(reasoning, list):
            reasoning = "".join(str(p.get("text", "")) for p in reasoning if isinstance(p, dict))
        # vlm_client._message_text: the answer, else the reasoning channel when the answer came back empty
        text = content_text if str(content_text).strip() else str(reasoning or "")
        u = data.get("usage") or {}
        details = u.get("prompt_tokens_details") or {}
        return {
            "text": strip_reasoning_artifacts(str(text)),
            "content": str(content_text),
            "reasoning": str(reasoning or ""),
            "usage": {
                "prompt_tokens": int(u.get("prompt_tokens") or 0),
                "completion_tokens": int(u.get("completion_tokens") or 0),
                "cached_tokens": int((details or {}).get("cached_tokens") or 0),
            },
            "latency_s": round(time.time() - t0, 3),
        }

    def _post(self, body: dict) -> dict:
        last, delay = "", 2.0
        for attempt in range(HTTP_RETRIES + 1):
            req = urllib.request.Request(self.url, data=json.dumps(body).encode(), headers=self.headers, method="POST")
            try:
                with urllib.request.urlopen(req, timeout=180) as r:
                    data = json.loads(r.read().decode())
                if isinstance(data, dict) and data.get("choices"):
                    return data
                last = f"bad completion body: {str(data)[:300]}"
            except urllib.error.HTTPError as e:
                text = e.read().decode(errors="replace")[:600]
                if e.code == 402:
                    raise SpendCap(text) from e
                last = f"HTTP {e.code}: {text}"
                if not (e.code in (408, 409, 425, 429) or 500 <= e.code < 600):
                    raise RuntimeError(f"model call failed: {last}") from e
            except (urllib.error.URLError, TimeoutError, ConnectionError, ValueError) as e:
                last = f"{type(e).__name__}: {e}"
            if attempt < HTTP_RETRIES:
                time.sleep(delay)
                delay = min(delay * 2, 30.0)
        raise RuntimeError(f"model call failed after {HTTP_RETRIES} retries: {last}")


# ======================================================================================================================
# Parsing (core/vlm/vlm_client.py, plugins/action_chunk, core/prompting/wrist_marker.py)
# ======================================================================================================================


def strip_reasoning_artifacts(text: str) -> str:
    text = re.sub(r"<think>.*?</think>", "", str(text), flags=re.DOTALL | re.IGNORECASE)
    text = re.sub(r"<reasoning>.*?</reasoning>", "", text, flags=re.DOTALL | re.IGNORECASE)
    return text.strip()


def tokens_in_text(text: str, tokens) -> list[str]:
    return [t for t in tokens if re.search(rf"\b{re.escape(t)}\b", text)]


def recover_allowed_token(raw: str, allowed) -> str:
    tokens = tuple(allowed)
    if not tokens:
        return ""
    pat = "|".join(re.escape(t) for t in tokens)
    m = re.search(rf'"(?:token|answer|direction|status|decision)"\s*:\s*"?\s*({pat})\b', raw)
    if m:
        return m.group(1)
    for p in (
        rf"\b(?:therefore|thus|so|final(?:ly)?|choose|chosen|select|selected|use|decision|answer)\b[^.\n]*?\b({pat})\b",
        rf"\b({pat})\b[^.\n]*?\b(?:best|correct|required|appropriate)\b",
    ):
        ms = list(re.finditer(p, raw, flags=re.IGNORECASE))
        if ms:
            return ms[-1].group(1)
    sentences = [s.strip() for s in re.split(r"[.\n]+", raw) if s.strip()]
    if sentences:
        last = tokens_in_text(sentences[-1], tokens)
        if len(last) == 1:
            return last[0]
    found = list(dict.fromkeys(tokens_in_text(raw, tokens)))
    return found[0] if len(found) == 1 else ""


def parse_single_token(raw: str, allowed) -> str:
    """vlm_client._parse_single_token: the token, else ValueError."""
    stripped = strip_reasoning_artifacts(raw).strip()
    if stripped in allowed:
        return stripped
    try:
        obj = json.loads(stripped)
    except ValueError:
        obj = None
    if isinstance(obj, dict):
        for key in ("token", "answer", "direction", "status", "decision"):
            v = obj.get(key)
            if isinstance(v, str) and v.strip() in allowed:
                return v.strip()
    pat = "|".join(re.escape(t) for t in allowed)
    if re.match(rf"^(?:{pat})$", stripped):
        return stripped
    m = re.search(rf"(?:^|[^A-Z_])({pat})\s*[.。]*\s*$", stripped)
    if m:
        return m.group(1)
    rec = recover_allowed_token(stripped, allowed)
    if rec in allowed:  # the cue patterns match case-insensitively; only an exact unit counts
        return rec
    raise ValueError(f"invalid token {raw[:200]!r}; allowed tokens are {list(allowed)}")


def parse_json_object(raw: str) -> dict | None:
    text = strip_reasoning_artifacts(raw)
    try:
        v = json.loads(text)
    except ValueError:
        v = None
    if isinstance(v, dict):
        return v
    dec = json.JSONDecoder()
    cands = []
    for m in re.finditer(r"\{", text):
        try:
            c, _ = dec.raw_decode(text[m.start() :])
        except ValueError:
            continue
        if isinstance(c, dict):
            if "subgoals" in c:
                return c
            cands.append(c)
    return cands[0] if cands else None


def parse_wrist_marker(text: str) -> bool | None:
    ms = re.findall(r"WRIST\s*[:=]\s*(YES|NO)\b", str(text or ""), flags=re.IGNORECASE)
    return None if not ms else ms[-1].upper() == "YES"


def parse_plan(text: str, steps: int) -> list[str]:
    m = re.search(r"PLAN\s*[:=]\s*(.+)", str(text or ""), flags=re.IGNORECASE)
    if not m:
        return []
    moves = [t.upper() for t in re.findall(r"MV_[A-Z]+", m.group(1), flags=re.IGNORECASE)]
    return [t for t in moves if t in MOVE_ATOMS][:steps]


def one_line(text: str) -> str:
    return " ".join(str(text).split())


# ======================================================================================================================
# Subgoals (core/v0_types.py, plugins/subgoal/plugin.py)
# ======================================================================================================================


def norm_motion(value) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", str(value or "").strip()).strip("_").upper()


def subgoals_from(payload: dict) -> list[dict]:
    items = payload.get("subgoals")
    if isinstance(items, dict):
        items = [items]
    if not isinstance(items, list):
        items = [payload.get("subgoal")] if isinstance(payload.get("subgoal"), dict) else []
    out = []
    for i, it in enumerate(x for x in items if isinstance(x, dict)):
        motion = norm_motion(it.get("motion"))
        aff, desc = str(it.get("affordance") or "").strip(), str(it.get("description") or "").strip()
        if not (motion and aff and desc):
            continue
        out.append(
            {
                "id": str(it.get("id") or f"subgoal_{i}"),
                "target": str(it.get("target") or ""),
                "affordance": aff,
                "motion": motion,
                "description": desc,
                "completion": str(it.get("completion") or "").strip(),
            }
        )
    return merge_pregrasp(out)


def _text_of(sg: dict) -> str:
    return " ".join(
        str(sg.get(k, "")).lower() for k in ("id", "target", "affordance", "motion", "description", "completion")
    )


def _has(text: str, terms) -> bool:
    return any(re.search(rf"(?<![A-Za-z0-9]){re.escape(t)}(?![A-Za-z0-9])", text) for t in terms)


def _norm_target(v: str) -> str:
    return re.sub(r"\s+", " ", re.sub(r"\b(the|a|an)\b", " ", str(v or "").lower())).strip()


def merge_pregrasp(sgs: list[dict]) -> list[dict]:
    """Fold a pure reach/align stage into the grasp stage for the same target that follows it."""
    out, i = [], 0
    while i < len(sgs):
        cur = sgs[i]
        if i + 1 < len(sgs):
            nxt = sgs[i + 1]
            a, b = _norm_target(cur["target"]), _norm_target(nxt["target"])
            same = bool(a and b) and (a == b or a in b or b in a)
            reach = _has(
                _text_of(cur),
                ("approach", "reach", "align", "center", "position", "move to", "move toward", "above", "over"),
            ) and not _has(
                _text_of(cur), ("grasp", "close", "clamp", "secure", "lift", "release", "open", "place", "drop")
            )
            pivot = "REASON" in (cur["motion"], nxt["motion"])
            if same and reach and not pivot and _has(_text_of(nxt), ("grasp", "close", "clamp", "secure", "pick")):
                descs = [one_line(t).rstrip(".") for t in (cur["description"], nxt["description"]) if one_line(t)]
                out.append(
                    {**nxt, "target": nxt["target"] or cur["target"], "description": ". Then ".join(descs) + "."}
                )
                i += 2
                continue
        out.append(cur)
        i += 1
    return out


# ======================================================================================================================
# The interpreter
# ======================================================================================================================


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def quat_to_mat(q) -> list[list[float]]:
    x, y, z, w = (float(v) for v in q)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _matmul_t(a, b) -> list[list[float]]:
    """a @ b.T"""
    return [[sum(a[i][k] * b[j][k] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_to_rotvec(m) -> list[float]:
    cos = _clamp((m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0, -1.0, 1.0)
    th = math.acos(cos)
    if th < 1e-9:
        return [0.0, 0.0, 0.0]
    if math.pi - th < 1e-6:
        b = [[(m[i][j] + (1.0 if i == j else 0.0)) / 2.0 for j in range(3)] for i in range(3)]
        axis = [math.sqrt(max(b[i][i], 0.0)) for i in range(3)]
        k = max(range(3), key=lambda i: axis[i])
        v = [b[i][k] / max(axis[k], 1e-12) for i in range(3)]
        n = math.sqrt(sum(c * c for c in v)) or 1.0
        return [c / n * th for c in v]
    w = [m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]]
    s = 2.0 * math.sin(th)
    return [c / s * th for c in w]


class Arm:
    """One embodiment's interpreter over the episode socket. Positions in metres (the unit square for cubepick)."""

    def __init__(self, robot: Robot, prof: dict, info: dict):
        self.robot, self.p = robot, prof
        act = info.get("action") or {}
        self.dim = len(act.get("names") or [])
        self.low = [float(x) for x in act.get("low") or [-1.0] * self.dim]
        self.high = [float(x) for x in act.get("high") or [1.0] * self.dim]
        self.grip_cmd = -1.0  # Robo Use: +1 close, -1 open; every episode starts open
        self.origin = self.hand()
        q = self.robot.state.get("eef_quat")
        self.rot_ref = quat_to_mat(q) if prof.get("rot") and isinstance(q, list) and len(q) == 4 else None

    # -- state -----------------------------------------------------------------------------------------------------
    def hand(self) -> list[float]:
        st = self.robot.state
        h = st.get("hand_pos") or st.get("eef_pos") or [0.0, 0.0, 0.0]
        s = float(self.p.get("pos_scale_m", 1.0))  # metres per position unit (cubepick's square: 0.5)
        return [float(x) * s for x in list(h) + [0.0] * (3 - len(h))][:3]

    def width(self) -> float | None:
        v = self.robot.state.get("gripper_open")
        if self.p["width"] is None or not isinstance(v, (int, float)):
            return None
        return float(v) * 0.10 if self.p["width"] == "fraction_10cm" else float(v)

    @property
    def gripper_closed(self) -> bool:
        return self.grip_cmd > 0

    def height(self) -> float | None:
        tz = self.p.get("table_z")
        return None if tz is None else self.hand()[2] - float(tz)

    # -- control steps ---------------------------------------------------------------------------------------------
    def step(self, delta_m) -> None:
        """One control step: a translation command (metres, already capped), the gripper command, and on LIBERO the
        orientation hold back to the episode's reference."""
        if self.robot.finished:
            raise EpisodeOver(self.robot.finished)
        a = [0.0] * self.dim
        for k, idx in enumerate(self.p["pos"]):
            if idx is not None and idx < self.dim:
                a[idx] = _clamp(float(delta_m[k]) / self.p["unit_m"], self.low[idx], self.high[idx])
        if self.p.get("grip") is not None:
            g = self.p["grip"]
            a[g] = _clamp(self.grip_cmd, self.low[g], self.high[g])
        rot = self.p.get("rot")
        q = self.robot.state.get("eef_quat")
        if rot and self.rot_ref is not None and isinstance(q, list) and len(q) == 4:
            r = mat_to_rotvec(_matmul_t(self.rot_ref, quat_to_mat(q)))
            n = math.sqrt(sum(c * c for c in r))
            cap = self.p["rot_unit_rad"]
            if n > cap:
                r = [c * cap / n for c in r]
            for k, idx in enumerate(rot):
                a[idx] = _clamp(r[k] / self.p["rot_unit_rad"], self.low[idx], self.high[idx])
        resp = self.robot.send({"op": "act", "action": [round(x, 6) for x in a], "repeat": 1})
        if not resp.get("ok"):
            if self.robot.finished:
                raise EpisodeOver(self.robot.finished)
            raise RuntimeError(f"act failed: {resp.get('error')}")
        if self.robot.finished:
            raise EpisodeOver(self.robot.finished)

    def lattice_node(self, pos: list[float], step_m: float) -> list[float]:
        node = [self.origin[i] + step_m * round((pos[i] - self.origin[i]) / step_m) for i in range(3)]
        return node if max(abs(node[i] - pos[i]) for i in range(3)) <= 0.25 * step_m else list(pos)

    def move(self, token: str, step_m: float, lattice_m: float) -> dict:
        """One MV_* unit: drive toward lattice node + step, commanding the remaining error each control step."""
        d = self.p["dirs"][token]
        start = self.hand()
        target = [c + d[i] * step_m for i, c in enumerate(self.lattice_node(start, lattice_m))]
        box = self.p.get("workspace")
        for i in range(3):
            if self.p["pos"][i] is None:
                target[i] = start[i]
            elif box is not None:  # the workspace projection: a setpoint the hand cannot reach is never commanded
                target[i] = _clamp(target[i], box[0][i], box[1][i])
        n, still, prev = 0, 0, start
        for n in range(1, MAX_CTRL_STEPS + 1):
            err = [target[i] - self.hand()[i] for i in range(3)]
            if math.sqrt(sum(e * e for e in err)) < TOL_M:
                n -= 1
                break
            cap = self.p["max_cmd_m"]
            self.step([_clamp(e, -cap, cap) for e in err])
            cur = self.hand()
            still = still + 1 if math.sqrt(sum((cur[i] - prev[i]) ** 2 for i in range(3))) < QUIESCE_TOL_M else 0
            prev = cur
            if still >= STALL_STEPS:
                break
        post = self.hand()
        out = {
            "kind": "move",
            "commanded": [round(d[i] * step_m, 4) for i in range(3)],
            "moved_m": round(math.sqrt(sum((post[i] - start[i]) ** 2 for i in range(3))), 4),
            "pre": [round(x, 4) for x in start],
            "post": [round(x, 4) for x in post],
            "control_steps": n,
        }
        if still >= STALL_STEPS:
            out["stalled"] = True
        return out

    def quiesce(self) -> int:
        prev = self.hand()
        for i in range(QUIESCE_MAX_STEPS):
            self.step([0.0, 0.0, 0.0])
            cur = self.hand()
            if math.sqrt(sum((cur[k] - prev[k]) ** 2 for k in range(3))) < QUIESCE_TOL_M:
                return i + 1
            prev = cur
        return QUIESCE_MAX_STEPS

    def hold(self, n: int = 1) -> None:
        for _ in range(n):
            self.step([0.0, 0.0, 0.0])

    def gripper(self, close: bool) -> dict:
        """GRASP / RELEASE: stop the hand first, then hold the new command (a no-op when already in that state, as on
        the real arms). An empty close is reopened and flagged, as the real interpreter does."""
        if self.p.get("grip") is None:
            return {"kind": "gripper", "note": "this robot has no gripper"}
        want = 1.0 if close else -1.0
        changed = want != self.grip_cmd
        res: dict = {"kind": "gripper", "changed": changed, "grasp_empty": False}
        if changed:
            res["quiesce_steps"] = self.quiesce()
            self.grip_cmd = want
            self.hold(self.p["gripper_steps"])
        w = self.width()
        res["width_m"] = None if w is None else round(w, 4)
        if close and changed and w is not None and w <= EMPTY_WIDTH_M:
            self.grip_cmd = -1.0
            self.hold(self.p["gripper_steps"])
            res.update(grasp_empty=True, note=f"empty-close: width {w:.4f}m <= {EMPTY_WIDTH_M:.4f}m -> reopened")
        return res


# ======================================================================================================================
# Observation
# ======================================================================================================================


class Eyes:
    """The views the model gets: the task's main camera as AgentView, and its wrist camera when it has one."""

    def __init__(self, robot: Robot, prof: dict, info: dict, ws: str):
        self.robot, self.p, self.ws = robot, prof, ws
        cams = ((info.get("embodiment") or {}).get("sensors") or {}).get("cameras") or []
        self.agent_cam = (
            str(cams[0].get("name")) if cams and isinstance(cams[0], dict) and cams[0].get("name") else None
        )
        self.wrist_cam: str | None = None

    def _shot(self, camera: str | None) -> list[str]:
        resp = self.robot.send({"op": "observe", "image": True, "camera": camera})
        if not resp.get("ok"):
            if self.robot.finished:
                raise EpisodeOver(self.robot.finished)
            raise RuntimeError(f"observe failed: {resp.get('error')}")
        p = resp["result"].get("image_path")
        return [p] if isinstance(p, str) else list(p or [])

    def probe(self) -> None:
        for cam in self.p["wrist"]:
            if any(x.endswith(f"_{cam}.png") for x in self._shot(cam)):
                self.wrist_cam = cam
                return

    def look(self) -> list[str]:
        """[agentview, wrist?]: the images the model gets, in that order, each turned as the profile says."""
        out = []
        for role, cam in (("agentview", self.agent_cam), ("wrist", self.wrist_cam)):
            if role == "wrist" and cam is None:
                continue
            paths = self._shot(cam)
            path = next((x for x in paths if cam and x.endswith(f"_{cam}.png")), paths[0] if paths else None)
            if path is None:
                continue
            if role == "agentview" and self.wrist_cam and path.endswith(f"_{self.wrist_cam}.png"):
                continue  # the task's only camera is the wrist camera: it is sent once, as the wrist view
            how = self.p["flip"].get(role)
            if how:
                dst = os.path.join(self.ws, "observations", os.path.basename(path)[:-4] + "_view.png")
                flip_png(path, dst, how)
                path = dst
            out.append(path)
        return out


# ======================================================================================================================
# Zero-shot mode (core/runners/real.py, core/vlm/roles.py and the plugins)
# ======================================================================================================================


class ZeroShot:
    def __init__(self, robot, arm, eyes, model, task, prof, cfg):
        self.robot, self.arm, self.eyes, self.model = robot, arm, eyes, model
        self.task, self.p, self.cfg = task, prof, cfg
        self.plug = cfg["plugins"]
        self.has_wrist = eyes.wrist_cam is not None
        # the shared WRIST: YES/NO judgment needs a wrist view; without one the wrist rules of adaptive step and
        # chunking have no signal (adaptive step keeps its height rules)
        self.wants_wrist = self.has_wrist and (self.plug["variable_step"] or self.plug["action_chunk"])
        self.allowed = executable_units(prof)
        self.descend: tuple | None = None
        self.unsettled_holds = 0

    # -- planner -------------------------------------------------------------------------------------------------
    def plan(self) -> list[dict]:
        images = self.eyes.look()
        roles = IMAGE_ROLES[: len(images)]
        block = "IMAGE ORDER (same order as the attached images):\n" + "\n".join(
            f"{i}. {r}" for i, r in enumerate(roles, 1)
        )
        prompt = "\n\n".join(
            x.strip() for x in (block, PLANNER_PROMPT.format(task=self.task, video_ref="")) if x and x.strip()
        )
        calls, subgoals, route = [], [], "task_fallback"
        attempts = (
            ("guided_json", prompt, True),
            (
                "free_json",
                prompt
                + "\n\nReturn only the JSON object in the format shown above. Do not include thought, reasoning, "
                "markdown, or prose.",
                False,
            ),
            (
                "text_retry",
                prompt
                + "\n\nRETRY FORMAT: Return exactly one JSON object matching the requested subgoals format. Preserve "
                "every task and scene-specific rule above. Do not include thought, reasoning, markdown, or prose.",
                False,
            ),
        )
        for name, text, guided in attempts:
            try:
                r = self.model.complete(
                    text,
                    images,
                    self.cfg["planner_max_tokens"],
                    self.cfg["temperature"],
                    no_think=True,
                    json_schema=PLAN_SCHEMA if guided else None,
                    reasoning_effort=self.cfg.get("reasoning_effort", ""),
                )
            except RuntimeError as e:
                calls.append({"role": "planner", "route": name, "error": str(e)[:500]})
                continue
            calls.append({"role": "planner", "route": name, **_call_record(r)})
            obj = parse_json_object(r["text"]) or parse_json_object(r["reasoning"])
            subgoals = subgoals_from(obj) if obj else []
            if subgoals:
                route = name
                break
        if not subgoals:  # the runner's single whole-task stage
            subgoals = [
                {
                    "id": "task",
                    "target": self.task,
                    "affordance": "the task-relevant object or region",
                    "motion": "TASK",
                    "description": self.task,
                    "completion": f"the task is visibly complete: {self.task}",
                }
            ]
        emit(
            type="plan",
            t=now(),
            route=route,
            subgoals=subgoals,
            calls=calls,
            images=_names(images),
            requests=self.robot.take_requests(),
            steps_used=self.robot.steps_used,
        )
        return subgoals

    # -- controller prompt ---------------------------------------------------------------------------------------
    def proprio(self) -> str:
        if not self.plug["proprioception"]:
            return ""
        gap = self.arm.height()
        if gap is None:
            return ""
        fine_cm = f"{self.cfg['fine_step_m'] * 100.0:g}"
        if self.plug["variable_step"] and self.cfg["coarse_step_m"] > self.cfg["fine_step_m"]:
            steps = PROPRIO_STEP_COARSE.format(fine_cm=fine_cm, coarse_cm=f"{self.cfg['coarse_step_m'] * 100.0:g}")
        else:
            steps = PROPRIO_STEP_FINE.format(fine_cm=fine_cm)
        holding = self.gripper_state() == "CLOSED"
        hint = (
            PROPRIO_HINT_HOLDING
            if holding
            else PROPRIO_HINT_DESCEND.format(high_cm=f"{self.cfg['high_above_table_m'] * 100.0:.0f}")
        )
        block = PROPRIO_BLOCK.format(gap_cm=f"{gap * 100.0:.1f}", step_sizes=steps, hint=hint)
        if self.descend is not None:
            moved, commanded = self.descend
            if commanded > 0 and moved < commanded * 0.7:
                block += "\n" + PROPRIO_DESCEND_STALL.format(
                    moved_cm=f"{moved * 100.0:.1f}", commanded_cm=f"{commanded * 100.0:.1f}"
                )
        return block

    def gripper_state(self) -> str:
        w = self.arm.width()
        if w is None:
            return "n/a"
        return "CLOSED" if w < CLOSED_BELOW_M else "OPEN"

    def render(self, sg: dict, recent: list[str], note: str) -> str:
        output_contract = (
            "Choose exactly one action:\n"
            + ", ".join(self.allowed)
            + '\nReturn JSON only: {"decision":"ONE_ACTION","reasoning":"one visual sentence"}'
        )
        return CONTROLLER_PROMPT.format(
            task=self.task,
            stage=sg["motion"],
            target=sg["target"],
            affordance=sg["affordance"],
            description=sg["description"],
            completion=sg["completion"],
            gripper_state=self.gripper_state(),
            gripper_color=self.p.get("gripper_color") or "black",
            mem_text=MEM_RECENT.format(moves=", ".join(recent) or "none") if self.plug["mem_text"] else "",
            mem_text_rules=MEM_RULES if self.plug["mem_text"] else "",
            recovery=RECOVERY_CONTEXT.format(note=one_line(note)) if (note and self.plug["recovery"]) else "",
            proprio=self.proprio(),
            rotation="",
            variable_step=WRIST_MARKER if self.wants_wrist else "",
            action_chunk=(
                CHUNK_PLAN.format(step_num=self.cfg["chunk_steps"])
                if (self.plug["action_chunk"] and self.has_wrist)
                else ""
            ),
            output_contract=output_contract,
        ).strip()

    def decide(self, prompt: str, images: list[str], previous: str | None, calls: list[dict]) -> tuple[str, str, dict]:
        """roles._complete_cot_decision: reason, then commit; else a token-only retry, a strict retry, and the
        commit fallback (the last direction, else MV_DOWN). Each model call is appended to `calls` as it returns."""
        r = self.model.complete(
            prompt,
            images,
            max(1024, int(self.cfg["max_tokens"])),
            self.cfg["temperature"],
            reasoning_effort=self.cfg.get("reasoning_effort", ""),
        )
        calls.append({"role": "controller", **_call_record(r)})
        raw = r["text"]
        token = recover_allowed_token(raw, self.allowed)
        # the cot path's "reasoning" is the whole reply on one line; the WRIST marker and PLAN are read from it
        reasoning = one_line(raw)
        source = "model"
        if token not in self.allowed:  # nothing recovered, or a case-insensitive cue match ("grasp"): retry
            token, source = self._token_retry(prompt, images, previous, calls)
        extra = {
            "target_in_wrist": parse_wrist_marker(reasoning) if self.wants_wrist else None,
            "plan": [],
            "source": source,
        }
        if self.plug["action_chunk"] and self.has_wrist and extra["target_in_wrist"] is False:
            extra["plan"] = parse_plan(reasoning, self.cfg["chunk_steps"])
        return token, reasoning, extra

    def _token_retry(self, prompt: str, images: list[str], previous: str | None, calls: list[dict]) -> tuple[str, str]:
        clean = "\n".join(line for line in prompt.splitlines() if not line.strip().startswith("Return JSON only"))
        token_prompt = (
            "Answer with exactly one token from this list and nothing else:\n"
            + " ".join(self.allowed)
            + "\n\nUse the task, stage, image, and rules below only to choose that token.\n\n"
            + clean.strip()
            + "\n\nFinal answer: exactly one allowed token, no JSON, no markdown, no prose."
        )
        strict = (
            token_prompt
            + "\n\nCritical output format: return exactly one token from this list and nothing else. Do not include "
            "thought, reasoning, markdown, punctuation, or prose.\n" + " ".join(self.allowed)
        )
        for role, text in (("token_retry", token_prompt), ("strict_retry", strict)):
            r = self.model.complete(
                text, images, int(self.cfg["max_tokens"]), 0.0, no_think=True, choice=list(self.allowed)
            )
            calls.append({"role": role, **_call_record(r)})
            try:
                return parse_single_token(r["text"], self.allowed), role
            except ValueError:
                continue
        fb = previous if previous in MOVE_ATOMS and previous in self.allowed else "MV_DOWN"
        if fb not in self.allowed:
            fb = next(t for t in self.allowed if t in MOVE_ATOMS)
        return fb, "fallback"

    # -- plugins ------------------------------------------------------------------------------------------------
    def step_m(self, token: str, target_in_wrist: bool | None) -> float:
        fine, coarse = self.cfg["fine_step_m"], self.cfg["coarse_step_m"]
        if token == "MV_UP":
            return self.cfg["up_step_m"]
        if not self.plug["variable_step"]:
            return fine
        gap = self.arm.height()
        if gap is not None and gap > self.cfg["high_above_table_m"]:
            return coarse
        if target_in_wrist is False:
            return coarse
        return fine

    def width_phase(self) -> str:
        w = self.arm.width()
        if w is None:
            return "unknown"
        if w <= EMPTY_WIDTH_M:
            return "empty"
        if w >= OPEN_WIDTH_M:
            return "open"
        return "holding"

    def recovery_before(self, idx: int, sgs: list[dict]) -> dict | None:
        if not self.plug["recovery"] or not self.arm.gripper_closed:
            self.unsettled_holds = 0
            return None
        phase = self.width_phase()
        if phase == "empty":
            self.unsettled_holds = 0
            rb = _nearest_grasp(sgs, idx)
            if rb is None:
                return None
            event = "empty_grasp" if sgs[idx]["motion"] == "GRASP" else "lost_grasp"
            return {
                "event": event,
                "token": RELEASE,
                "rollback": rb,
                "block_done": True,
                "reset": True,
                "grasp_empty": True,
                "note": RECOVERY_NOTES[event],
            }
        if phase != "open":
            self.unsettled_holds = 0
            return None
        if self.unsettled_holds >= 3:
            self.unsettled_holds = 0
            return None
        self.unsettled_holds += 1
        return {"event": "grasp_unsettled", "token": STOP, "block_done": True, "note": RECOVERY_NOTES["unsettled"]}

    def recovery_after(self, token: str, result: dict, idx: int, sgs: list[dict], done: bool) -> dict | None:
        if not self.plug["recovery"]:
            return None
        rb = _nearest_grasp(sgs, idx)
        if rb is None:
            return None
        if result.get("grasp_empty"):
            return {
                "event": "empty_grasp",
                "rollback": rb,
                "block_done": True,
                "reset": True,
                "grasp_empty": True,
                "note": RECOVERY_NOTES["empty_grasp"],
            }
        phase = self.width_phase()
        if sgs[idx]["motion"] == "GRASP" and done and phase != "holding":
            return {
                "event": "unverified_grasp",
                "release": self.arm.gripper_closed and phase == "empty",
                "rollback": idx,
                "block_done": True,
                "reset": True,
                "grasp_empty": phase == "empty",
                "note": RECOVERY_NOTES["empty_grasp" if phase == "empty" else "unverified_grasp"],
            }
        if sgs[idx]["motion"] != "GRASP" and self.arm.gripper_closed and phase == "empty":
            return {
                "event": "lost_grasp",
                "release": True,
                "rollback": rb,
                "block_done": True,
                "reset": True,
                "grasp_empty": True,
                "note": RECOVERY_NOTES["lost_grasp"],
            }
        return None

    # -- the loop ------------------------------------------------------------------------------------------------
    def run(self) -> str:
        sgs = self.plan() if self.plug["subgoal"] else [_whole_task(self.task)]
        if not self.plug["subgoal"]:
            emit(
                type="plan",
                t=now(),
                route="subgoal plugin off",
                subgoals=sgs,
                calls=[],
                images=[],
                requests=self.robot.take_requests(),
                steps_used=self.robot.steps_used,
            )
        idx, stage_start, recent, chunk, note = 0, 0, [], [], ""
        previous: str | None = None
        for i in range(int(self.cfg["max_decisions"])):
            if self.robot.finished:
                return "episode finished"
            sg = sgs[idx]
            if i - stage_start >= int(self.cfg["max_stage_decisions"]):
                idx += 1
                if idx >= len(sgs):
                    return "subgoal_step_cap_exceeded"
                sg, stage_start, previous, chunk = sgs[idx], i, None, []
                recent.clear()
            t0 = now()
            rec = self.recovery_before(idx, sgs)
            calls: list[dict] = []
            reasoning, images, wrist, source, token = "", [], None, "", None
            try:
                if rec is not None and rec.get("token"):
                    token, chunk, source = rec["token"], [], "recovery"
                elif chunk:
                    token, source, wrist = chunk.pop(0), "chunk", False
                else:
                    images = self.eyes.look()
                    prompt = self.render(sg, recent, note)
                    token, reasoning, extra = self.decide(prompt, images, previous, calls)
                    wrist, source = extra["target_in_wrist"], extra["source"]
                    if extra["plan"]:
                        token, chunk = extra["plan"][0], list(extra["plan"][1:])
                result = self.execute(token, wrist)
            except (EpisodeOver, SpendCap, RuntimeError) as e:
                # the decision's record (and its model usage) is kept whatever ended it
                emit(
                    type="decision",
                    i=i,
                    t=t0,
                    mode="zs",
                    stage=idx,
                    stage_motion=sg["motion"],
                    unit=token,
                    source=source,
                    target_in_wrist=wrist,
                    reasoning=clip_text(reasoning),
                    calls=calls,
                    images=_names(images),
                    result={"interrupted": f"{type(e).__name__}: {e}"},
                    recovery=rec,
                    requests=self.robot.take_requests(),
                    steps_used=self.robot.steps_used,
                    episode=self.robot.finished or None,
                )
                raise
            done = token == DONE
            post = self.recovery_after(token, result, idx, sgs, done)
            if post is not None:
                rec = post
            if rec is not None and rec.get("release"):
                result["recovery_release"] = self.arm.gripper(False)
            if rec is not None and rec.get("block_done"):
                done = False
            if token == GRASP and self.arm.gripper_closed and not result.get("grasp_empty"):
                note = ""
            if rec is not None and rec.get("note"):
                note = rec["note"]
            complete = done and idx + 1 >= len(sgs)
            if rec is not None and rec.get("reset"):
                previous = None
                recent.clear()
            elif token in MOVE_ATOMS:
                recent.insert(0, token)
                del recent[int(self.cfg["history"]) :]
                previous = token
            elif token == GRASP:
                recent.insert(0, GRASP)
                del recent[int(self.cfg["history"]) :]
                previous = None
            elif token in (GRASP, RELEASE) or done:
                previous = None
            emit(
                type="decision",
                i=i,
                t=t0,
                mode="zs",
                stage=idx,
                stage_motion=sg["motion"],
                unit=token,
                source=source,
                target_in_wrist=wrist,
                chunk_left=list(chunk),
                reasoning=clip_text(reasoning),
                calls=calls,
                images=_names(images),
                result=result,
                recovery=rec,
                stage_done=done,
                plan_complete=complete,
                requests=self.robot.take_requests(),
                steps_used=self.robot.steps_used,
                episode=self.robot.finished or None,
            )
            if complete:
                return "plan_complete"
            if rec is not None and rec.get("rollback") is not None:
                idx, stage_start, previous, chunk = int(rec["rollback"]), i + 1, None, []
                recent.clear()
                if rec.get("grasp_empty"):
                    recent.insert(0, EMPTY_GRASP_LABEL)
                continue
            if done:
                if sg["motion"] == "GRASP":
                    note = ""
                idx, stage_start, previous, chunk = idx + 1, i + 1, None, []
                recent.clear()
        return "max_steps_exceeded"

    def execute(self, token: str, target_in_wrist: bool | None) -> dict:
        if token in MOVE_ATOMS and token in self.p["dirs"]:
            step = self.step_m(token, target_in_wrist)
            res = self.arm.move(token, step, self.cfg["fine_step_m"])
            res["step_m"] = step
            if token == "MV_DOWN":
                self.descend = (max(0.0, res["pre"][2] - res["post"][2]), step)
            else:
                self.descend = None
            return res
        if token == GRASP:
            return self.arm.gripper(True)
        if token == RELEASE:
            return self.arm.gripper(False)
        if token == STOP:
            self.arm.hold(1)
            return {"kind": "hold"}
        return {"kind": "done" if token == DONE else "none"}


PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "subgoals": {
            "type": "array",
            "minItems": 1,
            "items": {
                "type": "object",
                "properties": {
                    k: {"type": "string"} for k in ("id", "target", "affordance", "motion", "description", "completion")
                },
                "required": ["id", "target", "affordance", "motion", "description", "completion"],
                "additionalProperties": False,
            },
        }
    },
    "required": ["subgoals"],
    "additionalProperties": False,
}


def _whole_task(task: str) -> dict:
    return {
        "id": "task",
        "target": task,
        "affordance": "the task-relevant object or region",
        "motion": "TASK",
        "description": task,
        "completion": f"the task is visibly complete: {task}",
    }


def _nearest_grasp(sgs: list[dict], idx: int) -> int | None:
    for k in range(min(max(idx, 0), len(sgs) - 1), -1, -1):
        if sgs[k]["motion"] == "GRASP":
            return k
    return None


def _names(paths: list[str]) -> list[str]:
    return [os.path.basename(p) for p in paths]


def _call_record(r: dict) -> dict:
    return {
        "content": clip_text(r.get("content", "")),
        "reasoning": clip_text(r.get("reasoning", "")),
        "usage": r.get("usage"),
        "latency_s": r.get("latency_s"),
    }


# ======================================================================================================================
# Fine-tuned mode (core/vlm/mvtoken_roles.py, scripts/run_libero_mvtoken.py)
# ======================================================================================================================


class FineTuned:
    def __init__(self, robot, arm, eyes, model, task, prof, cfg):
        self.robot, self.arm, self.eyes, self.model = robot, arm, eyes, model
        self.task, self.p, self.cfg = task, prof, cfg
        self.allowed = tuple(t for t in CONTROLLER_TOKENS if re.search(rf"\b{t}\b", FT_PROMPT))

    def run(self) -> str:
        recent: list[str] = []
        no_move: dict[str, int] = {}
        vllm = self.model.dialect == "vllm"
        # a served adapter answers in a few tokens; a hosted reasoning model needs room to think first
        budget = int(self.cfg["max_tokens"]) if vllm else max(1024, int(self.cfg["max_tokens"]))
        for i in range(int(self.cfg["max_decisions"])):
            if self.robot.finished:
                return "episode finished"
            t0 = now()
            calls: list[dict] = []
            images: list[str] = []
            token, blocked, result = None, [], {}
            try:
                images = self.eyes.look()
                if len(images) == 1:
                    images = images * 2  # the adapters take two views; a single-camera task fills both slots
                blocked = sorted(t for t, c in no_move.items() if c >= int(self.cfg["stuck_repeat"]))
                prompt = FT_PROMPT.format(task=self.task, recent_moves=", ".join(recent) or "none")
                # the stuck guard narrows the choice by guided decoding, which only a vLLM server offers
                choice = [t for t in self.allowed if t not in blocked] if (blocked and vllm) else None
                r = self.model.complete(prompt, images, budget, self.cfg["temperature"], no_think=True, choice=choice)
                calls.append({"role": "ft", **_call_record(r)})
                try:
                    token = parse_single_token(r["text"], choice or self.allowed)
                except ValueError as e:
                    emit(
                        type="decision",
                        i=i,
                        t=t0,
                        mode="ft",
                        unit=None,
                        error=str(e)[:300],
                        calls=calls,
                        images=_names(images),
                        blocked=blocked,
                        requests=self.robot.take_requests(),
                        steps_used=self.robot.steps_used,
                    )
                    return "invalid_token"
                result = {"kind": "done"}
                if token in MOVE_ATOMS and token in self.p["dirs"]:
                    result = self.arm.move(token, self.cfg["fine_step_m"], self.cfg["fine_step_m"])
                    result["step_m"] = self.cfg["fine_step_m"]  # the fine-tuned loop uses one step size
                    if result["moved_m"] * 1000 < float(self.cfg["stuck_guard_mm"]):
                        no_move[token] = no_move.get(token, 0) + 1
                    else:
                        no_move.clear()
                elif token == GRASP:
                    result = self.arm.gripper(True)
                elif token == RELEASE:
                    result = self.arm.gripper(False)
                elif token != DONE:
                    result = {"kind": "none", "note": f"{token} does nothing on this robot"}
                    if token in MOVE_ATOMS:  # it moved the hand 0 mm: the stuck guard counts it like a blocked move
                        no_move[token] = no_move.get(token, 0) + 1
            except (EpisodeOver, SpendCap, RuntimeError) as e:
                emit(
                    type="decision",
                    i=i,
                    t=t0,
                    mode="ft",
                    unit=token,
                    source="model",
                    calls=calls,
                    images=_names(images),
                    blocked=blocked,
                    result={"interrupted": f"{type(e).__name__}: {e}"},
                    requests=self.robot.take_requests(),
                    steps_used=self.robot.steps_used,
                    episode=self.robot.finished or None,
                )
                raise
            if token in MOVE_ATOMS:
                recent.insert(0, token)
                del recent[int(self.cfg["history"]) :]
            emit(
                type="decision",
                i=i,
                t=t0,
                mode="ft",
                unit=token,
                source="model",
                calls=calls,
                images=_names(images),
                blocked=blocked,
                result=result,
                requests=self.robot.take_requests(),
                steps_used=self.robot.steps_used,
                episode=self.robot.finished or None,
            )
            if token == DONE:
                return "done"
        return "max_steps_exceeded"


# ======================================================================================================================
# main
# ======================================================================================================================


# Only the model's own DONE claims success: zero-shot, the DONE that completes the plan's last stage; fine-tuned, DONE.
# Any other stop (the decision cap, a stage cap, a reply that is not a unit) ends without a claim: the client exits and
# the runner closes the episode with its own give-up ("[runner] agent exited without robo done"), as for any harness
# that stops without `robo done`, and the reason is in the `end` event. The client does not send give-up itself: on a
# task that rewards a refusal (`safety: refuse`), an agent's give-up counts as one.
CLAIMS = ("plan_complete", "done")


def settings() -> dict:
    s = json.loads(os.environ.get("ROBOUSE_SHOW_HARNESS_SETTINGS") or "{}")
    mode = s.get("mode") or ("ft" if os.environ.get("ROBOUSE_SHOW_HARNESS_URL") else "zs")
    if mode not in DEFAULTS:
        raise ValueError(f"unknown show-harness mode {mode!r}; modes: zs, ft")
    cfg = json.loads(json.dumps(DEFAULTS[mode]))
    for k, v in (s.get("overrides") or {}).items():
        if k == "plugins" and isinstance(v, dict) and "plugins" in cfg:
            cfg["plugins"].update({p: bool(x) for p, x in v.items() if p in cfg["plugins"]})
        elif k in cfg or k in ("reasoning_effort", "token_header"):
            cfg[k] = v
    cfg["mode"], cfg["task_sentence"] = mode, str(s.get("task_sentence") or "")
    return cfg


def main(argv: list[str]) -> int:
    body = argv[1] if len(argv) > 1 else ""
    robot = Robot(os.environ["ROBOUSE_SOCKET"])
    try:
        cfg = settings()
        model = Model(str(cfg.get("token_header") or ""))
    except (ValueError, KeyError) as e:
        emit(type="error", t=now(), error=f"{type(e).__name__}: {e}")
        return 2
    info_resp = robot.send({"op": "info"})
    if not info_resp.get("ok"):
        emit(type="error", t=now(), error=f"info failed: {info_resp.get('error')}", requests=robot.take_requests())
        return 1
    info = info_resp["result"]
    name, why = pick_profile(info)
    if name is None:
        emit(type="error", t=now(), error=why, requests=robot.take_requests())
        return 2
    prof = PROFILES[name]
    robot.send({"op": "observe"})  # the starting state (hand position, orientation)
    arm = Arm(robot, prof, info)
    eyes = Eyes(robot, prof, info, os.getcwd())
    eyes.probe()
    fine = cfg["fine_step_m"]
    coarse = cfg.get("coarse_step_m", fine)
    conventions = prof["conventions"].format(fine_cm=f"{fine * 100:g}", coarse_cm=f"{coarse * 100:g}")
    if cfg["mode"] == "zs":
        task = ADAPTER.format(conventions=conventions) + "\n\n" + body.strip()
        runner: ZeroShot | FineTuned = ZeroShot(robot, arm, eyes, model, task, prof, cfg)
    else:
        task = cfg["task_sentence"] or body.strip()
        runner = FineTuned(robot, arm, eyes, model, task, prof, cfg)
    emit(
        type="start",
        t=now(),
        mode=cfg["mode"],
        model=model.name,
        dialect=model.dialect,
        profile=name,
        backend=info.get("backend"),
        units=list(runner.allowed),
        max_steps=info.get("max_steps"),
        agentview=eyes.agent_cam,
        wrist=eyes.wrist_cam,
        settings={k: v for k, v in cfg.items() if k != "task_sentence"},
        task=clip_text(task, 20000),
        requests=robot.take_requests(),
    )
    reason = "error"
    try:
        reason = runner.run()
    except EpisodeOver as e:
        reason = f"episode ended: {e}"
    except SpendCap as e:
        emit(
            type="end",
            t=now(),
            reason="spend cap reached",
            detail=str(e)[:300],
            claimed_done=False,
            requests=robot.take_requests(),
            steps_used=robot.steps_used,
        )
        return 0
    except Exception as e:  # noqa: BLE001 - an API or interpreter failure: reported, the episode left open
        emit(
            type="error",
            t=now(),
            error=f"{type(e).__name__}: {str(e)[:1000]}",
            requests=robot.take_requests(),
            steps_used=robot.steps_used,
        )
        return 1
    claimed = False
    if not robot.finished and reason in CLAIMS:
        robot.send({"op": "done", "text": f"show-harness: {reason}"})
        claimed = True
    emit(
        type="end",
        t=now(),
        reason=reason,
        claimed_done=claimed,
        episode=robot.finished or None,
        requests=robot.take_requests(),
        steps_used=robot.steps_used,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
