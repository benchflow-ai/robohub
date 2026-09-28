"""Desk mounting for both arms: side by side at the back edge, both reaching forward.

World +X is right, +Y is the rear of the desk, +Z is up, tabletop at Z = 0, metres.
Bases sit at X = +/-0.35 m, Y = 0.30 m and both point toward -Y.
"""
import math

REAR_BASE_Y = .30
BASE_X = {"left": -.35, "right": .35}
FORWARD_QUAT = (math.sqrt(.5), 0., 0., -math.sqrt(.5))
BOWL_X = {"left": -.36, "right": .36}
BOWL_Y = -.12
BOARD_Y = -.06
SQUARE_SIZE = .04
TABLE_HALF = (.8, .47)
READY_Z = .25


def board_xy(rank, file):
    return (rank - 3.5) * SQUARE_SIZE, BOARD_Y + (file - 3.5) * SQUARE_SIZE


def layout_manifest():
    return {"id": "rear_edge_side_by_side_v1",
            "world_frame": "+X right, +Y rear, +Z up; tabletop at Z=0; metres",
            "table_size_m": [1.6, .94, .75], "tabletop_thickness_m": .05,
            "mount_plates": {s: {"xy_m": [x, REAR_BASE_Y], "surface_z_m": .015}
                             for s, x in BASE_X.items()},
            "base_forward_world": [0, -1, 0],
            "bowls": {s: [x, BOWL_Y] for s, x in BOWL_X.items()},
            "board_center_xy_m": [0, BOARD_Y], "square_size_m": SQUARE_SIZE}
