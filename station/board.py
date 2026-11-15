"""The printed ChArUco board, and finding it in a picture. Shared by the solver, the picture check and the live view.

Default board: 12 x 9 squares of 30 mm, ArUco markers of 22 mm from DICT_5X5_1000, which gives 11 x 8 = 88 inner
corners. Print it with `python station/board.py --print board.png` and measure a square after printing: the solver's
distances (baseline) scale with square_m. A different board is given as JSON in the STATION_BOARD setting, e.g.
STATION_BOARD='{"squares_x": 12, "squares_y": 9, "square_m": 0.030, "marker_m": 0.022, "dictionary": "DICT_5X5_1000"}'.

OpenCV 4.6 changed how ChArUco boards with an even number of rows are laid out. A board printed by an older tool with
an even row count needs "legacy": true; the default 12 x 9 board has an odd row count and is the same either way.
"""
import json
import os

import cv2
import numpy as np

BOARD = {"squares_x": 12, "squares_y": 9, "square_m": 0.030, "marker_m": 0.022, "dictionary": "DICT_5X5_1000", "legacy": False}
if os.environ.get("STATION_BOARD"):
    BOARD = {**BOARD, **json.loads(os.environ["STATION_BOARD"])}
CORNERS = (BOARD["squares_x"] - 1) * (BOARD["squares_y"] - 1)      # inner corners on the whole board (88 by default)
SHARP_MIN = 250            # sharpness score (variance of the Laplacian, whole lens picture) under which a picture is soft


def make_board(b=BOARD):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, b["dictionary"]))
    board = cv2.aruco.CharucoBoard((b["squares_x"], b["squares_y"]), b["square_m"], b["marker_m"], d)
    if b.get("legacy"):
        board.setLegacyPattern(True)
    return board


def make_detector(b=BOARD):
    """A ChArUco detector for this board. Not shared between threads: make one per thread."""
    board = make_board(b)
    return cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), cv2.aruco.DetectorParameters())


def detect(det, gray, min_corners=4):
    """(ids (n,), corners (n, 2) float32) of the board's inner corners found in one lens picture, or None."""
    corners, ids, _, _ = det.detectBoard(gray)
    if ids is None or len(ids) < min_corners:
        return None
    return ids.reshape(-1).astype(np.int32), corners.reshape(-1, 2).astype(np.float32)


def object_points(det, ids):
    """Board coordinates (metres, z = 0) of the given corner ids."""
    return det.getBoard().getChessboardCorners()[ids].astype(np.float32)


def sharpness(gray):
    """Variance of the Laplacian: high for a sharp, well lit picture, low for a soft, blurred or dark one."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


if __name__ == "__main__":
    import sys
    if len(sys.argv) == 3 and sys.argv[1] == "--print":
        # 200 pixels per square: a 2400 x 1800 picture. Print it so that one square measures square_m exactly.
        px = 200
        img = make_board().generateImage((BOARD["squares_x"] * px, BOARD["squares_y"] * px), marginSize=0, borderBits=1)
        img = cv2.copyMakeBorder(img, px // 2, px // 2, px // 2, px // 2, cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(sys.argv[2], img)
        print(f"wrote {sys.argv[2]}: {BOARD['squares_x']} x {BOARD['squares_y']} squares, print each square at "
              f"{BOARD['square_m'] * 1000:g} mm (markers {BOARD['marker_m'] * 1000:g} mm, {BOARD['dictionary']})")
    else:
        print(json.dumps({**BOARD, "inner_corners": CORNERS}))
