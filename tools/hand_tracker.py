"""Kinect-colour hand tracking -> VMC finger bones.

Why this exists
---------------
Kinect v2's skeleton gives five joints per hand: wrist, hand, fingertip, thumb. There is no
per-finger data and no reliable palm normal, so a fist was the only gesture that could be
derived, and palm roll could not be derived at all -- every attempt at it produced a constant
90 or 180 degree error or crashed the solver.

MediaPipe hands returns 21 landmarks per hand, which fully determine the palm plane and every
finger's direction. That removes the ambiguity entirely: palm roll becomes arithmetic.

This script therefore owns HAND AND FINGER bones, and the Kinect program stays responsible for
the body. The two do not overlap, so they can run side by side without fighting.

Usage
-----
    python hand_tracker.py --list-cameras        # find which index is the Kinect colour camera
    python hand_tracker.py --camera 1            # run, preview, and send
    python hand_tracker.py --camera 1 --no-send  # preview only
    python hand_tracker.py --camera 1 --kinect   # read Kinect colour via pykinect2 (if installed)

Keys in the preview window:  q or Esc to quit,  m to toggle mirror,  d to toggle landmarks.

Dependencies (all installable on Python 3.13):
    mediapipe>=1.1.0, opencv-python, python-osc
"""

import argparse
import math
import socket
import struct
import sys
import time
import json
import os
import urllib.request

import cv2
import numpy as np

# OpenCV logs verbosely while probing virtual cameras; only our own lines are useful here.
try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:
    pass

try:
    import mediapipe as mp
except ImportError:
    sys.exit("mediapipe is missing. Run:  python -m pip install mediapipe")


# --------------------------------------------------------------------------------------
# MediaPipe model
#
# mediapipe 1.1.0 REMOVED the old `mp.solutions.hands` API and now exposes only the Tasks API,
# which needs a .task model bundle. An earlier version of this script used the removed API and
# crashed on startup with "module 'mediapipe' has no attribute 'solutions'".
#
# The bundle is fetched on first use, so a fresh checkout needs no manual download step.
# --------------------------------------------------------------------------------------
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/1/hand_landmarker.task")

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "models", "hand_landmarker.task")


def ensure_model(path=MODEL_PATH, on_progress=None):
    """Returns a usable model path, downloading the bundle if it is not present yet.

    `on_progress` is called with (received_bytes, total_bytes) while downloading, so a caller can
    put something on screen. Without it the process looks hung for the length of the download --
    and on a fresh copy of the folder that is exactly what happens on first launch, because the
    model is not cached yet.
    """
    if os.path.isfile(path) and os.path.getsize(path) > 100000:
        return path

    os.makedirs(os.path.dirname(path), exist_ok=True)
    print("Downloading the MediaPipe hand model (about 8 MB, once)...")
    print("  " + MODEL_URL)

    try:
        response = urllib.request.urlopen(MODEL_URL, timeout=30)
        total = int(response.headers.get("Content-Length") or 0)

        received = 0
        chunk_size = 64 * 1024
        with open(path, "wb") as handle:
            while True:
                chunk = response.read(chunk_size)
                if not chunk:
                    break
                handle.write(chunk)
                received += len(chunk)
                if on_progress is not None:
                    try:
                        on_progress(received, total)
                    except Exception:
                        # A failing progress display must never stop the download.
                        on_progress = None
    except Exception as exc:
        try:
            if os.path.isfile(path):
                os.remove(path)
        except Exception:
            pass
        sys.exit("Could not download the model: {}\n"
                 "Download it manually and save it as:\n  {}".format(exc, path))

    print("  saved {} bytes to {}".format(os.path.getsize(path), path))
    return path


def make_landmarker(num_hands, min_detection, min_presence, min_tracking, on_progress=None):
    """Builds a HandLandmarker through the Tasks API."""
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision

    options = vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=ensure_model(on_progress=on_progress)),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=num_hands,
        min_hand_detection_confidence=min_detection,
        min_hand_presence_confidence=min_presence,
        min_tracking_confidence=min_tracking,
    )
    return vision.HandLandmarker.create_from_options(options)


class HandResult(object):
    """One detected hand, normalised so the rest of the script need not care about the API."""

    def __init__(self, points, label, world=None):
        self.points = points     # 21 (x, y, z) in normalised image coordinates, for drawing
        self.world = world       # 21 (x, y, z) in METRIC hand coordinates, for geometry
        self.label = label       # "Left" or "Right" as reported by MediaPipe


def detect(landmarker, mp_image, timestamp_ms):
    """Runs detection and returns a list of HandResult."""
    result = landmarker.detect_for_video(mp_image, timestamp_ms)

    hands = []
    landmarks = getattr(result, "hand_landmarks", None) or []
    world_landmarks = getattr(result, "hand_world_landmarks", None) or []
    handedness = getattr(result, "handedness", None) or []

    for index, hand in enumerate(landmarks):
        points = [(float(p.x), float(p.y), float(p.z)) for p in hand]

        # The WORLD landmarks are the ones that matter for geometry.
        #
        # MediaPipe returns two sets. `hand_landmarks` is normalised image space: x and y are
        # good but z is a relative depth whose scale varies with the hand's size and distance,
        # so with the palm facing the camera it is close to noise. `hand_world_landmarks` is a
        # metric 3D model of the hand centred on it, where every joint has a real position.
        #
        # This script originally used only the normalised set, which is why a palm-on hand could
        # not be read: the fold direction came out of depth values that carried no information.
        # Warudo's own capture works because it uses the world set.
        world = None
        if index < len(world_landmarks):
            world = [(float(p.x), float(p.y), float(p.z)) for p in world_landmarks[index]]

        label = "Right"
        if index < len(handedness):
            entries = getattr(handedness[index], "classification", None)
            if entries:
                label = str(entries[0].category_name)

        hands.append(HandResult(points, label, world))

    return hands

try:
    from pythonosc import udp_client
except ImportError:
    udp_client = None


# --------------------------------------------------------------------------------------
# VMC finger bone naming. These are Unity HumanBodyBones names, which is what VMC carries
# and what Warudo maps onto a VRM rig.
# --------------------------------------------------------------------------------------
FINGERS = ("Thumb", "Index", "Middle", "Ring", "Little")
SEGMENTS = ("Proximal", "Intermediate", "Distal")

# MediaPipe landmark indices, per finger, from the wrist outward.
# Thumb: CMC(1) MCP(2) IP(3) TIP(4);  the others: MCP, PIP, DIP, TIP.
FINGER_LANDMARKS = {
    "Thumb": (1, 2, 3, 4),
    "Index": (5, 6, 7, 8),
    "Middle": (9, 10, 11, 12),
    "Ring": (13, 14, 15, 16),
    "Little": (17, 18, 19, 20),
}

WRIST = 0
MIDDLE_MCP = 9
INDEX_MCP = 5
PINKY_MCP = 17
THUMB_CMC = 1


def bone_names(side):
    """Every VMC bone this script writes for one hand, in emit order."""
    names = []
    for finger in FINGERS:
        for segment in SEGMENTS:
            names.append("{}{}{}".format(side, finger, segment))
    return names


# --------------------------------------------------------------------------------------
# Rotation helpers. Kept deliberately small and explicit: every previous bug in this project
# came from a sign or an axis convention, so nothing here is clever.
# --------------------------------------------------------------------------------------
def vec(x, y, z):
    return np.array([float(x), float(y), float(z)], dtype=np.float64)


def normalize(v):
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return None
    return v / n


def quat_mul(a, b):
    """Hamilton product, (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=np.float64)


def quat_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def quat_from_axis_angle(axis, angle):
    axis = normalize(axis)
    if axis is None:
        return np.array([0.0, 0.0, 0.0, 1.0])
    half = angle * 0.5
    s = math.sin(half)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half)], dtype=np.float64)


def quat_between(a, b):
    """Shortest-arc rotation carrying unit vector a onto unit vector b."""
    a = normalize(a)
    b = normalize(b)
    if a is None or b is None:
        return np.array([0.0, 0.0, 0.0, 1.0])

    dot = float(np.dot(a, b))
    if dot > 1.0:
        dot = 1.0
    if dot < -1.0:
        dot = -1.0

    if dot > 0.999999:
        return np.array([0.0, 0.0, 0.0, 1.0])

    if dot < -0.999999:
        # Opposite directions: any perpendicular axis is a valid 180 degree turn. Pick one that
        # is stable rather than letting it flip between frames.
        helper = vec(1.0, 0.0, 0.0)
        if abs(float(np.dot(a, helper))) > 0.9:
            helper = vec(0.0, 1.0, 0.0)
        axis = normalize(np.cross(a, helper))
        return quat_from_axis_angle(axis, math.pi)

    axis = np.cross(a, b)
    angle = math.acos(dot)
    return quat_from_axis_angle(axis, angle)


def rotate(q, v):
    """Apply rotation q to vector v, (x, y, z, w) convention."""
    qv = np.array([q[0], q[1], q[2]], dtype=np.float64)
    w = q[3]
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def slerp(a, b, t):
    """Shortest-path interpolation between two unit quaternions."""
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        result = a + t * (b - a)
        n = float(np.linalg.norm(result))
        return result / n if n > 1e-9 else a
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    wa = math.sin((1.0 - t) * theta) / sin_theta
    wb = math.sin(t * theta) / sin_theta
    return wa * a + wb * b


def quat_to_list(q):
    return [float(q[0]), float(q[1]), float(q[2]), float(q[3])]


# --------------------------------------------------------------------------------------
# Hand frame and bone solving
# --------------------------------------------------------------------------------------
def hand_axes(points):
    """Build the hand's own frame from landmarks, in MediaPipe's coordinate space.

    MediaPipe returns x right, y DOWN, z toward the camera (smaller z is nearer). Working in
    that space is fine as long as it is used consistently; the axis conversion to VMC's
    left-handed Y-up convention happens once, at the end.

    Returns (forward, normal, side, normal_sign_report) or None.
      forward : wrist -> middle MCP, i.e. along the back of the hand
      normal  : perpendicular to the palm
      side    : across the knuckles, from the index side toward the pinky side
    """
    wrist = points[WRIST]
    middle = points[MIDDLE_MCP]
    index = points[INDEX_MCP]
    pinky = points[PINKY_MCP]

    forward = normalize(middle - wrist)
    if forward is None:
        return None

    across = normalize(pinky - index)
    if across is None:
        return None

    normal = normalize(np.cross(forward, across))
    if normal is None:
        return None

    # Re-orthogonalise so the frame is exactly right-handed.
    side = normalize(np.cross(normal, forward))
    if side is None:
        return None

    return forward, normal, side


def hand_frame_of(layout):
    """Builds the hand frame (columns: side, normal, forward) from a landmark layout."""
    forward = normalize(vec(*layout[MIDDLE_MCP]) - vec(*layout[WRIST]))
    across = normalize(vec(*layout[PINKY_MCP]) - vec(*layout[INDEX_MCP]))
    if forward is None or across is None:
        return np.eye(3, dtype=np.float64)
    normal = normalize(np.cross(forward, across))
    if normal is None:
        return np.eye(3, dtype=np.float64)
    side = normalize(np.cross(normal, forward))
    if side is None:
        return np.eye(3, dtype=np.float64)
    return np.array([[side[0], normal[0], forward[0]],
                     [side[1], normal[1], forward[1]],
                     [side[2], normal[2], forward[2]]], dtype=np.float64)

def canonical_finger_axis():
    """The finger axis in the canonical hand frame: fingers point along +x there."""
    return vec(1.0, 0.0, 0.0)

def canonical_open_hand():
    """A canonical OPEN hand in hand-local coordinates, used as the rest reference.

    This must NOT be derived from the hand currently being measured. Doing that makes each
    finger's rest direction equal to its own measured direction, so the proximal joint reads
    zero rotation no matter how the hand moves -- the test suite caught exactly that.

    The layout is the MediaPipe topology in a simplified, hand-local frame:
      * fingers run along +x,
      * the knuckle line runs along y, index at -y and little at +y (as MediaPipe orders them),
      * the palm normal is +z.
    Coordinates are arbitrary units; only directions matter.
    """
    pts = [None] * 21
    pts[WRIST] = (0.0, 0.0, 0.0)

    knuckle_x = 1.0
    for finger, index in (("Index", 5), ("Middle", 9), ("Ring", 13), ("Little", 17)):
        pts[index] = (knuckle_x, KNUCKLE_SPREAD[finger], 0.0)

    # The thumb leaves the palm at an angle, which is why it needs its own rest direction.
    pts[THUMB_CMC] = (0.35, -0.45, 0.0)

    for finger, (base, mcp, pip, tip) in FINGER_LANDMARKS.items():
        x, y, z = pts[base]
        for target in (mcp, pip, tip):
            x += FINGER_BONE
            pts[target] = (x, y, z)

    return pts


KNUCKLE_SPREAD = {"Index": -0.55, "Middle": -0.18, "Ring": 0.18, "Little": 0.55}
FINGER_BONE = 0.4


def rest_directions(points=None):
    """Per-finger rest directions, taken from the canonical open hand.

    Each finger gets its OWN direction, because the fingers of an open hand are not parallel:
    they fan out from the wrist by roughly ten degrees each. Forcing them all onto the hand's
    forward axis leaves a constant offset on every bone -- the same class of mistake as the
    T-pose-versus-A-pose error that bent the arm earlier in this project.

    `points` is accepted for backwards compatibility with the tests but ignored; the reference
    is deliberately independent of whatever hand happens to be in front of the camera.
    """
    canonical = canonical_open_hand()

    forward = normalize(vec(*canonical[MIDDLE_MCP]) - vec(*canonical[WRIST]))
    rest = {}
    for finger in FINGERS:
        base, mcp, _pip, _tip = FINGER_LANDMARKS[finger]
        direction = normalize(vec(*canonical[mcp]) - vec(*canonical[base]))
        rest[finger] = direction if direction is not None else forward
    return rest


# --------------------------------------------------------------------------------------
# Gesture classification
#
# The user's point, and it is the right one: the model already HAS valid hand poses -- its own
# rig can make a fist and an open hand. Trying to reconstruct 15 finger joint angles from 21
# monocular landmarks is an ill-posed problem, and the attempts kept failing in ways that were
# only visible on the model. Detecting a GESTURE and blending the model's own two poses avoids
# the mapping entirely: the avatar can then never show a pose it is not already capable of.
#
# Everything here uses only x and y. MediaPipe's depth is unreliable -- worst exactly when the
# palm faces the camera -- and the gesture question ("how curled is this finger?") is answerable
# in the image plane alone.
# --------------------------------------------------------------------------------------
FINGER_JOINTS = {
    "Thumb": (1, 2, 3, 4),
    "Index": (5, 6, 7, 8),
    "Middle": (9, 10, 11, 12),
    "Ring": (13, 14, 15, 16),
    "Little": (17, 18, 19, 20),
}


def _angle_2d(a, b, c):
    """Interior angle at b, in the image plane, in degrees. 180 = perfectly straight."""
    ba = np.array([a[0] - b[0], a[1] - b[1]], dtype=np.float64)
    bc = np.array([c[0] - b[0], c[1] - b[1]], dtype=np.float64)
    na, nc = np.linalg.norm(ba), np.linalg.norm(bc)
    if na < 1e-9 or nc < 1e-9:
        return 180.0
    cos = float(np.dot(ba, bc)) / (na * nc)
    cos = max(-1.0, min(1.0, cos))
    return math.degrees(math.acos(cos))


def _angle_3d(a, b, c):
    """Interior angle at b, measured in 3D, in degrees. 180 = perfectly straight."""
    ba = np.array([a[0] - b[0], a[1] - b[1], a[2] - b[2]], dtype=np.float64)
    bc = np.array([c[0] - b[0], c[1] - b[1], c[2] - b[2]], dtype=np.float64)
    na, nc = np.linalg.norm(ba), np.linalg.norm(bc)
    if na < 1e-9 or nc < 1e-9:
        return 180.0
    cos = float(np.dot(ba, bc)) / (na * nc)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def finger_curl(points, finger, use_3d=False):
    """How curled one finger is: 0 = straight, 1 = fully folded.

    use_3d=True measures the true joint angles from the metric world landmarks, which is the
    right choice when they are available. use_3d=False falls back to the image plane, which needs
    only x and y and is therefore immune to MediaPipe's depth noise -- but it cannot see a finger
    that folds straight towards the camera. That limitation is exactly why 3D is preferred now.
    """
    _base, mcp, pip, tip = FINGER_JOINTS[finger]
    measure = _angle_3d if use_3d else _angle_2d

    # Two interior angles along the finger; both are near 180 when the finger is straight.
    a1 = measure(points[mcp], points[pip], points[tip])
    a2 = measure(points[FINGER_JOINTS[finger][0]], points[mcp], points[pip])

    # Map 180..90 degrees onto 0..1. Below 90 degrees the finger is folded as far as it goes.
    def norm(angle):
        return max(0.0, min(1.0, (180.0 - angle) / 90.0))

    return 0.5 * (norm(a1) + norm(a2))


def hand_gesture(points, use_3d=False):
    """Classifies the hand from the image plane alone.

    Returns a dict with per-finger curl values and an overall "closed" value in 0..1, which is
    all a two-pose blend needs. The thumb is measured against the index knuckle rather than
    against its own joints, because a thumb has a very different geometry from the fingers.
    """
    curls = {}
    for finger in ("Index", "Middle", "Ring", "Little"):
        curls[finger] = finger_curl(points, finger, use_3d)

    # Thumb: how close its tip is to the index knuckle, scaled by hand size.
    thumb_tip = points[FINGER_JOINTS["Thumb"][3]]
    index_knuckle = points[FINGER_JOINTS["Index"][1]]
    wrist = points[WRIST]
    hand_span = float(np.linalg.norm(
        np.array([points[MIDDLE_MCP][0] - wrist[0], points[MIDDLE_MCP][1] - wrist[1]])))
    if hand_span < 1e-9:
        curls["Thumb"] = 0.0
    else:
        reach = float(np.linalg.norm(
            np.array([thumb_tip[0] - index_knuckle[0], thumb_tip[1] - index_knuckle[1]])))
        curls["Thumb"] = max(0.0, min(1.0, 1.0 - (reach / hand_span)))

    four = [curls[f] for f in ("Index", "Middle", "Ring", "Little")]
    closed = sum(four) / len(four)

    return curls, closed

def to_vector(p):
    """Accepts a landmark object (x/y/z attributes) or a plain (x, y, z) sequence."""
    if isinstance(p, np.ndarray):
        return p.astype(np.float64)
    if hasattr(p, "x") and hasattr(p, "y") and hasattr(p, "z"):
        return vec(p.x, p.y, p.z)
    return vec(p[0], p[1], p[2])

def solve_hand(points, side_name, mirror_x, rest=None, flip_palm=False):
    """Convert 21 landmarks into per-bone LOCAL rotations for the VMC finger chain.

    Design, and why the earlier attempts failed
    -------------------------------------------
    VMC finger bones are LOCAL: each carries the finger's rotation RELATIVE TO THE HAND. The
    hand itself is a separate bone, and on this setup it is driven by the Kinect body tracker,
    which the user confirmed is accurate for the wrist.

    Earlier versions of this function also carried the hand's own orientation -- the palm's
    facing and the finger axis -- into the finger bones. That is a DOUBLE rotation: the hand
    bone already applies it, and the fingers applied it again, so every joint came out badly
    twisted. Worse, the shortest-arc rotation used to derive it leaves the palm's roll free, so
    the extra copy could flip the palm 180 degrees, which is exactly what the user reported
    ("the thumbs point outwards once the script takes over, but are correct when uncontrolled").

    So this function now emits ONLY the finger bend relative to the palm:
      1. build the measured hand frame from the landmarks,
      2. express each bone's measured direction in that frame -- i.e. relative to the palm,
      3. rotate the canonical (flat hand) bone direction onto it,
      4. divide by the parent's rotation to get LOCAL rotations.

    Palm facing is deliberately NOT represented. The hand bone owns it.
    """
    points = [to_vector(p) for p in points]

    # ---- the measured hand frame, in the same shape as the canonical one -------------------
    forward = normalize(points[MIDDLE_MCP] - points[WRIST])
    across = normalize(points[PINKY_MCP] - points[INDEX_MCP])
    if forward is None or across is None:
        return None

    normal = normalize(np.cross(forward, across))
    if normal is None:
        return None

    # Re-orthogonalise: `across` is not exactly perpendicular to `forward` on a real hand.
    side = normalize(np.cross(normal, forward))
    if side is None:
        return None

    # Vectors as matrix COLUMNS: local-frame coordinates -> world coordinates.
    hand_frame = np.array([[side[0], normal[0], forward[0]],
                           [side[1], normal[1], forward[1]],
                           [side[2], normal[2], forward[2]]], dtype=np.float64)

    canonical = canonical_open_hand()
    canonical_rest = rest if rest is not None else rest_directions()
    canonical_frame = hand_frame_of(canonical)

    def to_canonical(v_world):
        """Expresses a direction measured on the real hand in the canonical hand's frame."""
        return canonical_frame.dot(hand_frame.T.dot(v_world))

    # ---- per-bone rotations ---------------------------------------------------------------
    world = {}
    for finger in FINGERS:
        base, mcp, pip, tip = FINGER_LANDMARKS[finger]
        chain = (base, mcp, pip, tip)

        for i, segment in enumerate(SEGMENTS):
            name = "{}{}{}".format(side_name, finger, segment)
            a, b = points[chain[i]], points[chain[i + 1]]

            measured = normalize(b - a)
            if measured is None:
                continue

            # The rest direction for this bone, and the measured one, both in the canonical
            # hand's own frame. Comparing them there means the answer is purely the finger's
            # bend relative to the palm, with no contribution from how the hand is oriented.
            bone_rest = canonical_rest[finger]

            canonical_a = canonical[chain[i]]
            canonical_b = canonical[chain[i + 1]]
            canonical_direction = normalize(vec(*canonical_b) - vec(*canonical_a))
            if canonical_direction is None:
                canonical_direction = bone_rest

            measured_in_hand = to_canonical(measured)

            # How far this joint is BENT.
            #
            # The bend must NOT be read off the angle between the rest and measured directions,
            # because that mixes in the finger's roll and -- far worse -- depends on MediaPipe's
            # DEPTH estimate. Monocular depth is least reliable exactly when the hand is hardest
            # to read: with the palm facing the camera the hand is a flat shape in the image, z is
            # largely guessed, and "curling towards the palm" then reads as pointing the wrong way.
            # The user isolated this precisely: an edge-on hand folded correctly, a palm-on hand
            # folded outwards.
            #
            # So measure only the rotation component about the axis a finger physically folds
            # about -- the palm's transverse axis. Depth noise lands mostly on the other axes and
            # is discarded. The atan2 form keeps a folded finger distinguishable from a straight
            # one, using the out-of-plane sine term against the in-plane cosine term.
            palm_normal = vec(0.0, 0.0, 1.0)                  # out of the back of the hand
            fold_axis = np.cross(palm_normal, canonical_direction)
            fold_axis_length = float(np.linalg.norm(fold_axis))

            if fold_axis_length < 1e-9:
                # The bone lies along the palm normal: nothing to fold about.
                world[name] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
                continue

            fold_axis = fold_axis / fold_axis_length

            along_fold = float(np.dot(canonical_direction, measured_in_hand))
            out_of_plane = float(np.dot(palm_normal, measured_in_hand))
            bend = math.atan2(out_of_plane, along_fold)

            rotation = quat_from_axis_angle(fold_axis, bend)

            # If the fold model cannot reproduce the measured direction the joint is doing
            # something it cannot express (a splayed or twisted finger). Fall back to the exact
            # arc so the DIRECTION is never wrong, even if the sense is then less certain.
            landed = rotate(rotation, canonical_direction)
            if float(np.dot(landed, measured_in_hand)) < 0.85:
                rotation = quat_between(canonical_direction, measured_in_hand)

            world[name] = rotation

    # ---- world -> local -------------------------------------------------------------------
    local = {}
    for finger in FINGERS:
        parent_world = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        for segment in SEGMENTS:
            name = "{}{}{}".format(side_name, finger, segment)
            if name not in world:
                continue
            local[name] = quat_mul(quat_conjugate(parent_world), world[name])
            parent_world = world[name]

    # NOTE: there is deliberately no "reverse the fold" switch here.
    #
    # Flipping the fold is not a sign change. Negating the rotation axis cancels against the
    # axis's own sign and yields an identical quaternion; a half turn about the bone's own axis
    # yields a 180 degree rotation instead of a reversed bend. Both were tried and neither does
    # what the name promises, so the switch was removed rather than shipped as a lie. The bend
    # below is the geometrically correct one for the landmarks MediaPipe reports.
    return local


# --------------------------------------------------------------------------------------
# Sending
# --------------------------------------------------------------------------------------
def palm_frame_quaternion(frame):
    """The palm's orientation as a quaternion, in the same space the landmarks are in.

    Sent to Kinect2VMC.exe, which owns the skeleton and composes this with Kinect's arm into one
    coherent hand rotation. Doing the composition there is the point: two senders on one bone is
    what made the hand twitch.

    Also orthonormalised. The three landmark-derived directions are close to, but not exactly,
    perpendicular, and a slightly skewed triple fed to a quaternion builder is a silent error --
    a class of mistake this project has already paid for several times.
    """
    forward, normal, side = frame

    forward = normalize(forward)
    if forward is None:
        return None

    # Remove any component of the normal along the fingers, so the two are exactly perpendicular.
    normal = normalize(normal - forward * float(np.dot(normal, forward)))
    if normal is None:
        return None

    # Derive the third axis rather than trusting the landmark-derived one, so the triple is
    # right-handed by construction.
    side = normalize(np.cross(normal, forward))
    if side is None:
        return None

    # Columns are the images of X, Y and Z, so the ORDER here is the convention.
    #
    # X is the finger direction, Y across the palm, Z out of the back of the hand. Getting this
    # order wrong does not produce an obviously broken result -- it produces a plausible rotation
    # of the wrong magnitude, which is how it slipped through: with the first two columns swapped
    # the quaternion came out at length 0.707 instead of 1.
    axes = np.array([
        [forward[0], side[0], normal[0]],
        [forward[1], side[1], normal[1]],
        [forward[2], side[2], normal[2]],
    ], dtype=np.float64)

    quat = quat_from_axes(axes)

    # Normalise. The matrix is orthonormal so the result should already be a unit quaternion, but
    # one that is a fraction off unit is a rotation that is a fraction wrong, and the receiver has
    # no way to tell.
    length = float(np.linalg.norm(quat))
    if length < 1e-9:
        return None
    return quat / length


def quat_from_axes(m):
    """Quaternion from a rotation matrix whose COLUMNS are the images of X, Y and Z."""
    trace = m[0][0] + m[1][1] + m[2][2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s

    return np.array([x, y, z, w], dtype=np.float64)


def hand_orientation(points):
    """The hand's own frame from landmarks: (forward, palm_normal, side).

    forward: wrist -> middle knuckle, i.e. along the fingers.
    normal : out of the back of the hand. This is the axis that makes a wrist's roll meaningful,
             and it is exactly what a single bone direction cannot supply.
    side   : across the knuckles.

    Only well defined for metric world landmarks; on normalised image landmarks the depth is too
    noisy to build a trustworthy frame.
    """
    if not points or len(points) < 21:
        return None

    forward = normalize(np.array(points[MIDDLE_MCP], dtype=np.float64)
                        - np.array(points[WRIST], dtype=np.float64))
    across = normalize(np.array(points[PINKY_MCP], dtype=np.float64)
                       - np.array(points[INDEX_MCP], dtype=np.float64))
    if forward is None or across is None:
        return None

    normal = normalize(np.cross(forward, across))
    if normal is None:
        return None

    side = normalize(np.cross(normal, forward))
    if side is None:
        return None

    return forward, normal, side


def rest_forearm_direction():
    """The forearm's direction in the canonical hand frame: along the fingers.

    A straight wrist means the forearm points the same way the fingers do, which is what makes
    "no rotation" the neutral answer for the wrist.
    """
    return vec(1.0, 0.0, 0.0)


def rest_palm_normal():
    """The palm normal in the canonical hand frame, matching canonical_open_hand's +Z."""
    return vec(0.0, 0.0, 1.0)

def hand_bone_rotation(points, rest_forward, rest_normal, measured_frame=None, roll_only=False):
    """The hand bone's LOCAL rotation: how the palm is turned relative to a straight wrist.

    This is the wrist's missing dimension. Without it the hand is a rigid extension of the
    forearm, so lowering the arm leaves the palm facing forward and raising it to the chest looks
    wrong: the model has no joint limit forcing a natural hand angle, and nothing tells it where
    the palm actually points.

    Computed on the FOREARM's own axes, which is what VMC's Hand bone wants -- a local rotation
    relative to its parent:

        swing : carries the forearm's direction onto the hand's direction      (wrist bend)
        roll  : carries the rest palm normal onto the measured one, about the
                hand's direction                                              (wrist twist)

    and the result is swing * roll.

    ORDER MATTERS and was wrong at first. The natural-looking "roll about forward" has to be
    built in the UNDO direction -- undoing the swing from the measured normal leaves the roll --
    because composing roll-after-swing in the other order does not carry the normal where it
    should go. With the order reversed here, a pure 45 degree roll measured 132 degrees and a
    90 degree roll measured less than the 45, which is impossible.
    """
    # `measured_frame` lets a caller that has already built the hand's frame pass it in, instead
    # of it being derived twice from the same landmarks.
    frame = measured_frame if measured_frame is not None else hand_orientation(points)
    if frame is None:
        return None

    forward, normal, _side = frame

    swing = quat_between(rest_forward, forward)

    # Undo the swing on the measured normal. What remains is the palm's rotation about the
    # hand's own axis: the pure roll, with the wrist bend taken back out.
    normal_after_undo = rotate(swing, normal)
    dot = max(-1.0, min(1.0, float(np.dot(rest_normal, normal_after_undo))))

    axis = np.cross(rest_normal, normal_after_undo)
    length = float(np.linalg.norm(axis))
    if length < 1e-9:
        if dot > 0.0:
            roll = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        else:
            # Exactly opposed: a half turn about the bone's own axis.
            roll = quat_from_axis_angle(forward, math.pi)
    else:
        roll = quat_from_axis_angle(axis / length, math.acos(dot))

    if roll_only:
        # Send the TWIST alone, with no direction component at all.
        #
        # This is the honest division of labour between two trackers that cannot see each other.
        # The swing above is built from an ASSUMED forearm direction, while the real one comes
        # from Kinect -- and when the assumption is wrong, the swing is wrong too. Worse, it is
        # wrong in a way that depends on how the arm is held, so the same wrist angle produces a
        # different value for every arm position:
        #
        #   hand rotated 0 / 60 / 120 degrees in space, wrist unchanged
        #     -> swing+roll gave four DIFFERENT rotations, which is exactly the bug
        #
        # The roll has no such dependency. It is a rotation about the bone's own axis, it needs to
        # know nothing about the forearm, and it is the part the camera is genuinely better at
        # measuring than Kinect. So send only that and let Kinect own the direction.
        return roll

    return quat_mul(swing, roll)


def hand_center_x(hand):
    """Average x of a detection, in normalised image coordinates. Used to tell the hands apart."""
    pts = hand.points
    if not pts:
        return 0.0
    return sum(p[0] for p in pts) / float(len(pts))

def parse_vmc_bone(data):
    """Minimal OSC reader for /VMC/Ext/Bone/Pos. Returns (bone_name, [x, y, z, w]) or None.

    Written by hand rather than reusing the sender's writer so capture mode needs no extra
    dependency beyond the standard library.
    """
    if not data or data[0:1] != b"/":
        return None

    def read_string(offset):
        end = data.index(b"\x00", offset)
        text = data[offset:end].decode("ascii", "replace")
        offset = end + 1
        while offset % 4 != 0:
            offset += 1
        return text, offset

    try:
        address, offset = read_string(0)
        if address != "/VMC/Ext/Bone/Pos":
            return None

        tags, offset = read_string(offset)
        if not tags.startswith(",s"):
            return None

        name, offset = read_string(offset)
        values = []
        for tag in tags[2:]:
            if tag != "f" or offset + 4 > len(data):
                break
            values.append(struct.unpack_from(">f", data, offset)[0])
            offset += 4
        if len(values) < 7:
            return None
        return name, values[3:7]
    except Exception:
        return None


class PoseBlender:
    """Blends between two CAPTURED poses of the model's own hand.

    This is the approach the user proposed, and it is the right one. Reconstructing fifteen
    joint angles from 21 monocular landmarks is an ill-posed problem -- the attempts kept
    producing poses the model was never meant to make, and only the user could see that.

    Instead, two poses the model can ALREADY make are captured through Warudo's own VMC sender
    and interpolated between. Because they come from the rig itself, the bone names, axis
    conventions and bind pose are correct by construction: nothing is derived, and the avatar can
    never be driven somewhere it cannot go.

    Without a capture file this stays inert, so the geometric path remains available.
    """

    def __init__(self, path=None):
        self.open_pose = {}
        self.fist_pose = {}
        self.loaded = False
        if path:
            self.load(path)

    def load(self, path):
        if not path or not os.path.isfile(path):
            return False
        try:
            with open(path, "r") as handle:
                data = json.load(handle)
            self.open_pose = data.get("open", {})
            self.fist_pose = data.get("fist", {})
            self.loaded = bool(self.open_pose) and bool(self.fist_pose)
        except Exception as exc:
            print("Could not read the pose file: {}".format(exc))
            self.loaded = False
        return self.loaded

    def describe(self):
        if not self.loaded:
            return "no captured poses"
        return "{} bones captured".format(
            len(set(self.open_pose) | set(self.fist_pose)))

    def blend(self, closed, side=None):
        """Returns {bone: quaternion} for a curl value in 0..1.

        `side` filters the result to ONE hand. Without it both hands' bones come back, and since
        the caller writes whatever it is given, a single detected hand drove BOTH model hands --
        which also made the per-hand bone counters identical, hiding the fault.
        """
        if not self.loaded:
            return {}

        # Latch near the ends so the model rests exactly ON its own poses rather than near them,
        # which is what it looks best doing.
        t = 0.0 if closed < 0.25 else (1.0 if closed > 0.65 else (closed - 0.25) / 0.40)

        names = set(self.open_pose) | set(self.fist_pose)
        if side is not None:
            names = set(n for n in names if n.startswith(side))

        result = {}
        for name in names:
            a = self.open_pose.get(name)
            b = self.fist_pose.get(name)
            if a is None:
                result[name] = b
                continue
            if b is None:
                result[name] = a
                continue
            result[name] = slerp(np.array(a, dtype=np.float64),
                                 np.array(b, dtype=np.float64), t)
        return result

    def blend_fingers(self, curls, side=None):
        """每根手指用自己的弯曲量插值（"原生"模式）。

        与 blend() 唯一的区别：blend() 用**一个**整体闭合值驱动所有骨骼，
        所以五根手指只能一起动；这里每根手指各用自己的值。

        之所以放在这里而不是接收端：两张姿态（open/fist）就在这个类里，
        每根手指的弯曲量也在这里算出来 —— 判断只需要一个地方做，接收端只管写骨骼。
        """
        if not self.loaded:
            return {}

        names = set(self.open_pose) | set(self.fist_pose)
        if side is not None:
            names = set(n for n in names if n.startswith(side))

        values = [v for v in curls.values() if v is not None]
        average = sum(values) / float(len(values)) if values else 0.0

        result = {}
        for name in names:
            value = None
            for finger in FINGERS:
                if finger in name:
                    value = curls.get(finger)
                    break
            if value is None:
                # 手骨之类的非手指骨骼：没有自己的弯曲量，用平均值跟随整体。
                value = average

            # 与 blend() 相同的端点锁定：让模型停在它自己的姿态上，而不是停在姿态附近。
            t = 0.0 if value < 0.25 else (1.0 if value > 0.65 else (value - 0.25) / 0.40)

            a = self.open_pose.get(name)
            b = self.fist_pose.get(name)
            if a is None:
                result[name] = b
                continue
            if b is None:
                result[name] = a
                continue
            result[name] = slerp(np.array(a, dtype=np.float64),
                                 np.array(b, dtype=np.float64), t)
        return result


class HandFrameSender:
    """Sends the measured palm orientation back to Kinect2VMC.exe.

    Why the data flows this way
    --------------------------
    The hand's orientation needs two things that live in different processes: the ARM comes from
    Kinect (only that program can read the sensor) and the PALM comes from this camera. Neither can
    be turned into a correct, self-consistent hand rotation on its own, and having them sent
    separately means the two sources fight over one bone -- which is exactly the twitching that was
    seen.

    So this sends the palm's orientation to the program that already owns the skeleton, and that
    program composes the two and emits one coherent pose.

    The message is a single quaternion rather than two vectors, because the combination needs an
    orthonormal frame and a quaternion IS one. It also keeps the wire format to four floats.

        "K2VH"                  magic
        frame counter (uint32)  so the receiver can tell a live stream from a stale one
        side                    'L' or 'R'
        valid                   '1' or '0' -- '0' means the hand was not seen, and the receiver
                                should fall back rather than hold a stale orientation
        qx qy qz qw             palm orientation in CHARACTER space
    """

    MAGIC = b"K2VH"

    # 4 magic + 4 counter + 1 side + 1 valid + 2 padding + 12 forward + 12 normal = 36.
    #
    # This was 16, which is smaller than the 28 the older quaternion layout needed.
    # struct.pack_into then raised, the exception was swallowed by the guard below, and NOT ONE
    # packet ever left -- with the only symptom being an avatar whose palm never turned.
    # The length is kept next to the layout so it cannot drift again.
    PACKET = 36

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.counter = 0
        self.sent = 0
        self.failures = 0

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except Exception as exc:
            raise RuntimeError("无法创建 UDP socket: {}".format(exc))

    def send(self, side, forward, normal, valid=True):
        """Sends one hand's palm frame as two direction vectors.

        Vectors rather than a quaternion: the receiving side builds the bone's rotation from the
        bone's own axis convention (local X = fingers, local Y = back of hand), and a quaternion
        baked in MediaPipe's camera frame would have to be un-baked first. Two directions survive
        the trip unchanged.

        Never raises: a diagnostic must not stop tracking.
        """
        if self.sock is None:
            return

        self.counter = (self.counter + 1) & 0xFFFFFFFF
        f = forward if forward is not None else (0.0, 0.0, 0.0)
        n = normal if normal is not None else (0.0, 0.0, 0.0)

        # EVERYTHING that can fail is inside the guard, including the packing.
        #
        # Packing used to sit outside it, so a layout mistake raised once per frame; worse, the
        # guard around the send swallowed nothing because the failure happened earlier -- and with
        # no console attached the traceback went nowhere. One wrong constant meant the feature was
        # silently dead. Reporting the first failure is what turns that into something visible.
        try:
            packet = bytearray(self.PACKET)
            packet[0:4] = self.MAGIC
            struct.pack_into("<I", packet, 4, self.counter)
            packet[8] = ord("L") if side == "Left" else ord("R")
            packet[9] = ord("1") if valid else ord("0")
            packet[10] = 0
            packet[11] = 0
            struct.pack_into("<3f", packet, 12,
                             float(f[0]), float(f[1]), float(f[2]))
            struct.pack_into("<3f", packet, 24,
                             float(n[0]), float(n[1]), float(n[2]))
            self.sock.sendto(bytes(packet), (self.host, self.port))
            self.sent += 1
        except Exception as exc:
            self.failures += 1
            if self.failures == 1:
                # Once is enough to explain the problem; repeating it every frame would drown the
                # status output.
                print("Palm frame send failed ({}): {}".format(type(exc).__name__, exc), flush=True)

    def release(self):
        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass
        self.sock = None

    # 骨骼帧（K2VB）：把已经算好的骨骼旋转直接发给插件，应用在最后。
    #
    # 复用给 VMC 的那份手指骨骼旋转 —— 那套计算是验证过的，不必再实现一遍，
    # 也不需要任何"张手/握拳姿态采集"。
    def send_bones(self, pairs, magic=b"K2VB"):
        """pairs: (name, quaternion) 序列。非手指骨骼会被忽略。"""
        if self.sock is None:
            return
        try:
            body = bytearray()
            count = 0
            for name, q in pairs:
                if not any(f in name for f in ("Thumb", "Index", "Middle", "Ring", "Little")):
                    continue
                encoded = name.encode("utf-8")
                if not encoded or len(encoded) > 255:
                    continue
                body += struct.pack("<B", len(encoded)) + encoded
                body += struct.pack("<4f", float(q[0]), float(q[1]), float(q[2]), float(q[3]))
                count += 1

            if count == 0:
                return

            self.counter = (self.counter + 1) & 0xFFFFFFFF
            packet = bytearray()
            packet += magic
            packet += struct.pack("<I", self.counter)
            packet += struct.pack("<H", count)
            packet += body
            self.sock.sendto(bytes(packet), (self.host, self.port))
        except Exception as exc:
            self.failures += 1
            if self.failures == 1:
                print("Bone frame send failed ({}): {}".format(type(exc).__name__, exc), flush=True)

    # 手指帧：每只手 5 个弯曲量（拇指→小指，0 = 张开，1 = 完全弯曲）。
    #
    # 只发"弯了多少"，不发骨骼旋转：弯曲轴与方向由模型自己的张手/握拳两个姿态决定，
    # 接收端在两者之间插值。这样就不必猜这根骨骼的局部弯曲轴是哪一根、符号朝哪边 ——
    # 手部朝向当初就是靠猜轴耗掉了十几轮。
    FINGER_PACKET = 4 + 4 + 1 + 1 + 2 + 20   # magic + counter + side + valid + pad + 5 floats

    def send_fingers(self, side, curls, valid=True):
        """curls: 长度为 5 的序列（拇指、食指、中指、无名指、小指）。"""
        if self.sock is None:
            return
        self.counter = (self.counter + 1) & 0xFFFFFFFF
        try:
            packet = bytearray(self.FINGER_PACKET)
            packet[0:4] = b"K2VF"
            struct.pack_into("<I", packet, 4, self.counter)
            packet[8] = ord("L") if side == "Left" else ord("R")
            packet[9] = ord("1") if valid else ord("0")
            packet[10] = 0
            packet[11] = 0
            values = list(curls)[:5]
            while len(values) < 5:
                values.append(0.0)
            struct.pack_into("<5f", packet, 12, *[float(v) for v in values])
            self.sock.sendto(bytes(packet), (self.host, self.port))
        except Exception as exc:
            self.failures += 1
            if self.failures == 1:
                print("Finger frame send failed ({}): {}".format(type(exc).__name__, exc), flush=True)


class VmcSender:
    def __init__(self, address, port):
        if udp_client is None:
            raise RuntimeError("python-osc is missing. Run:  python -m pip install python-osc")
        self.client = udp_client.SimpleUDPClient(address, port)
        self.sent = 0

    def send_ok(self):
        self.client.send_message("/VMC/Ext/OK", [1, 1, 1])

    def send_time(self, seconds):
        self.client.send_message("/VMC/Ext/T", float(seconds))

    def send_bone(self, name, q):
        # Position is zero: this program overrides rotations only, exactly like the body side.
        self.client.send_message("/VMC/Ext/Bone/Pos",
                                 [name, 0.0, 0.0, 0.0] + quat_to_list(q))


# --------------------------------------------------------------------------------------
# Camera
# --------------------------------------------------------------------------------------
def _open_camera(index):
    """OpenCV camera open, with the backend noise suppressed where possible.

    Windows exposes virtual cameras (NVIDIA Broadcast, DroidCam and friends) that log heavily
    and can be slow to enumerate. The environment variable below silences OpenCV's own
    diagnostics so a camera probe prints only the useful lines.
    """
    return cv2.VideoCapture(index, cv2.CAP_DSHOW)

def list_cameras(max_index=8):
    print("Probing camera indices 0..{} (a working index opens and returns a frame)".format(max_index))
    print()
    for index in range(max_index + 1):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame = cap.read()
        if ok and frame is not None:
            h, w = frame.shape[:2]
            print("  index {:<3} OK   {}x{}".format(index, w, h))
        else:
            print("  index {:<3} opened but returned no frame".format(index))
        cap.release()
    print()
    print("Kinect v2's colour camera is 1920x1080; the laptop webcam is 1280x720.")
    print("Pick the 1920x1080 one with --camera N.")


class KinectColour:
    """Reads Kinect v2's colour stream through pykinect2, if it is installed.

    Kept only for reference. This does NOT work on a current Python: pykinect2 fails at import
    with `AssertionError: 80` from `assert sizeof(tagSTATSTG) == 72`, because the ctypes structure
    layouts moved on after the library was written in 2016. Use ColourRelay instead.
    """

    def __init__(self):
        try:
            from pykinect2 import PyKinectV2, PyKinectRuntime
        except ImportError:
            raise RuntimeError("pykinect2 is not installed. Run:  python -m pip install pykinect2")

        self._v2 = PyKinectV2
        self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color)

    def read(self):
        if not self.kinect.has_new_color_frame():
            return False, None

        frame = self.kinect.get_last_color_frame()
        frame = frame.reshape((1080, 1920, 4)).astype(np.uint8)
        bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return True, bgr

    def release(self):
        try:
            self.kinect.close()
        except Exception:
            pass


class ColourRelay:
    """Reads the Kinect's colour stream forwarded by Kinect2VMC.exe over loopback.

    Why this exists rather than reading the camera directly
    -----------------------------------------------------
    Only one process can hold the Kinect, and Kinect2VMC.exe already does. The usual Python
    binding for the sensor cannot be used at all on a current Python (see KinectColour above).

    So the program that owns the sensor forwards its colour frames here, and this class just reads
    a socket. That also means the frames are the Kinect's own wide-angle image, which is the point:
    it works when the laptop webcam is broken or already taken by Warudo's face capture.

    Wire format, matching Kinect2VMC's ColourRelay:
        "K2VC"                       4 bytes, once
        then per frame: width, height (int32 little-endian), then width*height*4 bytes BGRA
    """

    MAGIC = b"K2VC"

    def __init__(self, host="127.0.0.1", port=39541, timeout=10.0):
        self.host = host
        self.port = port
        self.sock = None
        self._buffer = None

        try:
            self.sock = socket.create_connection((host, port), timeout=timeout)
            self.sock.settimeout(None)

            magic = self._read_exactly(4)
            if magic != self.MAGIC:
                raise RuntimeError("对端不是 Kinect2VMC 的彩色中继（收到 {!r}）".format(magic))
        except Exception as exc:
            self.release()
            raise RuntimeError(
                "无法连接彩色中继 {}:{} ({})。请先在 Kinect2VMC 里勾选「转发 Kinect 彩色画面」。"
                .format(host, port, exc))

    def _read_exactly(self, count):
        chunks = []
        remaining = count
        while remaining > 0:
            chunk = self.sock.recv(remaining)
            if not chunk:
                raise RuntimeError("彩色中继连接已断开")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def read(self):
        try:
            header = self._read_exactly(8)
            width, height = struct.unpack("<ii", header)
            if width <= 0 or height <= 0 or width > 8192 or height > 8192:
                return False, None

            size = width * height * 4
            if self._buffer is None or len(self._buffer) != size:
                self._buffer = bytearray(size)

            view = memoryview(self._buffer)
            got = 0
            while got < size:
                chunk = self.sock.recv_into(view[got:], size - got)
                if chunk <= 0:
                    return False, None
                got += chunk

            bgra = np.frombuffer(self._buffer, dtype=np.uint8).reshape((height, width, 4))
            return True, cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        except Exception:
            return False, None

    def release(self):
        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass
        self.sock = None


# --------------------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------------------
def draw(frame, points, side_name, colour):
    """points: 21 (x, y, z) tuples in normalised image coordinates."""
    h, w = frame.shape[:2]

    pts = {}
    for index, p in enumerate(points):
        pts[index] = (int(p[0] * w), int(p[1] * h))

    connections = [
        (0, 1), (1, 2), (2, 3), (3, 4),
        (0, 5), (5, 6), (6, 7), (7, 8),
        (5, 9), (9, 10), (10, 11), (11, 12),
        (9, 13), (13, 14), (14, 15), (15, 16),
        (13, 17), (17, 18), (18, 19), (19, 20),
        (0, 17),
    ]
    for a, b in connections:
        if a in pts and b in pts:
            cv2.line(frame, pts[a], pts[b], colour, 2)

    for index, pt in pts.items():
        cv2.circle(frame, pt, 4, (0, 0, 255) if index in (4, 8, 12, 16, 20) else colour, -1)

    cv2.putText(frame, side_name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, colour, 2)


def main():
    parser = argparse.ArgumentParser(description="MediaPipe hands -> VMC finger bones")
    parser.add_argument("--camera", type=int, default=0, help="camera index (see --list-cameras)")
    parser.add_argument("--list-cameras", action="store_true", help="probe camera indices and exit")
    parser.add_argument("--kinect", action="store_true", help="read Kinect colour via pykinect2 (does NOT work on current Python; see KinectColour)")
    parser.add_argument("--relay", type=int, metavar="PORT", default=None,
                        help="read the Kinect colour stream forwarded by Kinect2VMC.exe on this port")
    parser.add_argument("--relay-host", default="127.0.0.1")
    parser.add_argument("--frame-port", type=int, default=39542,
                        help="UDP port to send the measured palm orientation back to "
                             "Kinect2VMC.exe on. 0 disables it.")
    parser.add_argument("--no-send", action="store_true", help="preview only, send nothing")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=39539)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--hands", type=int, default=2)
    parser.add_argument("--min-detection", type=float, default=0.5)
    parser.add_argument("--min-tracking", type=float, default=0.5)
    parser.add_argument("--no-preview", action="store_true", help="run without the preview window")
    parser.add_argument("--preview", choices=("shown", "black", "hidden"), default="shown",
                        help="shown = camera image (default); black = an all-black window that "
                             "still takes the q/m/d keys; hidden = no window at all")
    parser.add_argument("--flip-palm", action="store_true",
                        help="turn the palm 180 degrees (only if the thumbs point the wrong way)")
    parser.add_argument("--mirrored-source", dest="mirrored_source", action="store_true",
                        default=True,
                        help="the image is mirrored, as a laptop webcam usually is (default)")
    parser.add_argument("--no-mirror", dest="mirrored_source", action="store_false",
                        help="the image is NOT mirrored, as Kinect's colour stream is. Use this "
                             "with --relay, or the two hands come out swapped.")
    parser.add_argument("--hand-swing", action="store_true",
                        help="also send the wrist BEND, not just the twist. Off by default: the "
                             "bend is built from an assumed forearm direction that Kinect owns, "
                             "so sending it fights the body tracker.")
    parser.add_argument("--send-hand", action="store_true",
                        help="also drive the Hand bone. OFF by default: Kinect2VMC supplies it, "
                             "and two senders on one bone make the hand twitch.")
    parser.add_argument("--poses", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json"),
                        help="captured model hand poses to blend between (see --capture)")
    parser.add_argument("--capture", metavar="OPEN,FIST",
                        help="run without the camera and record the model poses arriving on --port")
    parser.add_argument("--capture-seconds", type=float, default=6.0,
                        help="how long to record each captured pose")
    parser.add_argument("--label-hands", action="store_true",
                        help="assign hands from MediaPipe labels instead of screen position")
    parser.add_argument("--no-swap-hands", action="store_true",
                        help="trust MediaPipe's own left/right labels instead of swapping them")
    parser.add_argument("--dump-wrist", action="store_true",
                        help="print the raw wrist rotation each frame, for diagnosis")
    args = parser.parse_args()

    # ---- capture mode: record two of the MODEL's own hand poses ---------------------------
    # Warudo can act as a VMC sender. Pointing it at a spare port lets this record the hand's
    # real bone rotations while the user poses the model by hand. That is what makes blending
    # possible: the poses then come from the rig itself and nothing has to be derived.
    if args.capture:
        out_path = args.capture
        import socket

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        sock.bind(("0.0.0.0", args.port))
        sock.settimeout(0.5)

        print("Listening on UDP {} for Warudo's VMC sender.".format(args.port))
        print("Arm it BEFORE Warudo starts, or re-enable the sender after this is listening:")
        print("a UDP sender that finds nobody home simply drops its packets.")
        print()

        poses = {"open": {}, "fist": {}}
        for key, label in (("open", "OPEN"), ("fist", "FIST")):
            try:
                raw = input("Pose the model's hand as {} in Warudo, then press Enter... ".format(label))
            except EOFError:
                raw = ""

            print("  recording {} for {:.0f}s -- hold the pose".format(label, args.capture_seconds))
            bones = {}
            packets = 0
            deadline = time.time() + args.capture_seconds
            while time.time() < deadline:
                try:
                    data, _ = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                packets += 1
                parsed = parse_vmc_bone(data)
                if parsed:
                    bones[parsed[0]] = parsed[1]

            if not bones:
                print("  NOTHING ARRIVED after {} UDP packets.".format(packets))
                print("  Check that Warudo's VMC Sender is enabled and set to 127.0.0.1:{}"
                      .format(args.port))
                return 2

            fingers = dict((k, v) for k, v in bones.items()
                           if any(f in k for f in ("Thumb", "Index", "Middle", "Ring", "Little")))
            print("  got {} bones, {} of them fingers".format(len(bones), len(fingers)))
            poses[key] = fingers

        with open(out_path, "w") as handle:
            json.dump(poses, handle, indent=1, sort_keys=True)
        print()
        print("Written to {}".format(out_path))
        print("Now run:  hand_tracker.py --camera 0 --poses \"{}\"".format(out_path))
        return 0

    if args.list_cameras:
        list_cameras()
        return 0

    # ---- camera -----------------------------------------------------------------------
    if args.relay is not None:
        # Preferred Kinect path: the program that owns the sensor forwards its colour frames.
        try:
            source = ColourRelay(args.relay_host, args.relay)
            read = source.read
            release = source.release
            print("Source: Kinect v2 colour via Kinect2VMC relay on port {}".format(args.relay))
        except Exception as exc:
            print("Colour relay unavailable: {}".format(exc))
            return 2
    elif args.kinect:
        try:
            source = KinectColour()
            read = source.read
            release = source.release
            print("Source: Kinect v2 colour (pykinect2)")
        except Exception as exc:
            print("Kinect source unavailable: {}".format(exc))
            return 2
    else:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print("Camera index {} would not open. Run --list-cameras.".format(args.camera))
            return 2
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

        def read():
            ok, frame = cap.read()
            return ok, frame

        def release():
            cap.release()

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("Source: camera {} at {}x{}".format(args.camera, actual_w, actual_h))

    # ---- sender -----------------------------------------------------------------------
    sender = None
    if not args.no_send:
        try:
            sender = VmcSender(args.host, args.port)
            print("Sending VMC to {}:{}".format(args.host, args.port))
        except Exception as exc:
            print("Sender unavailable: {}".format(exc))
            return 2
    else:
        print("Preview only (--no-send)")

    # ---- mediapipe --------------------------------------------------------------------
    #
    # The model is downloaded HERE, and on a fresh copy of this folder that takes a few seconds with
    # nothing on screen -- which Windows reports as "not responding", and is exactly the freeze seen
    # on first launch. So the preview window is created first and used to show progress.
    #
    # --no-preview stays supported and simply means the hidden mode.
    preview_mode = "hidden" if args.no_preview else args.preview
    window_title = "hand_tracker  (q quit, m mirror, d preview)"
    model_window_ready = preview_mode != "hidden"

    if model_window_ready:
        cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

    def show_download_progress(received, total):
        if not model_window_ready:
            return
        fraction = (received / total) if total > 0 else 0.0
        canvas = np.zeros((120, 460, 3), dtype=np.uint8)
        cv2.putText(canvas, "Preparing the hand model (one time)", (14, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.rectangle(canvas, (14, 60), (446, 82), (70, 70, 70), -1)
        cv2.rectangle(canvas, (14, 60), (14 + int(432 * min(1.0, fraction)), 82),
                      (90, 200, 90), -1)
        cv2.putText(canvas, "{:.0f}%".format(fraction * 100.0), (14, 108),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        cv2.imshow(window_title, canvas)

        # Pump the event loop so Windows keeps painting the window instead of declaring the process
        # unresponsive.
        cv2.waitKey(1)

    landmarker = make_landmarker(args.hands, args.min_detection, args.min_detection,
                                 args.min_tracking, on_progress=show_download_progress)

    finger_rest = rest_directions()
    last_curls = {"Left": None, "Right": None}
    curls_sent = {"Left": 0, "Right": 0}
    curl_errors = {"Left": 0, "Right": 0}
    frame_sender = None
    if args.frame_port > 0:
        try:
            frame_sender = HandFrameSender(args.host, args.frame_port)
            print("Palm orientation -> {}:{}".format(args.host, args.frame_port))
        except Exception as exc:
            print("Palm frame channel unavailable: {}".format(exc))
    blender = PoseBlender(args.poses)
    print("Hand poses:", blender.describe(), flush=True)
    if blender.loaded:
        print("  POSE MODE: fingers blend between the captured poses; no geometry used.",
              flush=True)

    print()
    print("Bones written per hand:", ", ".join(bone_names("Left")), flush=True)
    print("Keys: q/Esc quit, m toggle mirror, d toggle landmarks", flush=True)
    print()

    started = time.time()
    wrist_error_shown = [False]
    show_landmarks = True
    last_report = time.time()
    frames = 0
    detected = 0
    bone_counts = {"Left": 0, "Right": 0}
    black_frame = None

    # Whether this camera's image is mirrored, which decides how the frame's left and right map to
    # the user's hands. The `m` key flips it at runtime so the answer can be checked by eye.
    source_mirrored = args.mirrored_source
    if preview_mode == "black":
        # A small, fully black image: the window exists so keys still arrive, but nothing of the
        # camera or of the user is on screen.
        black_frame = np.zeros((120, 320, 3), dtype=np.uint8)

    try:
        while True:
            ok, frame = read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue

            # The image itself is NOT flipped to match. Doing so would change the frame's left and
            # right, and the position-based hand assignment below already accounts for whether the
            # source is mirrored. Flipping here as well would cancel that out and make the setting
            # impossible to reason about.

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False

            # The Tasks API takes a mediapipe.Image and a monotonically increasing timestamp.
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
            hands = detect(landmarker, mp_image, int((time.time() - started) * 1000))

            writes = {}
            native_writes = {}
            assigned = []
            gesture_report = {}
            frames_sent = {"Left": False, "Right": False}

            # Which model hand each detection drives.
            #
            # MediaPipe's handedness label is NOT usable here: with a single mirrored camera both
            # detections often come back labelled "Left", so mapping by label put both hands onto
            # the same model hand and left the other frozen -- the user saw exactly that, along
            # with "two hands both mapped to Left".
            #
            # Handedness is ambiguous from one camera anyway: a left hand seen from behind and a
            # right hand seen from the front are the same shape. POSITION resolves it instead.
            #
            # Which side of the frame maps to which hand depends on whether the source is MIRRORED,
            # and that differs per camera:
            #   a laptop webcam is usually mirrored, so the image behaves like a mirror and the
            #     user's right hand appears on the left of the frame
            #   Kinect's colour stream is NOT mirrored, so the user's right hand appears on the
            #     right of the frame, like a photograph of them
            #
            # The `leftmost = user's right` rule is therefore only correct for a mirrored source,
            # and using it with Kinect swapped the hands.
            if hands and not args.label_hands:
                ordered = sorted(hands, key=lambda h: hand_center_x(h))

                # For a non-mirrored source the frame is a photograph, so the leftmost detection is
                # the user's LEFT hand. For a mirrored one it is the right hand.
                leftmost_is_users = "Right" if source_mirrored else "Left"
                rightmost_is_users = "Left" if source_mirrored else "Right"

                if len(ordered) >= 2:
                    sides = [leftmost_is_users, rightmost_is_users]
                else:
                    # ONE hand only. Decide from which half of the frame it is in, so the answer
                    # does not flip when the other hand comes and goes -- falling back to
                    # MediaPipe's label here is what made a lone hand read as the wrong side.
                    on_left_of_frame = hand_center_x(ordered[0]) < 0.5
                    sides = [leftmost_is_users if on_left_of_frame else rightmost_is_users]
                plan = list(zip(ordered, sides))
            else:
                plan = []
                for hand in hands:
                    reported = hand.label
                    if args.no_swap_hands:
                        side = reported
                    else:
                        side = "Right" if reported == "Left" else "Left"
                    if source_mirrored:
                        side = "Right" if side == "Left" else "Left"
                    plan.append((hand, side))

            for hand, side in plan:

                # Geometry uses the metric world landmarks; the image ones are only for the
                # preview overlay.
                geometry = hand.world if hand.world else hand.points

                if blender.loaded:
                    # POSE MODE. Classify the gesture and blend between two of the model's OWN
                    # poses. Nothing is derived from landmarks, so the reference-frame mismatch
                    # that made a fist fold outwards cannot occur: the avatar only ever sits
                    # between two poses it can already make.
                    per_finger, closed = hand_gesture(geometry, use_3d=bool(hand.world))
                    local = blender.blend(closed, side=side)

                    # 同一份 MediaPipe 数据的另一种表现：每根手指各用自己的弯曲量。
                    # 两份都发，接收端按开关选一个应用 —— 判断只在这一个地方做。
                    if blender.loaded:
                        native_writes.update(blender.blend_fingers(per_finger, side=side))

                    # The wrist, which the pose data cannot supply.
                    #
                    # Two poses of the hand give the fingers something to blend between, but
                    # the hand bone itself was left as a rigid extension of the forearm. That is
                    # why lowering the arm left the palm facing forward and raising it to the
                    # chest looked wrong: nothing said where the palm pointed.
                    #
                    # MediaPipe can say: the palm normal comes from the knuckle line, and the
                    # wrist rotation relative to the forearm follows from it. Added here rather
                    # than stored in the poses because it is a measured quantity, not one of the
                    # model's own poses.
                    if hand.world:
                        try:
                            # The palm's orientation goes back to Kinect2VMC.exe, which owns the
                            # skeleton and will compose it with Kinect's arm. Sending it here
                            # instead would put this script and the body tracker on the same bone,
                            # which is what made the hand twitch.
                            frame_now = hand_orientation(hand.world)
                            if frame_now is not None and frame_sender is not None:
                                # frame_now = (forward, normal, side). The two directions are what
                                # the receiver needs: its bone's local X is the finger direction
                                # and its local Y is the back-of-hand normal.
                                frame_sender.send(side, frame_now[0], frame_now[1])
                                frames_sent[side] = True

                                # 每根手指弯了多少（拇指、食指、中指、无名指、小指）。
                                # 接收端拿它在"张手姿态"和"握拳姿态"之间插值 ——
                                # 弯曲轴与方向由模型自己的两个姿态决定，不需要猜。
                                try:
                                    per_finger, _closed = hand_gesture(
                                        geometry, use_3d=bool(hand.world))
                                    curls_now = [
                                        per_finger.get("Thumb", 0.0),
                                        per_finger.get("Index", 0.0),
                                        per_finger.get("Middle", 0.0),
                                        per_finger.get("Ring", 0.0),
                                        per_finger.get("Little", 0.0),
                                    ]
                                    frame_sender.send_fingers(side, curls_now)
                                    last_curls[side] = curls_now
                                    curls_sent[side] += 1
                                except Exception as exc:
                                    # 手指只是附加信息，失败不该影响手部朝向；但要报一次，
                                    # 否则"每帧都抛异常"会表现成"数值冻结"，完全看不出原因。
                                    curl_errors[side] += 1
                                    if curl_errors[side] == 1:
                                        print("Finger curl failed ({}): {}".format(
                                            type(exc).__name__, exc), flush=True)

                            wrist = hand_bone_rotation(hand.world,
                                                      rest_forearm_direction(),
                                                      rest_palm_normal(),
                                                      roll_only=not args.hand_swing)
                            if wrist is not None:
                                if args.send_hand:
                                    local[side + "Hand"] = wrist

                                # Diagnostic: print the raw wrist values so a problem can be
                                # read rather than inferred. The geometry checks out on synthetic
                                # data, so if the model twitches or the palm does not follow, the
                                # numbers here say which of the two is happening -- an unstable
                                # rotation, or a stable rotation pointing the wrong way.
                                if args.dump_wrist:
                                    frame_now = hand_orientation(hand.world)
                                    angle = 2.0 * math.degrees(
                                        math.acos(min(1.0, abs(float(wrist[3])))))
                                    axis = np.array([wrist[0], wrist[1], wrist[2]])
                                    length = float(np.linalg.norm(axis))
                                    axis = axis / length if length > 1e-9 else axis
                                    print(
                                        "WRIST {} angle={:7.2f} axis=({:+.3f},{:+.3f},{:+.3f}) "
                                        "q=({:+.3f},{:+.3f},{:+.3f},{:+.3f}) fwd=({:+.3f},{:+.3f},{:+.3f}) "
                                        "nrm=({:+.3f},{:+.3f},{:+.3f})".format(
                                            side, angle, axis[0], axis[1], axis[2],
                                            wrist[0], wrist[1], wrist[2], wrist[3],
                                            frame_now[0][0], frame_now[0][1], frame_now[0][2],
                                            frame_now[1][0], frame_now[1][1], frame_now[1][2]),
                                        flush=True)
                        except Exception as exc:
                            if not wrist_error_shown[0]:
                                print("  wrist rotation unavailable: {}".format(exc), flush=True)
                                wrist_error_shown[0] = True
                    gesture_report[side] = closed
                else:
                    local = solve_hand(geometry, side, mirror_x=source_mirrored, rest=finger_rest,
                                        flip_palm=args.flip_palm)
                if local:
                    # A later detection must not silently overwrite an earlier one for the same
                    # side; the caller sees the collision in the log instead.
                    if side in assigned:
                        print("  note: two hands both mapped to {}; keeping the first"
                              .format(side), flush=True)
                        continue
                    writes.update(local)
                    assigned.append(side)

                if show_landmarks:
                    colour = (0, 255, 0) if side == "Left" else (255, 128, 0)
                    draw(frame, hand.points, side, colour)

            if hands:
                detected += 1

            if sender is not None:
                sender.send_ok()
                sender.send_time(time.time())
                for name, q in writes.items():
                    sender.send_bone(name, q)
                    side = "Left" if name.startswith("Left") else "Right"
                    bone_counts[side] += 1

            # 同一份手指骨骼旋转也直接发给 Warudo 插件（应用在最后，覆盖死数据）。
            #
            # 必须放在 `sender is not None` 之外：正在验证插件通路的时候 VMC 是关着的，
            # 一旦写在里面，这条通路在"最需要它"的场景下恰好永远不执行 ——
            # 现象就是"已接收手指帧恒为 0"，而数据和计算其实都是好的。
            if frame_sender is not None:
                frame_sender.send_bones(writes.items(), b"K2VB")
                if native_writes:
                    frame_sender.send_bones(native_writes.items(), b"K2VN")

            frames += 1
            now = time.time()
            if now - last_report >= 2.0:
                fps = frames / (now - last_report)
                print("  {:.1f} fps | seen {}/{} | drv L={} R={} | curls L={} R={} | sent L={} R={} err={}".format(
                    fps, detected, frames, bone_counts["Left"], bone_counts["Right"],
                    last_curls["Left"], last_curls["Right"],
                    curls_sent["Left"], curls_sent["Right"],
                    curl_errors["Left"] + curl_errors["Right"]), flush=True)
                frames = 0
                detected = 0
                bone_counts = {"Left": 0, "Right": 0}
                last_report = now

            # The window is a debugging aid, and it steals focus and screen space while streaming.
            # Three modes:
            #   shown  : the camera image with the landmark overlay (the old behaviour)
            #   black  : a small all-black window. Still receives keys, so q/m/d keep working,
            #            but nothing of the camera is on screen -- useful while recording or when
            #            the preview would be visible to an audience.
            #   hidden : no window at all. Keys cannot be read, so the script is quit with Ctrl+C.
            if preview_mode != "hidden":
                if preview_mode == "black":
                    shown = black_frame
                else:
                    shown = frame
                cv2.imshow(window_title, shown)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
                if key == ord("m"):
                    # Flip whether the source counts as mirrored. This is what decides which hand
                    # each detection drives, so it is the key to press when the hands come out
                    # swapped.
                    source_mirrored = not source_mirrored
                    print("  mirrored source: {}".format(source_mirrored), flush=True)
                if key == ord("d"):
                    if preview_mode == "black":
                        preview_mode = "shown"
                    else:
                        show_landmarks = not show_landmarks
    except KeyboardInterrupt:
        pass
    finally:
        landmarker.close()
        release()
        if preview_mode != "hidden":
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())

