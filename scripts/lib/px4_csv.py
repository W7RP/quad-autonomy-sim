"""Read CSVs written by `ros2 topic echo --csv` into named numpy columns.

`ros2 topic echo --csv` writes one row per message with every field flattened
in definition order and no header. Instead of hard-coding column numbers (which
silently break when a message changes), the layout is derived at runtime from
`ros2 interface show --no-comments <type>`, so this needs a sourced ROS 2 env.

Offline analysis tooling only.
"""
import csv
import functools
import re
import subprocess

import numpy as np

_FIELD = re.compile(r"^(?P<type>[\w/]+)(?:\[(?P<n>\d+)\])?\s+(?P<name>\w+)\s*$")


@functools.lru_cache(maxsize=None)
def layout(msg_type):
    """{field: slice} for a flat px4_msgs message (no nested message fields)."""
    out = subprocess.run(["ros2", "interface", "show", "--no-comments", msg_type],
                         check=True, capture_output=True, text=True).stdout
    cols, i = {}, 0
    for line in out.splitlines():
        # Strip comments first: Humble's --no-comments keeps trailing ones, and
        # a comment may itself contain '=' (DistanceSensor.signal_quality does).
        line = line.split("#", 1)[0].strip()
        if not line or "=" in line:  # blank, or a constant definition
            continue
        m = _FIELD.match(line)
        if not m:
            raise ValueError(f"cannot parse field line in {msg_type}: {line!r}")
        n = int(m.group("n") or 1)
        cols[m.group("name")] = slice(i, i + n) if m.group("n") else i
        i += n
    cols["__ncols__"] = i
    return cols


def _to_float(v):
    if v == "True":
        return 1.0
    if v == "False":
        return 0.0
    return float(v)


def read(path, msg_type):
    """Rows of a PX4 CSV as a dict {field: ndarray}, plus '__rows__' (2D array)."""
    lay = layout(msg_type)
    rows = []
    with open(path, newline="") as f:
        for r in csv.reader(f):
            if len(r) != lay["__ncols__"]:
                continue  # truncated last line when the recorder was stopped
            try:
                rows.append([_to_float(v) for v in r])
            except ValueError:
                continue
    a = np.array(rows) if rows else np.zeros((0, lay["__ncols__"]))
    out = {name: a[:, s] for name, s in lay.items() if name != "__ncols__"}
    out["__rows__"] = a
    return out


def detect_clock_steps(t, threshold_s=0.1):
    """Find timesync artefacts in a stamp series (seconds).

    uXRCE-DDS timesync produces two kinds: a one-off glitch (a single sample
    stamped far off, e.g. with raw PX4 boot time) and a persistent clock step
    (every later sample shifted, e.g. +6 s when the simulator runs slower than
    real time). Returns (keep_mask, steps) with steps = [(index, size_s), ...]:
    the series jumps by size_s between sample index-1 and index.
    """
    n = len(t)
    keep = np.ones(n, bool)
    steps = []
    if n < 3:
        return keep, steps
    # Absurd samples first (a raw-boot-time stamp is decades off the synced
    # time). They must go before step detection: the glitch can land on the very
    # sample where a real step happens, so "does the next sample return to the
    # old timeline?" cannot identify it.
    keep &= np.abs(t - np.median(t)) < 3600.0
    idx = np.flatnonzero(keep)
    nominal = float(np.median(np.diff(t[idx])))
    prev = t[idx[0]]
    for k in range(1, len(idx)):
        i = idx[k]
        d = t[i] - prev - nominal
        if abs(d) <= threshold_s:
            prev = t[i]
        elif k + 1 < len(idx) and abs(t[idx[k + 1]] - prev - 2 * nominal) <= threshold_s:
            keep[i] = False  # smaller glitch: the next sample returns to the old timeline
        else:
            steps.append((int(i), float(d)))
            prev = t[i]
    return keep, steps


def unwrap_clock(t, reference_steps=None, threshold_s=0.1):
    """Remove clock steps and glitches from a stamp series.

    Returns (t_corrected, keep_mask). With reference_steps (a list of
    (stamp_s, size_s) from a regular high-rate stream, see reference_steps()),
    each detected step uses the reference size, so all streams get identical
    corrections; a low-rate stream's own estimate of a step size is only good
    to within its sample spacing.
    """
    keep, steps = detect_clock_steps(t, threshold_s)
    offset = np.zeros(len(t))
    for i, size in steps:
        if reference_steps:
            # Same event only: close in time AND in size (a low-rate stream sees
            # the step size to within its sample spacing).
            near = min(reference_steps, key=lambda r: abs(r[0] - t[i]))
            if abs(near[0] - t[i]) < 1.0 and abs(near[1] - size) < 0.2:
                size = near[1]
        offset[i:] += size
    return t - offset, keep


def reference_steps(t, threshold_s=0.1):
    """(stamp_s, size_s) of each clock step in a reference stream."""
    _, steps = detect_clock_steps(t, threshold_s)
    return [(float(t[i]), size) for i, size in steps]


def monotonic_mask(t, absurd_s=5.0):
    """Keep samples whose time increases; drop absurd jumps (timesync glitches)
    and the span after a backward clock step until time passes the old maximum."""
    keep = np.ones(len(t), bool)
    if len(t) == 0:
        return keep
    t_max = t[0]
    for i in range(1, len(t)):
        if abs(t[i] - t_max) > absurd_s or t[i] <= t_max:
            keep[i] = False
        else:
            t_max = t[i]
    return keep
