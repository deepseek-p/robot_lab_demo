#!/usr/bin/env python3
"""Move the RViz WSLg window back onto the leftmost visible XWayland monitor."""

import ctypes
import re
import subprocess
import sys
import time
from ctypes import POINTER, byref, c_char_p, c_int, c_ulong, c_void_p


WINDOW_TITLE_MARKER = "- RViz"


def connected_monitors():
    try:
        output = subprocess.check_output(
            ["xrandr", "--query"], stderr=subprocess.DEVNULL, text=True
        )
    except (OSError, subprocess.CalledProcessError):
        return []

    monitors = []
    pattern = re.compile(r"\S+ connected(?: primary)? (\d+)x(\d+)\+(-?\d+)\+(-?\d+)")
    for line in output.splitlines():
        match = pattern.match(line)
        if not match:
            continue
        width, height, x, y = (int(value) for value in match.groups())
        monitors.append({"width": width, "height": height, "x": x, "y": y})
    return monitors


def target_position():
    monitors = connected_monitors()
    if not monitors:
        return 80, 80

    monitor = min(monitors, key=lambda item: (item["x"], item["y"]))
    return monitor["x"] + 80, monitor["y"] + 80


def find_rviz_window():
    try:
        output = subprocess.check_output(
            ["xwininfo", "-root", "-tree"], stderr=subprocess.DEVNULL, text=True
        )
    except (OSError, subprocess.CalledProcessError):
        return None

    for line in output.splitlines():
        if WINDOW_TITLE_MARKER not in line:
            continue
        match = re.search(r"(0x[0-9a-fA-F]+)", line)
        if match:
            return int(match.group(1), 16)
    return None


def move_window(client_window, x, y):
    xlib = ctypes.CDLL("libX11.so.6")
    xlib.XOpenDisplay.argtypes = [c_char_p]
    xlib.XOpenDisplay.restype = c_void_p
    xlib.XQueryTree.argtypes = [
        c_void_p,
        c_ulong,
        POINTER(c_ulong),
        POINTER(c_ulong),
        POINTER(POINTER(c_ulong)),
        POINTER(ctypes.c_uint),
    ]
    xlib.XFree.argtypes = [c_void_p]
    xlib.XMoveWindow.argtypes = [c_void_p, c_ulong, c_int, c_int]
    xlib.XMapRaised.argtypes = [c_void_p, c_ulong]
    xlib.XRaiseWindow.argtypes = [c_void_p, c_ulong]
    xlib.XFlush.argtypes = [c_void_p]
    xlib.XCloseDisplay.argtypes = [c_void_p]

    display = xlib.XOpenDisplay(None)
    if not display:
        return False

    root = c_ulong()
    parent = c_ulong()
    children = POINTER(c_ulong)()
    child_count = ctypes.c_uint()
    ok = xlib.XQueryTree(
        display,
        c_ulong(client_window),
        byref(root),
        byref(parent),
        byref(children),
        byref(child_count),
    )
    if children:
        xlib.XFree(children)
    if not ok or not parent.value:
        xlib.XCloseDisplay(display)
        return False

    frame_window = parent.value
    xlib.XMoveWindow(display, c_ulong(frame_window), x, y)
    xlib.XMapRaised(display, c_ulong(frame_window))
    xlib.XRaiseWindow(display, c_ulong(frame_window))
    xlib.XFlush(display)
    xlib.XCloseDisplay(display)
    return True


def main():
    x, y = target_position()
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        window = find_rviz_window()
        if window and move_window(window, x, y):
            print(f"Recentered RViz window 0x{window:x} to +{x}+{y}")
            return 0
        time.sleep(0.5)

    print("RViz window was not found for recentering", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
