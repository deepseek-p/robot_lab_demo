#!/usr/bin/env python3
"""Move the RViz WSLg window back onto the leftmost visible XWayland monitor."""

import ctypes
import os
import re
import shutil
import subprocess
import sys
import time
from ctypes import POINTER, byref, c_char_p, c_int, c_ulong, c_void_p


WINDOW_TITLE_MARKER = "- RViz"


def running_under_wsl():
    return bool(os.environ.get("WSL_DISTRO_NAME")) and shutil.which("powershell.exe")


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


def move_windows_window(x, y, width=1500, height=950):
    """Move the WSLg RAIL window from the Windows side.

    WSLg can ignore XMoveWindow for top-level app windows because the actual
    host window is managed by Windows RAIL. When that happens, RViz may be
    running but minimized/off-screen. This PowerShell fallback restores and
    positions the host window directly.
    """
    if not running_under_wsl():
        return False

    script = rf'''
$code=@"
using System;
using System.Text;
using System.Runtime.InteropServices;
public class Win32 {{
  public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
  [DllImport("user32.dll")] public static extern bool EnumWindows(EnumWindowsProc lpEnumFunc, IntPtr lParam);
  [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr hWnd, StringBuilder text, int count);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr hWnd);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
  [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr hWnd, IntPtr hWndInsertAfter, int X, int Y, int cx, int cy, uint uFlags);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
}}
"@;
Add-Type $code;
$moved=$false;
[Win32]::EnumWindows({{
  param($h,$l)
  $sb=New-Object Text.StringBuilder 512
  [void][Win32]::GetWindowText($h,$sb,$sb.Capacity)
  $title=$sb.ToString()
  if([Win32]::IsWindowVisible($h) -and $title -match "RViz|rviz") {{
    [void][Win32]::ShowWindow($h,9)
    Start-Sleep -Milliseconds 150
    [void][Win32]::SetWindowPos($h,[IntPtr]::Zero,{x},{y},{width},{height},0x0040)
    [void][Win32]::SetForegroundWindow($h)
    Write-Host "Moved RViz host window $h to +{x}+{y}"
    $script:moved=$true
  }}
  return $true
}}, [IntPtr]::Zero) | Out-Null;
if(-not $moved) {{ exit 1 }}
'''
    try:
        result = subprocess.run(
            ["powershell.exe", "-NoProfile", "-Command", script],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=8.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return False

    if result.stdout.strip():
        print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)
    return result.returncode == 0


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
        if move_windows_window(x, y):
            return 0
        window = find_rviz_window()
        if window and move_window(window, x, y):
            print(f"Recentered RViz window 0x{window:x} to +{x}+{y}")
            return 0
        time.sleep(0.5)

    print("RViz window was not found for recentering", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
