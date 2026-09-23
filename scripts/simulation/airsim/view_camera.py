#!/usr/bin/env python3
"""Display AirSim's off-screen RGB and depth camera feeds."""

import argparse
import os
import signal
import time
from pathlib import Path

import cv2
import numpy as np

from python.simulation.airsim_rpc import MsgpackRpcClient

WINDOW_NAME = "Icarus AirSim Camera — RGB | Depth"


def decode_rgb(response):
    encoded = response.get("image_data_uint8") or b""
    image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.size == 0:
        raise ValueError("AirSim returned an empty or invalid RGB image")
    return image


def colorize_depth(response, near_m, far_m):
    width = int(response.get("width", 0))
    height = int(response.get("height", 0))
    values = np.asarray(response.get("image_data_float") or [], dtype=np.float32)
    if width <= 0 or height <= 0 or values.size != width * height:
        raise ValueError("AirSim returned an empty or malformed depth image")
    depth = values.reshape(height, width)
    depth = np.nan_to_num(depth, nan=far_m, posinf=far_m, neginf=near_m)
    depth = np.clip(depth, near_m, far_m)
    # Near obstacles are warm; distant geometry is cool.
    normalized = ((far_m - depth) * (255.0 / (far_m - near_m))).astype(np.uint8)
    return cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)


def add_label(image, label):
    result = image.copy()
    cv2.rectangle(result, (0, 0), (result.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(result, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="Icarus")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--scale", type=float, default=3.0)
    parser.add_argument("--near-m", type=float, default=0.5)
    parser.add_argument("--far-m", type=float, default=50.0)
    parser.add_argument("--rgb-only", action="store_true")
    parser.add_argument("--no-window", action="store_true",
                        help="Fetch frames without opening a GUI (automation only).")
    parser.add_argument("--frames", type=int,
                        help="Exit after this many frames; default is continuous.")
    parser.add_argument("--output", type=Path,
                        help="Continuously replace this path with the latest grid image.")
    args = parser.parse_args()
    if not 0 < args.fps <= 60:
        parser.error("--fps must be in (0, 60]")
    if not 0.25 <= args.scale <= 8:
        parser.error("--scale must be in [0.25, 8]")
    if args.near_m < 0 or args.far_m <= args.near_m:
        parser.error("depth range must satisfy 0 <= near < far")
    if args.frames is not None and args.frames <= 0:
        parser.error("--frames must be positive")
    if not args.no_window and not os.environ.get("DISPLAY"):
        raise SystemExit("No graphical DISPLAY is available; use --no-window --output FILE for capture")

    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    requests = [{
        "camera_name": args.camera,
        "image_type": 0,
        "pixels_as_float": False,
        "compress": True,
    }]
    if not args.rgb_only:
        requests.append({
            "camera_name": args.camera,
            "image_type": 1,
            "pixels_as_float": True,
            "compress": False,
        })

    rpc = MsgpackRpcClient(args.host, args.port, timeout=10)
    interval = 1.0 / args.fps
    count = 0
    started = time.monotonic()
    last_timestamp = 0
    try:
        if not rpc.ping():
            raise RuntimeError("AirSim RPC ping returned false")
        vehicles = rpc.list_vehicles()
        if args.vehicle not in vehicles:
            raise RuntimeError(f"AirSim vehicle {args.vehicle!r} is absent: {vehicles}")
        print(f"Live AirSim camera: vehicle={args.vehicle} camera={args.camera}; "
              "press q, Esc, or Ctrl+C to close", flush=True)
        while not stopping:
            frame_started = time.monotonic()
            responses = rpc.images(requests, args.vehicle)
            if len(responses) != len(requests):
                raise RuntimeError("AirSim returned an incomplete camera response")
            timestamp = int(responses[0].get("time_stamp", 0))
            if timestamp <= 0 or (last_timestamp and timestamp < last_timestamp):
                raise RuntimeError("AirSim camera timestamp is missing or moved backwards")
            last_timestamp = timestamp
            rgb = add_label(decode_rgb(responses[0]), "Forward RGB")
            panels = [rgb]
            if not args.rgb_only:
                depth = colorize_depth(responses[1], args.near_m, args.far_m)
                panels.append(add_label(depth, f"Depth {args.near_m:g}-{args.far_m:g} m"))
            height = min(panel.shape[0] for panel in panels)
            panels = [cv2.resize(panel, (round(panel.shape[1] * height / panel.shape[0]), height))
                      if panel.shape[0] != height else panel for panel in panels]
            grid = cv2.hconcat(panels)
            if args.scale != 1:
                grid = cv2.resize(grid, None, fx=args.scale, fy=args.scale,
                                  interpolation=cv2.INTER_NEAREST)
            count += 1
            elapsed = max(time.monotonic() - started, 1e-6)
            cv2.putText(grid, f"{count / elapsed:.1f} fps", (8, grid.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55 * args.scale,
                        (255, 255, 255), max(1, round(args.scale)), cv2.LINE_AA)
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                temporary = args.output.with_name(args.output.stem + ".tmp" + args.output.suffix)
                if not cv2.imwrite(str(temporary), grid):
                    raise RuntimeError(f"Could not write camera frame to {temporary}")
                temporary.replace(args.output)
            if not args.no_window:
                cv2.imshow(WINDOW_NAME, grid)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27) or cv2.getWindowProperty(
                        WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
            if args.frames is not None and count >= args.frames:
                break
            time.sleep(max(0.0, interval - (time.monotonic() - frame_started)))
    except (ConnectionError, OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"AirSim camera viewer failed: {error}") from error
    finally:
        rpc.close()
        if not args.no_window:
            cv2.destroyAllWindows()
    print(f"AirSim camera viewer stopped after {count} frames", flush=True)


if __name__ == "__main__":
    main()
