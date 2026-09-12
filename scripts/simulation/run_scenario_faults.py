#!/usr/bin/python3
"""Replay a deterministic Phase 5 public-sensor fault schedule."""

import argparse
import json
import signal
import time
from pathlib import Path

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node
from scenario_config import load_scenario


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("--directory", required=True, type=Path)
    args = parser.parse_args()
    _, scenario = load_scenario(args.scenario)
    events = sorted(scenario["sensor_fault_schedule"], key=lambda item: item["start_s"])
    stop = False

    def stopping(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, stopping)
    signal.signal(signal.SIGTERM, stopping)
    node = Node()
    publisher = node.advertise("/icarus/test/sensor_fault", StringMsg)
    clock = {"simulation_s": None}

    def update_clock(message):
        clock["simulation_s"] = message.sim.sec + message.sim.nsec * 1e-9

    assert node.subscribe(Clock, "/clock", update_clock)
    wall_start = time.monotonic()
    origin_sim = None
    active, completed, log = {}, set(), []
    while not stop:
        if clock["simulation_s"] is None:
            time.sleep(0.01)
            continue
        if origin_sim is None:
            origin_sim = clock["simulation_s"]
        elapsed = clock["simulation_s"] - origin_sim
        for index, event in enumerate(events):
            if index not in active and index not in completed and elapsed >= event["start_s"]:
                active[index] = elapsed
                payload = {"channel": event["channel"], "mode": event["mode"]}
                if event["mode"] == "delay":
                    payload["latency_s"] = scenario["communication"]["delay_s"]
                publisher.publish(StringMsg(data=json.dumps(payload)))
                log.append({"simulation_elapsed_s": elapsed, "wall_elapsed_s": time.monotonic() - wall_start, "action": "start", **payload})
        for index in list(active):
            event = events[index]
            if elapsed >= event["start_s"] + event["duration_s"]:
                payload = {"channel": event["channel"], "mode": "normal"}
                publisher.publish(StringMsg(data=json.dumps(payload)))
                log.append({"simulation_elapsed_s": elapsed, "wall_elapsed_s": time.monotonic() - wall_start, "action": "recover", **payload})
                del active[index]
                completed.add(index)
        time.sleep(0.05)
    for index in list(active):
        publisher.publish(StringMsg(data=json.dumps({"channel": events[index]["channel"], "mode": "normal"})))
    args.directory.mkdir(parents=True, exist_ok=True)
    (args.directory / "scenario_faults.json").write_text(
        json.dumps({"configured": events, "events": log, "completed": len(completed)}, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
