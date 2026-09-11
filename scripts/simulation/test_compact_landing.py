#!/usr/bin/python3
"""Numeric drop/contact and visual-invariance tests, with no renderer/cameras."""

import copy
import json
import os
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from sensor_profiles import ROOT
from test_mark4_motors import stop


def case(strip_visuals, directory):
    os.environ["GZ_PARTITION"] = "icarus_drop_" + uuid.uuid4().hex
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "simulation/models")
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    tree = ET.Element("sdf", version="1.9")
    world = ET.SubElement(tree, "world", name="contact_test")
    ET.SubElement(world, "gravity").text = "0 0 -9.80665"
    physics = ET.SubElement(world, "physics", name="physics", type="ignored")
    ET.SubElement(physics, "max_step_size").text = ".001"
    ET.SubElement(physics, "real_time_factor").text = "1"
    ET.SubElement(
        world,
        "plugin",
        filename="gz-sim-physics-system",
        name="gz::sim::systems::Physics",
    )
    world.append(
        ET.fromstring(
            '<model name="ground"><static>true</static><link name="ground_link"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></collision></link></model>'
        )
    )
    model = copy.deepcopy(
        ET.parse(ROOT / "simulation/models/akshu_compact_sitl/model.sdf")
        .getroot()
        .find("model")
    )
    for plugin in list(model.findall("plugin")):
        model.remove(plugin)
    for link in model.findall("link"):
        for sensor in list(link.findall("sensor")):
            link.remove(sensor)
        if strip_visuals:
            for visual in list(link.findall("visual")):
                link.remove(visual)
    ET.SubElement(model, "pose").text = "0 0 .8 0 0 0"
    publisher = ET.SubElement(
        model,
        "plugin",
        filename="gz-sim-pose-publisher-system",
        name="gz::sim::systems::PosePublisher",
    )
    for key, value in {
        "publish_model_pose": "true",
        "publish_link_pose": "false",
        "use_pose_vector_msg": "true",
        "update_frequency": "1000",
        "topic": "/contact/pose",
    }.items():
        ET.SubElement(publisher, key).text = value
    world.append(model)
    ET.ElementTree(tree).write(directory / "world.sdf", encoding="unicode")
    node = Node()
    samples = []

    def receive(msg):
        for p in msg.pose:
            if p.name == "icarus_compact":
                stamp = p.header.stamp.sec + p.header.stamp.nsec * 1e-9
                samples.append([stamp, p.position.z, p.orientation.x, p.orientation.y])

    assert node.subscribe(Pose_V, "/contact/pose", receive)
    with (directory / "gazebo.log").open("w") as log:
        proc = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "-v", "2", str(directory / "world.sdf")],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 15
            while not samples or samples[-1][0] < 5:
                assert proc.poll() is None and time.monotonic() < deadline, len(samples)
                time.sleep(0.05)
        finally:
            stop(proc)
    assert len(samples) > 1500, len(samples)
    (directory / "numeric_samples.json").write_text(json.dumps(samples)+"\n")
    contact = next(i for i, s in enumerate(samples) if s[1] < 0.1902)
    # DART resolves its small impact penetration with bounded correction speed.
    # Allow this correction to finish before applying the sub-millimetre rest gate.
    rest = [s[1] for s in samples if s[0] > 4]
    assert min(s[1] for s in samples) > 0.1821
    rebound = max(s[1] for s in samples[contact:]) - 0.1901
    assert rebound < 0.025, rebound
    assert max(rest) - min(rest) < 0.001 and abs(sum(rest) / len(rest) - 0.1901) < 0.001, (min(rest),max(rest),sum(rest)/len(rest))
    result = {
        "status": "passed",
        "visuals_removed": strip_visuals,
        "samples": len(samples),
        "minimum_z_m": min(s[1] for s in samples),
        "maximum_height_after_contact_m": rebound,
        "rest_height_m": sum(rest) / len(rest),
    }
    (directory / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    directory = ROOT / "logs/simulation" / ("compact_landing_" + uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    results = []
    # Separate processes are needed for independent Gazebo transport partitions.
    for strip in (False, True):
        part = directory / str(strip)
        part.mkdir()
        subprocess.run(
            ["/usr/bin/python3", __file__, "--case", str(int(strip)), str(part)],
            check=True,
        )
        results.append(json.loads((part / "results.json").read_text()))
    assert abs(results[0]["rest_height_m"] - results[1]["rest_height_m"]) < 1e-8
    (directory / "results.json").write_text(
        json.dumps({"status": "passed", "cases": results}, indent=2) + "\n"
    )
    print(directory)
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    import sys
    from pathlib import Path

    if len(sys.argv) == 4:
        case(bool(int(sys.argv[2])), Path(sys.argv[3]))
    else:
        main()
