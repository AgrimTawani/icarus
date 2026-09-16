#!/usr/bin/env python3
"""Run the frozen V1 mission set through the Drone API, never direct MAVLink."""

import json
import math
import time
import uuid
from pathlib import Path

from run_mission import MissionClient, action_pb2, state_pb2

ROOT = Path(__file__).resolve().parents[2]


def local_point(north, east, altitude, radius=1.5):
    return action_pb2.RoutePoint(
        position={
            "local_ned": {
                "north_m": north,
                "east_m": east,
                "down_m": -altitude,
                "origin_id": "ekf-origin",
            }
        },
        acceptance_radius_m=radius,
        heading=action_pb2.HeadingPolicy(mode=action_pb2.HEADING_MODE_FACE_PATH),
        limits=action_pb2.ActionLimits(
            maximum_ground_speed_mps=3,
            execution_timeout_ms=90_000,
        ),
    )


class Acceptance:
    def __init__(self):
        self.client = MissionClient("127.0.0.1:50051")
        self.results = []

    def action(self, name, request, timeout):
        receipt = getattr(self.client.action_api, name)(request, timeout=5)
        return self.client.wait_action(receipt, timeout)

    def arm(self, label):
        return self.action(
            "Arm",
            action_pb2.ArmRequest(context=self.client.context(label + ":arm")),
            55,
        )

    def disarm(self, label):
        return self.action(
            "Disarm",
            action_pb2.DisarmRequest(
                context=self.client.context(label + ":disarm")
            ),
            20,
        )

    def takeoff(self, label, altitude):
        return self.action(
            "Takeoff",
            action_pb2.TakeoffRequest(
                context=self.client.context(label + ":takeoff"),
                target_altitude_agl_m=altitude,
                heading=action_pb2.HeadingPolicy(
                    mode=action_pb2.HEADING_MODE_KEEP_CURRENT
                ),
                limits=action_pb2.ActionLimits(
                    maximum_climb_rate_mps=2,
                    execution_timeout_ms=60_000,
                ),
            ),
            70,
        )

    def hold(self, label, seconds):
        return self.action(
            "Hold",
            action_pb2.HoldRequest(
                context=self.client.context(label + ":hold"),
                duration_ms=int(seconds * 1000),
                heading=action_pb2.HeadingPolicy(
                    mode=action_pb2.HEADING_MODE_KEEP_CURRENT
                ),
            ),
            int(seconds) + 20,
        )

    def land(self, label):
        return self.action(
            "Land",
            action_pb2.LandRequest(context=self.client.context(label + ":land")),
            100,
        )

    def goto(self, label, north, east, altitude, radius=1.5):
        point = local_point(north, east, altitude, radius)
        return self.action(
            "Goto",
            action_pb2.GotoRequest(
                context=self.client.context(label + ":goto"),
                destination=point.position,
                acceptance_radius_m=point.acceptance_radius_m,
                heading=point.heading,
                limits=point.limits,
            ),
            100,
        )

    def run_case(self, name, function):
        started = time.monotonic()
        print(f"\n=== {name} ===", flush=True)
        try:
            function()
            result = {"mission": name, "status": "passed"}
        except Exception as error:
            result = {"mission": name, "status": "failed", "error": str(error)}
            raise
        finally:
            result["duration_s"] = time.monotonic() - started
            self.results.append(result)

    def run(self):
        self.client.connect()
        self.client.acquire()

        self.run_case("M01_arm_disarm", self.m01)
        self.run_case("M02_takeoff", self.m02)
        self.run_case("M03_hover", self.m03)
        self.run_case("M04_local_waypoint", self.m04)
        self.run_case("M05_short_route", self.m05)
        self.run_case("M06_return_home", self.m06)
        self.run_case("M07_land", self.m07)
        self.run_case("M08_cancel", self.m08)
        self.run_case("M09_reject_unsafe", self.m09)
        self.run_case("X01_orbit", self.orbit)

    def m01(self):
        self.arm("m01")
        time.sleep(3)
        self.disarm("m01")

    def m02(self):
        self.arm("m02")
        self.takeoff("m02", 5)
        self.hold("m02", 5)
        self.land("m02")

    def m03(self):
        self.arm("m03")
        self.takeoff("m03", 5)
        self.hold("m03", 20)
        self.land("m03")

    def m04(self):
        self.arm("m04")
        self.takeoff("m04", 5)
        state = self.client.state().local_position_ned
        self.goto("m04", state.north_m + 20, state.east_m, 5)
        self.hold("m04", 5)
        self.land("m04")

    def m05(self):
        self.arm("m05")
        self.takeoff("m05", 5)
        state = self.client.state().local_position_ned
        north, east = state.north_m, state.east_m
        route = action_pb2.ExecuteRouteRequest(
            context=self.client.context("m05:route"),
            route_id="m05-square",
            points=[
                local_point(north + 20, east, 5),
                local_point(north + 20, east + 20, 5),
                local_point(north, east + 20, 5),
                local_point(north, east, 5),
            ],
            completion_behavior=action_pb2.ROUTE_COMPLETION_BEHAVIOR_LAND,
        )
        self.action("ExecuteRoute", route, 180)

    def m06(self):
        self.arm("m06")
        self.takeoff("m06", 8)
        state = self.client.state().local_position_ned
        self.goto("m06", state.north_m + 30, state.east_m + 10, 8)
        self.action(
            "ReturnHome",
            action_pb2.ReturnHomeRequest(
                context=self.client.context("m06:return"),
                completion_behavior=(
                    action_pb2.RETURN_COMPLETION_BEHAVIOR_LAND_AT_HOME
                ),
            ),
            150,
        )

    def m07(self):
        self.arm("m07")
        self.takeoff("m07", 5)
        self.land("m07")

    def m08(self):
        self.arm("m08")
        self.takeoff("m08", 5)
        start = self.client.state().local_position_ned
        target_north = start.north_m + 40
        point = local_point(target_north, start.east_m, 5)
        receipt = self.client.action_api.Goto(
            action_pb2.GotoRequest(
                context=self.client.context("m08:goto"),
                destination=point.position,
                acceptance_radius_m=1.5,
                heading=point.heading,
                limits=point.limits,
            ),
            timeout=5,
        )
        deadline = time.monotonic() + 40
        travelled = 0.0
        while travelled < 5 and time.monotonic() < deadline:
            state = self.client.state().local_position_ned
            travelled = math.hypot(
                state.north_m - start.north_m, state.east_m - start.east_m
            )
            time.sleep(0.1)
        if travelled < 5:
            raise RuntimeError("vehicle did not travel far enough for cancellation")
        cancelled = self.client.action_api.CancelAction(
            action_pb2.CancelActionRequest(
                context=self.client.context("m08:cancel"),
                action_id=receipt.action_id,
            ),
            timeout=12,
        )
        if cancelled.disposition != action_pb2.ACTION_STATE_CANCELLED:
            raise RuntimeError("active waypoint was not cancelled safely")
        self.land("m08")

    def m09(self):
        self.arm("m09")
        state = self.client.state().local_position_ned
        point = local_point(state.north_m + 101, state.east_m, 5)
        receipt = self.client.action_api.Goto(
            action_pb2.GotoRequest(
                context=self.client.context("m09:unsafe"),
                destination=point.position,
                acceptance_radius_m=1.5,
                heading=point.heading,
                limits=point.limits,
            ),
            timeout=5,
        )
        if receipt.disposition != action_pb2.ACTION_STATE_REJECTED:
            raise RuntimeError("unsafe destination was not rejected")
        if receipt.reason_code != action_pb2.REASON_CODE_GEOFENCE_VIOLATION:
            raise RuntimeError("unsafe destination returned wrong reason code")
        self.disarm("m09")

    def orbit(self):
        self.arm("orbit")
        self.takeoff("orbit", 5)
        state = self.client.state().local_position_ned
        request = action_pb2.OrbitRequest(
            context=self.client.context("orbit:execute"),
            center={
                "local_ned": {
                    "north_m": state.north_m + 10,
                    "east_m": state.east_m,
                    "down_m": -5,
                    "origin_id": "ekf-origin",
                }
            },
            radius_m=5,
            altitude_m=5,
            altitude_frame=state_pb2.COORDINATE_FRAME_LOCAL_NED,
            ground_speed_mps=3,
            clockwise=True,
            revolutions=0.5,
            heading=action_pb2.HeadingPolicy(
                mode=action_pb2.HEADING_MODE_FACE_POINT
            ),
        )
        self.action("Orbit", request, 180)
        self.land("orbit")


def main():
    acceptance = Acceptance()
    status = "failed"
    try:
        acceptance.run()
        status = "passed"
        return 0
    except Exception as error:  # noqa: BLE001 - retain acceptance evidence
        print("ACCEPTANCE FAILED:", error, flush=True)
        return 1
    finally:
        acceptance.client.close()
        report = {
            "status": status,
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "missions": acceptance.results,
        }
        output = ROOT / "logs/phase8"
        output.mkdir(parents=True, exist_ok=True)
        path = output / f"v1_acceptance_{time.strftime('%Y%m%dT%H%M%S')}_{uuid.uuid4().hex[:6]}.json"
        path.write_text(json.dumps(report, indent=2) + "\n")
        print("Acceptance report:", path, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
