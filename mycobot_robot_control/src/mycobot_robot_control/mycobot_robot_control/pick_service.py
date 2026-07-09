#!/usr/bin/env python3
"""ROS2 service trigger for the existing pick_flow.py sequence.

Start this once, then call /pick_and_place whenever a pick-place cycle should run.
Runtime parameters mirror pick_flow.py defaults.
"""
import os
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class PickPlaceService(Node):
    def __init__(self):
        super().__init__("pick_place_service")
        self.busy = False

        self.declare_parameter("robot_ip", os.environ.get("ROBOT_IP", "192.168.123.50"))
        self.declare_parameter("pick_x", 0.18)
        self.declare_parameter("pick_y", 0.0)
        self.declare_parameter("rotate_deg", 90.0)
        self.declare_parameter("grasp_z", 0.02)
        self.declare_parameter("place_z", 0.06)
        self.declare_parameter("grip_val", 35)
        self.declare_parameter("speed", 25)

        self.script = Path(__file__).with_name("pick_flow.py")
        self.create_service(Trigger, "pick_and_place", self.handle_pick)
        self.get_logger().info("ready: call /pick_and_place to run one cycle")

    def _param(self, name):
        return self.get_parameter(name).value

    def handle_pick(self, request, response):
        del request
        if self.busy:
            response.success = False
            response.message = "busy: pick-and-place already running"
            return response

        grip_val = int(self._param("grip_val"))
        if grip_val < 25:
            response.success = False
            response.message = "refused: grip_val < 25 risks gripper stall-current brownout"
            return response

        args = [
            sys.executable,
            str(self.script),
            str(float(self._param("pick_x"))),
            str(float(self._param("pick_y"))),
            str(float(self._param("rotate_deg"))),
            str(float(self._param("grasp_z"))),
            str(float(self._param("place_z"))),
            str(grip_val),
            str(int(self._param("speed"))),
        ]

        env = os.environ.copy()
        env["ROBOT_IP"] = str(self._param("robot_ip"))
        self.get_logger().info("starting: " + " ".join(args[2:]) + f" ROBOT_IP={env['ROBOT_IP']}")

        self.busy = True
        lines = []
        try:
            proc = subprocess.Popen(
                args,
                cwd=str(self.script.parent),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert proc.stdout is not None
            for line in proc.stdout:
                line = line.rstrip()
                lines.append(line)
                self.get_logger().info(line)
            rc = proc.wait(timeout=10)
        except Exception as exc:
            response.success = False
            response.message = f"exception: {exc}"
            return response
        finally:
            self.busy = False

        response.success = rc == 0
        last = next((line for line in reversed(lines) if line), "")
        response.message = last if last else f"pick_flow exited rc={rc}"
        return response


def main():
    rclpy.init()
    node = PickPlaceService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
