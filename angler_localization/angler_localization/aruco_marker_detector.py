# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from typing import Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from gi.repository import Gst
from rclpy.node import Node


class ArucoMarkerDetector(Node):
    ARUCO_MARKER_TYPES = {
        "4X4_50": cv2.aruco.DICT_4X4_50,
        "4X4_100": cv2.aruco.DICT_4X4_100,
        "4X4_250": cv2.aruco.DICT_4X4_250,
        "4X4_1000": cv2.aruco.DICT_4X4_1000,
        "5X5_50": cv2.aruco.DICT_5X5_50,
        "5X5_100": cv2.aruco.DICT_5X5_100,
        "5X5_250": cv2.aruco.DICT_5X5_250,
        "5X5_1000": cv2.aruco.DICT_5X5_1000,
        "6X6_50": cv2.aruco.DICT_6X6_50,
        "6X6_100": cv2.aruco.DICT_6X6_100,
        "6X6_250": cv2.aruco.DICT_6X6_250,
        "6X6_1000": cv2.aruco.DICT_6X6_1000,
        "7X7_50": cv2.aruco.DICT_7X7_50,
        "7X7_100": cv2.aruco.DICT_7X7_100,
        "7X7_250": cv2.aruco.DICT_7X7_250,
        "7X7_1000": cv2.aruco.DICT_7X7_1000,
        "ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    }

    def __init__(self) -> None:
        super().__init__("aruco_marker_detector")

        self.declare_parameters(
            "",
            [
                ("port", 5600),
                ("camera_matrix", np.zeros((3, 3))),
                ("projection_matrix", np.zeros((3, 4))),
                ("distortion_coefficients", np.zeros((1, 5))),
            ],
        )

        self.visual_odom_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", 1
        )

        self.video_pipe, self.video_sink = self.init_stream(
            self.get_parameter("port").get_parameter_value().integer_value
        )

    def init_stream(self, port: int, marker_type: str) -> tuple[Any, Any]:
        Gst.init(None)

        video_source = f"udpsrc port={port}"
        video_codec = (
            "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264"
        )
        video_decode = (
            "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert"
        )
        video_sink_conf = (
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        )

        command = " ".join([video_source, video_codec, video_decode, video_sink_conf])

        video_pipe = Gst.parse_launch(command)
        video_pipe.set_state(Gst.State.PLAYING)

        video_sink = video_pipe.get_by_name("appsink0")

        video_sink.connect("new-sample", self.extract_and_publish_pose_cb)

        return video_pipe, video_sink

    @staticmethod
    def gst_to_opencv(sample: Any) -> np.ndarray:
        buf = sample.get_buffer()
        caps = sample.get_caps()

        return np.ndarray(
            (
                caps.get_structure(0).get_value("height"),
                caps.get_structure(0).get_value("width"),
                3,
            ),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )

    def extract_and_publish_pose_cb(self, sink: Any) -> Any:
        frame = self.gst_to_opencv(sink.emit("pull-sample"))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        pose_msg = PoseStamped()

        pose_msg.header.stamp = self.get_clock().now()
        pose_msg.header.frame_id = "map"

        (
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
        ) = [0, 0, 0]

        (
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ) = [0, 0, 0, 0]

        self.visual_odom_pub.publish(pose_msg)

        return Gst.FlowReturn.OK


def main(args=None):
    """Run the ArUco tag detector."""
    rclpy.init(args=args)

    node = ArucoMarkerDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    return


if __name__ == "__main__":
    main()
