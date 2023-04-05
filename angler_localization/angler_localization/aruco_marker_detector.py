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
import gi
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from gi.repository import Gst
from rclpy.node import Node


class ArucoMarkerDetector(Node):
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

        self.get_parameter("port").get_parameter_value().integer_value

        self.visual_odom_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", 1
        )

    def init_stream(self, port: int) -> None:
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

        # TODO(evan-palmer): Add callback here
        video_sink.connect("new-sample", self.extract_and_publish_pose_cb)

    def extract_and_publish_pose_cb(self, sink: Any) -> Any:
        frame = sink.emit("pull-sample")

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
