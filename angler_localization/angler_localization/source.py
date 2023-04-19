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

from abc import ABC
from typing import Any

import gi
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa


class Source(Node, ABC):
    """Base class for a localization source (e.g., camera, sonar, etc.)."""

    def __init__(self, node_name: str) -> None:
        """Create a new localization source.

        Args:
            node_name: The name of the ROS 2 node.
        """
        Node.__init__(self, node_name)
        ABC.__init__(self)


class Camera(Source):
    """BlueROV2 camera source.

    The camera source uses GStreamer to proxy the BlueROV2 camera stream (i.e., frames
    received from GStreamer are converted to ROS ``Image`` messages and republished for
    other packages to use).
    """

    def __init__(self) -> None:
        """Create a new Camera source."""
        super().__init__("camera")

        self.bridge = CvBridge()

        self.declare_parameter("port", 5600)

        self.camera_frame_pub = self.create_publisher(Image, "/blue/camera", 1)

        # Start the GStreamer stream
        self.video_pipe, self.video_sink = self.init_stream(
            self.get_parameter("port").get_parameter_value().integer_value
        )

    def init_stream(self, port: int) -> tuple[Any, Any]:
        """Initialize a GStreamer video stream interface.

        GStreamer is used to receive video frames from the BlueROV2 for processing.

        Args:
            port: The port over which the video feed is being streamed.

        Returns:
            The video pipe and sink.
        """
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

        video_sink.connect("new-sample", self.proxy_frame_cb)

        return video_pipe, video_sink

    @staticmethod
    def gst_to_opencv(frame: Any) -> np.ndarray:
        """Convert a GStreamer frame to an array.

        Args:
            frame: The GStreamer frame to convert.

        Returns:
            The GStreamer video frame as an array.
        """
        buf = frame.get_buffer()
        caps = frame.get_caps()

        return np.ndarray(
            (
                caps.get_structure(0).get_value("height"),
                caps.get_structure(0).get_value("width"),
                3,
            ),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )

    def proxy_frame_cb(self, sink: Any) -> Any:
        """Convert the GStreamer frame to a ROS 2 message and republish.

        Args:
            sink: The GStreamer video sink.

        Returns:
            The GStreamer response.
        """
        # Convert from a GStreamer frame to a ROS 2 message and publish
        frame = self.gst_to_opencv(sink.emit("pull-sample"))
        self.camera_frame_pub.publish(self.bridge.cv2_to_imgmsg(frame))

        return Gst.FlowReturn.OK


def main_camera(args: list[str] | None = None):
    """Run the camera source."""
    rclpy.init(args=args)

    node = Camera()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
