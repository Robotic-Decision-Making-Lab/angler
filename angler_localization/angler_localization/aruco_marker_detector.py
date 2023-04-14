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
import tf2_geometry_msgs  # noqa
import tf_transformations as tf
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa


class ArucoMarkerDetector(Node):
    """ArUco marker pose estimator.

    The ArUco marker detector detects ArUco markers in the BlueROV2 camera stream
    and estimates the pose of the BlueROV2 relative to the tag. The resulting pose
    is sent to the ArduSub EKF.
    """

    ARUCO_MARKER_TYPES = [
        cv2.aruco.DICT_4X4_50,
        cv2.aruco.DICT_4X4_100,
        cv2.aruco.DICT_4X4_250,
        cv2.aruco.DICT_4X4_1000,
        cv2.aruco.DICT_5X5_50,
        cv2.aruco.DICT_5X5_100,
        cv2.aruco.DICT_5X5_250,
        cv2.aruco.DICT_5X5_1000,
        cv2.aruco.DICT_6X6_50,
        cv2.aruco.DICT_6X6_100,
        cv2.aruco.DICT_6X6_250,
        cv2.aruco.DICT_6X6_1000,
        cv2.aruco.DICT_7X7_50,
        cv2.aruco.DICT_7X7_100,
        cv2.aruco.DICT_7X7_250,
        cv2.aruco.DICT_7X7_1000,
        cv2.aruco.DICT_ARUCO_ORIGINAL,
    ]

    def __init__(self) -> None:
        """Create a new ArUco marker detector."""
        super().__init__("aruco_marker_detector")

        self.declare_parameters(
            "",
            [
                ("port", 5600),
                ("camera_matrix", [0.0 for _ in range(9)]),  # Reshaped to 3x3
                ("projection_matrix", [0.0 for _ in range(12)]),  # Reshaped to 3x4
                ("distortion_coefficients", [0.0 for _ in range(5)]),
            ],
        )

        # Get the camera intrinsics
        self.camera_matrix = np.array(
            self.get_parameter("camera_matrix")
            .get_parameter_value()
            .double_array_value,
            np.float32,
        ).reshape(3, 3)

        self.projection_matrix = np.array(
            self.get_parameter("projection_matrix")
            .get_parameter_value()
            .double_array_value,
            np.float32,
        ).reshape(3, 4)

        self.distortion_coefficients = np.array(
            self.get_parameter("distortion_coefficients")
            .get_parameter_value()
            .double_array_value,
            np.float32,
        ).reshape(1, 5)

        self.visual_odom_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", 1
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Start the GStreamer stream
        self.video_pipe, self.video_sink = self.init_stream(
            self.get_parameter("port").get_parameter_value().integer_value
        )

    def init_stream(self, port: int) -> tuple[Any, Any]:
        """Initialize a GStreamer video stream interface.

        GStreamer is used to receive video frames from the BlueROV2 for processing.

        Args:
            port: The port at which the video feed is being streamed on.

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

        video_sink.connect("new-sample", self.extract_and_publish_pose_cb)

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

    def detect_markers(self, frame: np.ndarray) -> tuple[Any, Any] | None:
        """Detect any ArUco markers in the frame.

        All markers in a frame should be the same type of ArUco marker
        (e.g., 4x4 50) if multiple are expected to be in-frame.

        Args:
            frame: The video frame containing ArUco markers.

        Returns:
            A list of marker corners and IDs. If no markers were found, returns None.
        """
        # Check each tag type, breaking when we find one that works
        for tag_type in self.ARUCO_MARKER_TYPES:
            aruco_dict = cv2.aruco.Dictionary_get(tag_type)
            aruco_params = cv2.aruco.DetectorParameters_create()

            try:
                # Return the corners and ids if we find the correct tag type
                corners, ids, _ = cv2.aruco.detectMarkers(
                    frame, aruco_dict, parameters=aruco_params
                )

                if len(ids) > 0:
                    return corners, ids

            except Exception:
                continue

        # Nothing was found
        return None

    def get_camera_pose(self, frame: np.ndarray) -> tuple[Any, Any, int] | None:
        """Get the pose of the camera relative to any ArUco markers detected.

        If multiple markers are detected, then the "largest" marker will be used to
        determine the pose of the camera.

        Args:
            frame: The camera frame containing ArUco markers.

        Returns:
            The rotation vector and translation vector of the camera in the marker
            frame and the ID of the marker detected. If no marker was detected,
            returns None.
        """
        # Convert to greyscale image then try to detect the tag(s)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detection = self.detect_markers(gray)

        if detection is None:
            return None

        corners, ids = detection

        # If there are multiple markers, get the marker with the "longest" side, where
        # "longest" should be interpreted as the relative size in the image
        side_lengths = [
            abs(corner[0][0][0] - corner[0][2][0])
            + abs(corner[0][0][1] - corner[0][2][1])
            for corner in corners
        ]

        min_side_idx = side_lengths.index(max(side_lengths))
        min_marker_id = ids[min_side_idx]

        # Get the estimated pose
        rot_vec, trans_vec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners[min_side_idx],
            min_marker_id,
            self.camera_matrix,
            self.distortion_coefficients,
        )

        return rot_vec, trans_vec, min_marker_id

    def extract_and_publish_pose_cb(self, sink: Any) -> Any:
        """Get the camera pose relative to the marker and send to the ArduSub EKF.

        Args:
            sink: The GStreamer video sink.

        Returns:
            The GStreamer response.
        """
        # Convert from a GStreamer frame to a numpy array
        frame = self.gst_to_opencv(sink.emit("pull-sample"))

        # Get the pose of the camera in the `marker` frame
        camera_pose = self.get_camera_pose(frame)

        # If there was no marker in the image, exit early
        if camera_pose is None:
            self.get_logger().debug(
                "An ArUco marker could not be detected in the current image"
            )
            return Gst.FlowReturn.OK

        rot_vec, trans_vec, marker_id = camera_pose

        # Convert the pose into a PoseStamped message
        pose = PoseStamped()

        pose.header.frame_id = f"marker_{marker_id}"
        pose.header.stamp = self.get_clock().now().to_msg()

        (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ) = trans_vec.squeeze()

        tf_mat = np.identity(4)
        tf_mat[:3, :3], _ = cv2.Rodrigues(rot_vec)

        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = tf.quaternion_from_matrix(tf_mat)

        # Transform the pose from the `marker` frame to the `map` frame
        try:
            pose = self.tf_buffer.transform(pose, "map")
        except TransformException as e:
            self.get_logger().warning(
                f"Could not transform from frame marker_{marker_id} to map: {e}"
            )
            return Gst.FlowReturn.OK

        # Publish the transformed pose
        self.visual_odom_pub.publish(pose)

        return Gst.FlowReturn.OK


def main(args=None):
    """Run the ArUco marker detector."""
    rclpy.init(args=args)

    node = ArucoMarkerDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
