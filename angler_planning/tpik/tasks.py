from abc import ABC, abstractmethod
from typing import Any

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3
from scipy.spatial.transform import Rotation as R
from tpik.constraint import EqualityConstraint, SetConstraint


def quaternion_to_rotation(quat: Quaternion) -> R:
    """Convert a Quaternion message into a SciPy Rotation.

    Args:
        quat: The Quaternion message to convert.

    Returns:
        The resulting SciPy Rotation.
    """
    return R.from_quat([quat.x, quat.y, quat.z, quat.w])


def point_to_array(point: Point | Vector3) -> np.ndarray:
    """Convert a Point message into a numpy array.

    Args:
        point: The Point message to convert.

    Returns:
        The resulting point as a 3x1 numpy array.
    """
    return np.array([point.x, point.y, point.z]).reshape((3, 1))


def calculate_quaternion_error(desired: Quaternion, actual: Quaternion) -> np.ndarray:
    """Calculate the error between two quaternions.

    Args:
        desired: The desired orientation.
        actual: The actual orientation.

    Returns:
        The angle between the two quaternions as a quaternion.
    """

    def quat_to_array(quat: Quaternion):
        return np.array([quat.x, quat.y, quat.z, quat.w])

    quat_d = quat_to_array(desired)
    quat_a = quat_to_array(actual)

    error_eps = (
        quat_a[3] * quat_d[:3]
        - quat_d[3] * quat_a[:3]
        + np.cross(quat_a[:3], quat_d[:3])
    )
    error_lambda = quat_d[3] * quat_a[3] + quat_d[:3].T @ quat_a[:3]

    return np.hstack((error_eps, error_lambda))


def calculate_vehicle_angular_velocity_jacobian(
    rot_map_to_base: Quaternion,
) -> np.ndarray:
    """Calculate the angular velocity Jacobian for the vehicle.

    This is defined by Gianluca Antonelli in his textbook "Underwater Robotics" in
    Equation 2.3, and is denoted there as "J_k,o".

    Args:
        rot_map_to_base: The quaternion describing the rotation from the inertial frame
            to the vehicle-fixed frame (map -> base_link).

    Returns:
        The vehicle's angular velocity Jacobian.
    """
    rot = quaternion_to_rotation(rot_map_to_base)
    roll, pitch, _ = rot.as_euler("xyz")

    return np.array(
        [
            [1, 0, -np.sin(pitch)],
            [0, np.cos(roll), np.cos(pitch) * np.sin(roll)],
            [0, -np.sin(roll), np.cos(pitch) * np.cos(roll)],
        ]  # type: ignore
    )


def calculate_manipulator_jacobian(
    serial_chain: Any, joint_angles: np.ndarray
) -> np.ndarray:
    """Generate the manipulator Jacobian using a kinpy serial chain.

    Args:
        serial_chain: The kinpy serial chain.
        joint_angles: The current joint angles.

    Returns:
        The manipulator Jacobian matrix.
    """
    return np.array(serial_chain.jacobian(joint_angles))


class TaskFactory(ABC):
    """Base class for a class which implements the factory pattern."""

    @abstractmethod
    def create_task_from_params(self, *args, **kwargs) -> Any:
        """Create a new constraint from a configuration file.

        Raises:
            NotImplementedError: This method has not yet been implemented.

        Returns:
            A new constraint.
        """
        raise NotImplementedError("This method has not yet been implemented!")


class VehicleRollPitch(EqualityConstraint, TaskFactory):
    """Control the vehicle roll and pitch angles."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle roll pitch task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        EqualityConstraint.__init__(self, "vehicle_roll_pitch", gain, priority)

        self.desired_rot = np.zeros((2, 1))
        self.current_rot = np.zeros((2, 1))
        self.n_manipulator_joints = 0
        self.rot_map_to_base = Quaternion()

    def create_task_from_params(
        self,
        gain: float,
        priority: float,
        roll: float | None = None,
        pitch: float | None = None,
    ) -> Any:
        """Create a vehicle roll-pitch task from a set of parameters.

        Args:
            gain: The task gain.
            priority: The task priority.
            roll: The desired roll angle, if known beforehand. Defaults to None.
            pitch: The desired pitch angle, if known beforehand. Defaults to None.

        Returns:
            A vehicle roll-pitch task.
        """
        task = VehicleRollPitch(gain, priority)

        if None not in [roll, pitch]:
            task.desired_rot = np.array([roll, pitch]).reshape((2, 1))

        return task

    def update(
        self,
        current_rot: Quaternion,
        desired_rot: Quaternion | None = None,
        n_manipulator_joints: int | None = None,
    ) -> None:
        """Update the task context.

        Args:
            current_rot: The current vehicle orientation in the inertial (map)
                frame.
            desired_rot: The desired vehicle orientation. The desired roll and
                pitch angles will be extracted from the rotation. Defaults to None.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.rot_map_to_base = current_rot

        # Remove the yaw from the quaternions and get the roll, pitch angles
        cr = R.from_quat(
            [
                current_rot.x,
                current_rot.y,
                current_rot.z,
                current_rot.w,
            ]
        )
        current_roll, current_pitch, _ = cr.as_euler("xyz")
        self.current_rot = np.array([current_roll, current_pitch]).reshape((2, 1))

        if desired_rot is not None:
            dr = R.from_quat(
                [
                    desired_rot.x,
                    desired_rot.y,
                    desired_rot.z,
                    desired_rot.w,
                ]
            )
            desired_roll, desired_pitch, _ = dr.as_euler("xyz")
            self.desired_rotation_rotation = np.array(
                [desired_roll, desired_pitch]
            ).reshape((2, 1))

        # The number of manipulator joints doesn't need to change every update
        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the vehicle roll-pitch task Jacobian.

        Returns:
            The Jacobian for a task which controls the vehicle roll and pitch.
        """
        J_ko = calculate_vehicle_angular_velocity_jacobian(self.rot_map_to_base)

        J = np.zeros((2, 6 + self.n_manipulator_joints))
        J[:, 3:6] = np.array([[1, 0, 0], [0, 1, 0]]) @ np.linalg.pinv(J_ko)

        return J

    @property
    def reference(self) -> np.ndarray:
        """Calculate the reference signal for the vehicle roll-pitch task.

        Args:
            current_orientation: The current vehicle orientation--the yaw will be
                ignored when calculating the reference.

        Returns:
            The reference signal.
        """
        return self._calculate_reference(
            np.zeros((2, 1)), self.desired_rot - self.current_rot
        )


class VehicleYaw(EqualityConstraint, TaskFactory):
    """Control the vehicle yaw angle."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle yaw task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        EqualityConstraint.__init__(self, "vehicle_yaw", gain, priority)

        self.desired_rot = np.zeros((1, 1))
        self.rot_map_to_base = Quaternion()
        self.n_manipulator_joints = 0

    def create_task_from_params(
        self, gain: float, priority: float, yaw: float | None = None
    ) -> Any:
        """Create a new vehicle yaw task from a set of parameters..

        Args:
            gain: The task gain.
            priority: The task priority.
            yaw: The desired vehicle yaw, if known beforehand. Defaults to None.
        """
        task = VehicleYaw(gain, priority)

        if yaw is not None:
            task.desired_rot = np.array([yaw])

        return task

    def update(
        self,
        current_rot: Quaternion,
        desired_rot: Quaternion | None = None,
        n_manipulator_joints: int | None = None,
    ) -> None:
        """Update the context for the yaw task.

        Args:
            current_rot: The current vehicle orientation in the inertial (map)
                frame.
            desired_rot: The desired vehicle orientation--the desired yaw angle
                will be extracted from the quaternion. Defaults to None.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.rot_map_to_base = current_rot

        # Remove the roll, pitch from the quaternion and get the yaw angle
        cr = R.from_quat(
            [
                current_rot.x,
                current_rot.y,
                current_rot.z,
                current_rot.w,
            ]
        )
        _, _, current_yaw = cr.as_euler("xyz")
        self.current_rotation = np.array([current_yaw])

        if desired_rot is not None:
            dr = R.from_quat(
                [
                    desired_rot.x,
                    desired_rot.y,
                    desired_rot.z,
                    desired_rot.w,
                ]
            )
            _, _, desired_yaw = dr.as_euler("xyz")
            self.desired_rot = np.array([desired_yaw])

        # The total number of manipulator joints doesn't need to be updated at each
        # iteration
        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the vehicle yaw task Jacobian.

        Returns:
            The Jacobian for a task which controls the vehicle yaw.
        """
        J_ko = calculate_vehicle_angular_velocity_jacobian(self.rot_map_to_base)

        J = np.zeros((1, 6 + self.n_manipulator_joints))
        J[:, 3:6] = np.array([0, 0, 1]) @ np.linalg.inv(J_ko)

        return J

    @property
    def reference(self) -> np.ndarray:
        """Calculate the reference signal for the vehicle yaw task.

        Returns:
            The reference signal.
        """
        return self._calculate_reference(
            np.zeros((2, 1)), self.desired_rot - self.current_rotation
        )


class VehicleOrientation(EqualityConstraint, TaskFactory):
    """Control the vehicle orientation."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle orientation task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        EqualityConstraint.__init__(self, "vehicle_orientation", gain, priority)

        self.current_rot = Quaternion()
        self.desired_rot = Quaternion()
        self.rot_base_to_map = Quaternion()
        self.n_manipulator_joints = 0

    def create_task_from_params(
        self,
        gain: float,
        priority: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
    ) -> Any:
        """Create a new vehicle orientation task.

        Args:
            gain: The task gain.
            priority: The task priority.
            roll: The desired vehicle roll, if known beforehand. Defaults to
                None.
            pitch: The desired vehicle pitch, if known beforehand. Defaults to None.
            yaw: The desired vehicle yaw, if known beforehand. Defaults to None.
        """
        task = VehicleOrientation(gain, priority)

        if None not in [roll, pitch, yaw]:
            dr = R.from_euler("xyz", [roll, pitch, yaw])  # type: ignore
            desired_quat = Quaternion()
            (
                desired_quat.x,
                desired_quat.y,
                desired_quat.z,
                desired_quat.w,
            ) = dr.as_quat(False)

            task.desired_rot = desired_quat

        return task

    def update(
        self,
        current_rot: Quaternion,
        desired_rot: Quaternion | None = None,
        n_manipulator_joints: int | None = None,
    ) -> None:
        """Update the vehicle orientation context.

        Args:
            current_rot: The current vehicle orientation.
            desired_rot: The desired vehicle orientation. Defaults to None.
            n_manipulator_joints: The total number of manipulator joints. Defaults to
                None.
        """
        self.rot_base_to_map = current_rot
        self.current_rot = current_rot

        if desired_rot is not None:
            self.desired_rot = desired_rot

        if n_manipulator_joints is not None:
            n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the vehicle orientation Jacobian.

        Returns:
            The vehicle orientation Jacobian.
        """
        J = np.zeros((3, 6 + self.n_manipulator_joints))
        J[:, 3:6] = quaternion_to_rotation(self.rot_base_to_map).as_matrix()

        return J

    @property
    def reference(self) -> np.ndarray:
        """Calculate the reference signal for the vehicle orientation.

        Returns:
            The reference signal needed to move the vehicle to the desired orientation.
        """
        return super()._calculate_reference(
            np.zeros((3, 1)),
            calculate_quaternion_error(self.desired_rot, self.current_rot)[:3].reshape(
                (3, 1)
            ),
        )


class EndEffectorPose(EqualityConstraint, TaskFactory):
    """Control the end-effector pose."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new end effector pose task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        EqualityConstraint.__init__(self, "end_effector_pose", gain, priority)

        self.serial_chain: Any | None = None
        self.joint_angles = np.zeros((1, 1))
        self.transform_map_to_base = Transform()
        self.transform_manipulator_base_to_base = Transform()
        self.transform_map_to_end_effector = Transform()
        self.current_pose = Pose()
        self.desired_pose = Pose()

    def create_task_from_params(
        self,
        gain: float,
        priority: float,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
    ) -> Any:
        """Create a new end effector pose task from a set of parameters.

        Args:
            gain: The task gain.
            priority: The task priority.
            x: The desired end effector x position in the inertial (map) frame. Defaults
                to None.
            y: The desired end effector y position in the inertial (map) frame. Defaults
                to None.
            z: The desired end effector z position in the inertial (map) frame. Defaults
                to None.
            roll: The desired end effector roll in the inertial (map) frame. Defaults
                to None.
            pitch: The desired end effector pitch in the inertial (map) frame. Defaults
                to None.
            yaw: The desired end effector yaw in the inertial (map) frame. Defaults
                to None.

        Returns:
            A new end effector pose task.
        """
        task = EndEffectorPose(gain, priority)

        if None not in [x, y, z, roll, pitch, yaw]:
            task.desired_pose.position.x = x
            task.desired_pose.position.y = y
            task.desired_pose.position.z = z

            (
                task.desired_pose.orientation.x,
                task.desired_pose.orientation.y,
                task.desired_pose.orientation.z,
                task.desired_pose.orientation.w,
            ) = R.from_euler(
                "xyz", [roll, pitch, yaw]  # type: ignore
            ).as_quat(
                False
            )

        return task

    def update(
        self,
        joint_angles: np.ndarray,
        current_pose: Pose,
        transform_manipulator_base_to_base: Transform,
        transform_map_to_end_effector: Transform,
        serial_chain: Any | None = None,
        desired_pose: Pose | None = None,
    ) -> None:
        """Update the current context of the end effector pose task.

        Args:
            joint_angles: The current manipulator joint angles.
            current_pose: The current vehicle pose in the inertial (map) frame.
            transform_manipulator_base_to_base: The transformation from the manipulator
                base frame to the vehicle base frame.
            transform_map_to_end_effector: The transformation from the map frame to the
                end effector frame.
            serial_chain: The manipulator kinpy serial chain. Defaults to None.
            desired_pose: The desired end effector pose. Defaults to None.
        """
        self.joint_angles = joint_angles
        self.current_pose = current_pose

        # The transformation from the map to base_link frame is just the current pose
        self.transform_map_to_base.translation = current_pose.position
        self.transform_map_to_base.rotation = current_pose.orientation

        self.transform_manipulator_base_to_base = transform_manipulator_base_to_base
        self.transform_map_to_end_effector = transform_map_to_end_effector

        if serial_chain is not None:
            self.serial_chain = serial_chain

        if desired_pose is not None:
            self.desired_pose = desired_pose

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the UVMS Jacobian.

        Returns:
            The UVMS Jacobian.
        """
        # Get the transformation translations
        # denoted as r_{from frame}{to frame}_{with respecto x frame}
        r_MB_M = point_to_array(self.transform_map_to_base.translation)
        r_0B_B = point_to_array(self.transform_manipulator_base_to_base.translation)
        r_Mee_M = point_to_array(self.transform_map_to_end_effector.translation)

        # Get the transformation rotations
        # denoted as R_{from frame}_{to frame}
        R_0_B = quaternion_to_rotation(
            self.transform_manipulator_base_to_base.rotation
        ).as_matrix()
        R_B_M = np.linalg.inv(
            quaternion_to_rotation(self.transform_map_to_base.rotation).as_matrix()
        )
        R_0_M = R_B_M @ R_0_B

        r_B0_M = R_B_M @ r_0B_B
        r_0ee_M = r_Mee_M - r_MB_M - r_B0_M  # type: ignore

        def get_skew_matrix(x: np.ndarray) -> np.ndarray:
            return np.array(
                [[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]],  # type: ignore
            )

        J = np.zeros((6, 6 + len(self.joint_angles)))
        J_man = calculate_manipulator_jacobian(self.serial_chain, self.joint_angles)

        # Position Jacobian
        J[:3, :3] = R_B_M
        J[:3, 3:6] = -(get_skew_matrix(r_B0_M) + get_skew_matrix(r_0ee_M)) @ R_B_M  # type: ignore # noqa
        J[:3, 6:] = R_0_M @ J_man[:3]

        # Orientation Jacobian
        J[3:6, 3:6] = R_B_M
        J[3:6, 6:] = R_0_M @ J_man[3:6]

        return J

    @property
    def reference(self) -> np.ndarray:
        """Calculate the reference signal for the controller.

        Returns:
            The reference signal to use to drive the system to the desired end effector
                pose.
        """
        pos_error = np.array(
            [
                self.desired_pose.position.x - self.current_pose.position.x,
                self.desired_pose.position.y - self.current_pose.position.y,
                self.desired_pose.position.z - self.current_pose.position.z,
            ]
        ).reshape((3, 1))
        ori_error = calculate_quaternion_error(
            self.desired_pose.orientation, self.current_pose.orientation
        )[:3].reshape((3, 1))

        error = np.vstack((pos_error, ori_error))

        return super()._calculate_reference(np.zeros((6, 1)), error)


class JointLimit(SetConstraint, TaskFactory):
    """Limit a joint angle to a desired range."""

    def __init__(
        self,
        upper: float,
        lower: float,
        activation_threshold: float,
        deactivation_threshold: float,
        gain: float,
        priority: float,
        joint: int,
    ) -> None:
        """Create a new joint limit task.

        Args:
            upper: The joint angle upper limit.
            lower: The joint angle lower limit.
            activation_threshold: The task activation threshold.
            deactivation_threshold: The task deactivation threshold.
            gain: The task gain.
            priority: The task priority.
            joint: The joint that the constraint applies to.
        """
        super().__init__(
            "joint_limit_set",
            upper,
            lower,
            activation_threshold,
            deactivation_threshold,
            gain,
            priority,
        )

        self.joint = joint
        self.num_manipulator_joints = 0
        self.current_angle = 0.0
        self.desired_angle = 0.0

    def create_task_from_params(
        self,
        upper: float,
        lower: float,
        activation_threshold: float,
        deactivation_threshold: float,
        gain: float,
        priority: float,
        joint: int,
    ) -> Any:
        """Create a new joint limit task.

        Args:
            upper: The joint angle upper limit.
            lower: The joint angle lower limit.
            activation_threshold: The task activation threshold.
            deactivation_threshold: The task deactivation threshold.
            gain: The task gain.
            priority: The task priority.
            joint: The joint that the constraint applies to.
        """
        return JointLimit(
            upper,
            lower,
            activation_threshold,
            deactivation_threshold,
            gain,
            priority,
            joint,
        )

    def update(
        self,
        current_angle: float,
        desired_angle: float,
        n_manipulator_joints: int | None = None,
    ) -> None:
        """Update the current context of the joint limit task.

        Args:
            current_angle: The current joint angle.
            desired_angle: The desired joint angle. This is required because the task
                will be activated with a desired angle.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.current_angle = current_angle

        if desired_angle is not None:
            self.desired_angle = desired_angle

        if n_manipulator_joints is not None:
            self.num_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the Jacobian for a joint limit.

        Args:
            num_manipulator_joints: The total number of joints that the manipulator has.

        Returns:
            The joint limit Jacobian.
        """
        J = np.zeros((1, 6 + self.num_manipulator_joints))
        J[self.joint] = 1

        return J

    @property
    def reference(self) -> np.ndarray:
        """Calculate the reference signal for the joint limit task.

        Returns:
            The reference signal to use to drive the joint back to a safe range.
        """
        return super()._calculate_reference(
            np.zeros((1, 1)), np.array([self.desired_angle - self.current_angle])
        )


class ManipulatorConfiguration(EqualityConstraint, TaskFactory):
    """Control the joint angles of the manipulator."""

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new manipulator configuration task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        EqualityConstraint.__init__(self, "manipulator_configuration", gain, priority)

        self.current_joint_angles = np.zeros((1, 1))
        self.desired_joint_angles = np.zeros((1, 1))
        self.n_manipulator_joints = 0

    def create_task_from_params(
        self, gain: float, priority: float, desired_joint_angles: list[float] | None
    ) -> Any:
        """Create a new manipulator configuration task from a set of parameters.

        Args:
            gain: The task gain.
            priority: The task priority.
            desired_joint_angles: The desired joint angles. Defaults to None.

        Returns:
            A new manipulator configuration task.
        """
        task = ManipulatorConfiguration(gain, priority)

        if desired_joint_angles is not None:
            task.desired_joint_angles = np.array(desired_joint_angles)

        return task

    def update(
        self,
        current_joint_angles: np.ndarray,
        desired_joint_angles: np.ndarray | None = None,
        n_manipulator_joints: int | None = None,
    ) -> None:
        """Update the current context for the manipulator configuration task.

        Args:
            current_joint_angles: The current joint angles.
            desired_joint_angles: The desired joint angles. Defaults to None.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.current_joint_angles = current_joint_angles

        if desired_joint_angles is not None:
            self.desired_joint_angles = desired_joint_angles

        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the Jacobian for the manipulator joint configuration task.

        Returns:
            The joint configuration Jacobian.
        """
        J = np.zeros((1, 6 + self.n_manipulator_joints))
        J[6:] = 1

        return J

    @property
    def reference(self) -> np.ndarray:
        """Calculate the reference for the joint configuration task.

        Returns:
            The joint configuration reference signal.
        """
        return super()._calculate_reference(
            np.zeros((6, 1)), self.desired_joint_angles - self.current_joint_angles
        )
