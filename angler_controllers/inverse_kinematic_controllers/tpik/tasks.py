from typing import Any

import numpy as np
from geometry_msgs.msg import Point, Quaternion, Transform, Vector3
from scipy.spatial.transform import Rotation as R
from tpik.constraint import EqualityTask, SetTask


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


class VehicleRollPitch(EqualityTask):
    """Control the vehicle roll and pitch angles."""

    name = "vehicle_roll_pitch_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle roll pitch task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.desired_value = np.zeros((2, 1))
        self.current_value = np.zeros((2, 1))
        self.n_manipulator_joints = 0
        self.rot_map_to_base = Quaternion()

    @staticmethod
    def create_task_from_params(
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
            task.desired_value = np.array([roll, pitch]).reshape((2, 1))

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
        self.current_value = np.array([current_roll, current_pitch]).reshape((2, 1))

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
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the vehicle roll-pitch task.

        Returns:
            The reference signal.
        """
        return self.desired_value - self.current_value  # type: ignore


class VehicleYaw(EqualityTask):
    """Control the vehicle yaw angle."""

    name = "vehicle_yaw_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle yaw task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.current_value = np.zeros((1, 1))
        self.desired_value = np.zeros((1, 1))
        self.rot_map_to_base = Quaternion()
        self.n_manipulator_joints = 0

    @staticmethod
    def create_task_from_params(
        gain: float, priority: float, yaw: float | None = None
    ) -> Any:
        """Create a new vehicle yaw task from a set of parameters.

        Args:
            gain: The task gain.
            priority: The task priority.
            yaw: The desired vehicle yaw, if known beforehand. Defaults to None.
        """
        task = VehicleYaw(gain, priority)

        if yaw is not None:
            task.desired_value = np.array([yaw])

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
        self.current_value = np.array([current_yaw])

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
            self.desired_value = np.array([desired_yaw])

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
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the vehicle yaw task.

        Returns:
            The reference signal.
        """
        return self.desired_value - self.current_value  # type: ignore


class VehicleOrientation(EqualityTask):
    """Control the vehicle orientation."""

    name = "vehicle_orientation_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new vehicle orientation task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.current_rot = Quaternion()
        self.desired_rot = Quaternion()
        self.rot_base_to_map = Quaternion()
        self.n_manipulator_joints = 0

    @staticmethod
    def create_task_from_params(
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
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the vehicle orientation.

        Returns:
            The reference signal needed to move the vehicle to the desired orientation.
        """
        return calculate_quaternion_error(self.desired_rot, self.current_rot)[
            :3
        ].reshape((3, 1))


class EndEffectorPose(EqualityTask):
    """Control the end-effector pose."""

    name = "end_effector_pose_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new end effector pose task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.serial_chain: Any | None = None
        self.joint_angles = np.zeros((1, 1))
        self.tf_map_to_base = Transform()
        self.tf_base_to_manipulator_base = Transform()
        self.tf_manipulator_base_to_ee = Transform()
        self.current_value = Transform()
        self.desired_value = Transform()

    @staticmethod
    def create_task_from_params(
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
            task.desired_value.translation.x = x
            task.desired_value.translation.y = y
            task.desired_value.translation.z = z

            (
                task.desired_value.rotation.x,
                task.desired_value.rotation.y,
                task.desired_value.rotation.z,
                task.desired_value.rotation.w,
            ) = R.from_euler(
                "xyz", [roll, pitch, yaw]  # type: ignore
            ).as_quat(
                False
            )

        return task

    def update(
        self,
        joint_angles: np.ndarray,
        tf_map_to_ee: Transform,
        tf_map_to_base: Transform,
        tf_base_to_manipulator_base: Transform,
        tf_manipulator_base_to_ee: Transform,
        serial_chain: Any | None = None,
        desired_pose: Transform | None = None,
    ) -> None:
        """Update the current context of the end effector pose task.

        Args:
            joint_angles: The current manipulator joint angles.
            current_pose: The current vehicle pose in the inertial (map) frame.
            tf_base_to_manipulator_base: The transformation from the vehile base
                frame to the manipulator base frame.
            tf_manipulator_base_to_ee: The transformation from the manipulator base
                frame to the end effector frame.
            serial_chain: The manipulator kinpy serial chain. Defaults to None.
            desired_pose: The desired end effector pose. Defaults to None.
        """
        self.joint_angles = joint_angles

        self.current_value = tf_map_to_ee

        self.tf_map_to_base = tf_map_to_base
        self.tf_base_to_manipulator_base = tf_base_to_manipulator_base
        self.tf_manipulator_base_to_ee = tf_manipulator_base_to_ee

        if serial_chain is not None:
            self.serial_chain = serial_chain

        if desired_pose is not None:
            self.desired_value = desired_pose

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the UVMS Jacobian.

        Returns:
            The UVMS Jacobian.
        """
        # Get the transformation translations
        # denoted as r_{from frame}{to frame}_{with respect to x frame}
        eta1 = point_to_array(self.tf_map_to_base.translation)
        r_B0_B = point_to_array(self.tf_base_to_manipulator_base.translation)
        eta_0ee_0 = point_to_array(self.tf_manipulator_base_to_ee.translation)

        # Get the transformation rotations
        # denoted as R_{from frame}_{to frame}
        R_0_B = np.linalg.inv(
            quaternion_to_rotation(
                self.tf_base_to_manipulator_base.rotation
            ).as_matrix()
        )
        R_B_I = np.linalg.inv(
            quaternion_to_rotation(self.tf_map_to_base.rotation).as_matrix()
        )
        R_0_I = R_B_I @ R_0_B

        r_B0_I = R_B_I @ r_B0_B
        eta_0ee_I = R_0_I @ eta_0ee_0

        def get_skew_matrix(x: np.ndarray) -> np.ndarray:
            # Expect a 3x1 vector
            return np.array(
                [
                    [0, -x[2][0], x[1][0]],  # type: ignore
                    [x[2][0], 0, -x[0][0]],
                    [-x[1][0], x[0][0], 0],
                ],
            )

        J = np.zeros((6, 6 + len(self.joint_angles)))
        J_man = calculate_manipulator_jacobian(self.serial_chain, self.joint_angles)

        # Position Jacobian
        J[:3, :3] = R_B_I
        J[:3, 3:6] = -(get_skew_matrix(r_B0_I) + get_skew_matrix(eta_0ee_I)) @ R_B_I  # type: ignore # noqa
        J[:3, 6:] = R_0_I @ J_man[:3]

        # Orientation Jacobian
        J[3:6, 3:6] = R_B_I
        J[3:6, 6:] = R_0_I @ J_man[3:]

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the controller.

        Returns:
            The reference signal to use to drive the system to the desired end effector
                pose.
        """
        pos_error = np.array(
            [
                self.desired_value.translation.x - self.current_value.translation.x,
                self.desired_value.translation.y - self.current_value.translation.y,
                self.desired_value.translation.z - self.current_value.translation.z,
            ]
        ).reshape((3, 1))
        ori_error = calculate_quaternion_error(
            self.desired_value.rotation, self.current_value.rotation
        )[:3].reshape((3, 1))

        return np.vstack((pos_error, ori_error))


class JointLimit(SetTask):
    """Limit a joint angle to a desired range."""

    name = "joint_limit_set"

    def __init__(
        self,
        physical_upper: float,
        physical_lower: float,
        safety_upper: float,
        safety_lower: float,
        activation_threshold: float,
        gain: float,
        priority: float,
        joint: int,
    ) -> None:
        """Create a new set constraint.

        Args:
            physical_upper: The maximum joint angle.
            physical_lower: The minimum joint angle.
            safety_upper: The upper safety limit used to create a buffer from the
                upper physical limit.
            safety_lower: The lower safety limit used to create a buffer from the lower
                physical limit.
            activation_threshold: The distance from safety thresholds at which the task
                should become activated.
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
            joint: The joint that the constraint applies to.
        """
        super().__init__(
            physical_upper,
            physical_lower,
            safety_upper,
            safety_lower,
            activation_threshold,
            gain,
            priority,
        )

        self.joint = joint
        self.n_manipulator_joints = 0
        self.current_value = 0.0
        self.desired_value = 0.0

    @staticmethod
    def create_task_from_params(
        physical_upper: float,
        physical_lower: float,
        safety_upper: float,
        safety_lower: float,
        activation_threshold: float,
        gain: float,
        priority: float,
        joint: int,
    ) -> Any:
        """Create a new joint limit task.

        Args:
            physical_upper: The maximum joint angle.
            physical_lower: The minimum joint angle.
            safety_upper: The upper safety limit used to create a buffer from the
                upper physical limit.
            safety_lower: The lower safety limit used to create a buffer from the lower
                physical limit.
            activation_threshold: The distance from safety thresholds at which the task
                should become activated.
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
            joint: The joint that the constraint applies to.
        """
        return JointLimit(
            physical_upper,
            physical_lower,
            safety_upper,
            safety_lower,
            activation_threshold,
            gain,
            priority,
            joint,
        )

    def update(
        self, current_angle: float, n_manipulator_joints: int | None = None
    ) -> None:
        """Update the current context of the joint limit task.

        Args:
            current_angle: The current joint angle.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.current_value = current_angle

        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the Jacobian for a joint limit.

        Args:
            num_manipulator_joints: The total number of joints that the manipulator has.

        Returns:
            The joint limit Jacobian.
        """
        J = np.zeros((1, 6 + self.n_manipulator_joints))
        J[0, self.joint] = 1

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the joint limit task.

        Returns:
            The reference signal to use to drive the joint back to a safe range.
        """
        return np.array([self.desired_value - self.current_value])  # type: ignore


class ManipulatorConfiguration(EqualityTask):
    """Control the joint angles of the manipulator."""

    name = "manipulator_configuration_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new manipulator configuration task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.current_value = np.zeros((1, 1))
        self.desired_value = np.zeros((1, 1))
        self.n_manipulator_joints = 0

    @staticmethod
    def create_task_from_params(
        gain: float, priority: float, desired_joint_angles: list[float] | None
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
            task.desired_value = np.array(desired_joint_angles).reshape(
                (len(desired_joint_angles), 1)
            )

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
        self.current_value = current_joint_angles

        if desired_joint_angles is not None:
            self.desired_value = desired_joint_angles

        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the Jacobian for the manipulator joint configuration task.

        Returns:
            The joint configuration Jacobian.
        """
        J = np.zeros((self.n_manipulator_joints, 6 + self.n_manipulator_joints))
        J[0:, 6:] = np.eye(self.n_manipulator_joints)

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference for the joint configuration task.

        Returns:
            The joint configuration reference signal.
        """
        return self.desired_value - self.current_value  # type: ignore


class VehicleOrientationLimit(SetTask):
    """Constraint the vehicle roll."""

    name = "vehicle_orientation_limit_set"

    def __init__(
        self,
        physical_upper: float,
        physical_lower: float,
        safety_upper: float,
        safety_lower: float,
        activation_threshold: float,
        gain: float,
        priority: float,
        joint: int,
    ) -> None:
        """Create a new vehicle orientation limit task.

        Args:
            physical_upper: The maximum rotation angle.
            physical_lower: The minimum rotation angle.
            safety_upper: The upper safety limit used to create a buffer from the
                upper physical limit.
            safety_lower: The lower safety limit used to create a buffer from the lower
                physical limit.
            activation_threshold: The distance from safety thresholds at which the task
                should become activated.
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
            joint: The vehicle orientation joint to constrain: 3 for roll, 4 for pitch,
                5 for yaw.
        """
        super().__init__(
            physical_upper,
            physical_lower,
            safety_upper,
            safety_lower,
            activation_threshold,
            gain,
            priority,
        )
        self.joint = joint

    @staticmethod
    def create_task_from_params(
        physical_upper: float,
        physical_lower: float,
        safety_upper: float,
        safety_lower: float,
        activation_threshold: float,
        gain: float,
        priority: float,
        joint: int,
    ) -> Any:
        """Create a new vehicle roll limit task.

        Args:
            physical_upper: The maximum rotation angle.
            physical_lower: The minimum rotation angle.
            safety_upper: The upper safety limit used to create a buffer from the
                upper physical limit.
            safety_lower: The lower safety limit used to create a buffer from the lower
                physical limit.
            activation_threshold: The distance from safety thresholds at which the task
                should become activated.
            gain: The constraint gain to use for closed-loop control.
            priority: The constraint priority.
            joint: The vehicle orientation joint to constrain: 3 for roll, 4 for pitch,
                5 for yaw.
        """
        return VehicleOrientationLimit(
            physical_upper,
            physical_lower,
            safety_upper,
            safety_lower,
            activation_threshold,
            gain,
            priority,
            joint,
        )

    def update(
        self,
        current_rot: Quaternion,
        n_manipulator_joints: int | None = None,
    ) -> None:
        """Update the task context.

        Args:
            current_rot: The current vehicle orientation in the inertial (map)
                frame.
            n_manipulator_joints: The total number of joints that the manipulator has.
                Defaults to None.
        """
        self.rot_map_to_base = current_rot

        cr = R.from_quat(
            [
                current_rot.x,
                current_rot.y,
                current_rot.z,
                current_rot.w,
            ]
        )
        self.current_value = np.array([cr.as_euler("xyz")[self.joint - 2]])

        if n_manipulator_joints is not None:
            self.n_manipulator_joints = n_manipulator_joints

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the vehicle roll limit task Jacobian.

        Returns:
            The Jacobian for a task which controls the vehicle roll and pitch.
        """
        J_ko = calculate_vehicle_angular_velocity_jacobian(self.rot_map_to_base)

        J = np.zeros((1, 6 + self.n_manipulator_joints))
        axis = np.zeros((1, 3))
        axis[:, self.joint - 2] = 1
        J[:, 3:6] = axis @ np.linalg.pinv(J_ko)

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the vehicle roll limit task.

        Returns:
            The reference signal.
        """
        return self.desired_value - self.current_value  # type: ignore

    def set_task_active(self, value: Quaternion) -> bool:
        """Enable/disable the task according to its current state.

        Args:
            value: The current orientation of the vehicle with respect to the inertial
                frame.

        Returns:
            Whether or not the task was activated.
        """
        cr = R.from_quat(
            [
                value.x,
                value.y,
                value.z,
                value.w,
            ]
        )
        angle = cr.as_euler("xyz")[self.joint - 2]

        return super().set_task_active(angle)


class EndEffectorPosition(EqualityTask):
    """Control the end-effector pose."""

    name = "end_effector_position_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new end effector pose task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.serial_chain: Any | None = None
        self.joint_angles = np.zeros((1, 1))
        self.tf_map_to_base = Transform()
        self.tf_base_to_manipulator_base = Transform()
        self.tf_manipulator_base_to_ee = Transform()
        self.current_value = Transform()
        self.desired_value = Transform()

    @staticmethod
    def create_task_from_params(
        gain: float,
        priority: float,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
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

        Returns:
            A new end effector pose task.
        """
        task = EndEffectorPosition(gain, priority)

        if None not in [x, y, z]:
            task.desired_value.translation.x = x
            task.desired_value.translation.y = y
            task.desired_value.translation.z = z

        return task

    def update(
        self,
        joint_angles: np.ndarray,
        tf_map_to_ee: Transform,
        tf_map_to_base: Transform,
        tf_base_to_manipulator_base: Transform,
        tf_manipulator_base_to_ee: Transform,
        serial_chain: Any | None = None,
        desired_pose: Transform | None = None,
    ) -> None:
        """Update the current context of the end effector pose task.

        Args:
            joint_angles: The current manipulator joint angles.
            current_pose: The current vehicle pose in the inertial (map) frame.
            tf_base_to_manipulator_base: The transformation from the vehile base
                frame to the manipulator base frame.
            tf_manipulator_base_to_ee: The transformation from the manipulator base
                frame to the end effector frame.
            serial_chain: The manipulator kinpy serial chain. Defaults to None.
            desired_pose: The desired end effector pose. Defaults to None.
        """
        self.joint_angles = joint_angles

        self.current_value = tf_map_to_ee

        self.tf_map_to_base = tf_map_to_base
        self.tf_base_to_manipulator_base = tf_base_to_manipulator_base
        self.tf_manipulator_base_to_ee = tf_manipulator_base_to_ee

        if serial_chain is not None:
            self.serial_chain = serial_chain

        if desired_pose is not None:
            self.desired_value = desired_pose

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the UVMS Jacobian.

        Returns:
            The UVMS Jacobian.
        """
        eta1 = point_to_array(self.tf_map_to_base.translation)

        r_B0_B = point_to_array(self.tf_base_to_manipulator_base.translation)
        R_0_B = quaternion_to_rotation(
            self.tf_base_to_manipulator_base.rotation
        ).as_matrix()
        J = np.zeros((3, 6 + len(self.serial_chain.get_joint_parameter_names())))

        J_man = calculate_manipulator_jacobian(self.serial_chain, self.joint_angles)
        R_B_I = np.linalg.inv(
            quaternion_to_rotation(self.tf_map_to_base.rotation).as_matrix()
        )
        R_0_I = R_B_I @ R_0_B

        eta_ee1 = point_to_array(self.current_value.translation)
        r_B0_I = R_B_I @ r_B0_B
        eta_0ee_I = eta_ee1 - eta1 - r_B0_I

        def get_skew_matrix(x: np.ndarray) -> np.ndarray:
            # Expect a 3x1 vector
            x = x.reshape(3)
            return np.array(
                [
                    [0, -x[2], x[1]],  # type: ignore
                    [x[2], 0, -x[0]],
                    [-x[1], x[0], 0],
                ],
            )

        J[:, 0:3] = R_B_I
        J[:, 3:6] = -(get_skew_matrix(r_B0_I) + get_skew_matrix(eta_0ee_I)) @ R_B_I
        J[:, 6:] = R_0_I @ J_man[:3]

        self.jman = R_B_I @ R_B_I.T

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the controller.

        Returns:
            The reference signal to use to drive the system to the desired end effector
                pose.
        """
        pos_error = np.array(
            [
                self.desired_value.translation.x - self.current_value.translation.x,
                self.desired_value.translation.y - self.current_value.translation.y,
                self.desired_value.translation.z - self.current_value.translation.z,
            ]
        ).reshape((3, 1))

        return pos_error


class EndEffectorOrientation(EqualityTask):
    """Control the end-effector pose."""

    name = "end_effector_orientation_eq"

    def __init__(self, gain: float, priority: float) -> None:
        """Create a new end effector pose task.

        Args:
            gain: The task gain.
            priority: The task priority.
        """
        super().__init__(gain, priority)

        self.serial_chain: Any | None = None
        self.joint_angles = np.zeros((1, 1))
        self.tf_map_to_base = Transform()
        self.tf_base_to_manipulator_base = Transform()
        self.tf_manipulator_base_to_ee = Transform()
        self.current_value = Transform()
        self.desired_value = Transform()

    @staticmethod
    def create_task_from_params(
        gain: float,
        priority: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
    ) -> Any:
        """Create a new end effector pose task from a set of parameters.

        Args:
            gain: The task gain.
            priority: The task priority.
            roll: The desired end effector roll in the inertial (map) frame. Defaults
                to None.
            pitch: The desired end effector pitch in the inertial (map) frame. Defaults
                to None.
            yaw: The desired end effector yaw in the inertial (map) frame. Defaults
                to None.

        Returns:
            A new end effector pose task.
        """
        task = EndEffectorOrientation(gain, priority)

        if None not in [roll, pitch, yaw]:
            (
                task.desired_value.rotation.x,
                task.desired_value.rotation.y,
                task.desired_value.rotation.z,
                task.desired_value.rotation.w,
            ) = R.from_euler(
                "xyz", [roll, pitch, yaw]  # type: ignore
            ).as_quat(
                False
            )

        return task

    def update(
        self,
        joint_angles: np.ndarray,
        tf_map_to_ee: Transform,
        tf_map_to_base: Transform,
        tf_base_to_manipulator_base: Transform,
        tf_manipulator_base_to_ee: Transform,
        serial_chain: Any | None = None,
        desired_pose: Transform | None = None,
    ) -> None:
        """Update the current context of the end effector pose task.

        Args:
            joint_angles: The current manipulator joint angles.
            current_pose: The current vehicle pose in the inertial (map) frame.
            tf_base_to_manipulator_base: The transformation from the vehile base
                frame to the manipulator base frame.
            tf_manipulator_base_to_ee: The transformation from the manipulator base
                frame to the end effector frame.
            serial_chain: The manipulator kinpy serial chain. Defaults to None.
            desired_pose: The desired end effector pose. Defaults to None.
        """
        self.joint_angles = joint_angles

        self.current_value = tf_map_to_ee

        self.tf_map_to_base = tf_map_to_base
        self.tf_base_to_manipulator_base = tf_base_to_manipulator_base
        self.tf_manipulator_base_to_ee = tf_manipulator_base_to_ee

        if serial_chain is not None:
            self.serial_chain = serial_chain

        if desired_pose is not None:
            self.desired_value = desired_pose

    @property
    def jacobian(self) -> np.ndarray:
        """Calculate the UVMS Jacobian.

        Returns:
            The UVMS Jacobian.
        """
        # Get the transformation rotations
        # denoted as R_{from frame}_{to frame}
        R_0_B = np.linalg.inv(
            quaternion_to_rotation(
                self.tf_base_to_manipulator_base.rotation
            ).as_matrix()
        )
        R_B_I = np.linalg.inv(
            quaternion_to_rotation(self.tf_map_to_base.rotation).as_matrix()
        )
        R_0_I = R_B_I @ R_0_B

        J = np.zeros((3, 6 + len(self.joint_angles)))
        J_man = calculate_manipulator_jacobian(self.serial_chain, self.joint_angles)

        # Orientation Jacobian
        J[:, 3:6] = R_B_I
        J[:, 6:] = R_0_I @ J_man[3:]

        return J

    @property
    def error(self) -> np.ndarray:
        """Calculate the reference signal for the controller.

        Returns:
            The reference signal to use to drive the system to the desired end effector
                pose.
        """
        ori_error = calculate_quaternion_error(
            self.desired_value.rotation, self.current_value.rotation
        )[:3].reshape((3, 1))

        return ori_error
