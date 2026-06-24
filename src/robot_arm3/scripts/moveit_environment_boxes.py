#!/usr/bin/env python3

import math
import os
import xml.etree.ElementTree as ET
from typing import NamedTuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class BoxSpec(NamedTuple):
    identifier: str
    size: tuple[float, float, float]
    pose: tuple[float, float, float, float, float, float, float]


def _six_values(values, label):
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError(f"{label} must contain six values")
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{label} contains NaN or infinity")
    return converted


def _sdf_pose(element):
    pose_element = element.find("pose")
    if pose_element is None or not pose_element.text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    values = [float(value) for value in pose_element.text.split()]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("SDF pose must contain six finite values")
    return tuple(values)


def _quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _compose_pose_chain(poses):
    position = (0.0, 0.0, 0.0)
    quaternion = (0.0, 0.0, 0.0, 1.0)
    for pose in poses:
        rotated = _rotate_vector(quaternion, pose[:3])
        position = tuple(position[index] + rotated[index] for index in range(3))
        quaternion = _quaternion_multiply(
            quaternion,
            _quaternion_from_rpy(*pose[3:]),
        )
    return position + quaternion


def _parse_sdf_box(sdf_file):
    root = ET.parse(sdf_file).getroot()
    model = root.find("model")
    if model is None:
        raise ValueError(f"SDF has no model: {sdf_file}")

    matches = []
    for link in model.findall("link"):
        for collision in link.findall("collision"):
            size_element = collision.find("geometry/box/size")
            if size_element is not None and size_element.text:
                size = tuple(float(value) for value in size_element.text.split())
                matches.append((link, collision, size))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one box collision in {sdf_file}")

    link, collision, size = matches[0]
    if len(size) != 3 or not all(math.isfinite(value) and value > 0.0 for value in size):
        raise ValueError(f"Invalid box size in {sdf_file}")
    local_poses = [_sdf_pose(model), _sdf_pose(link), _sdf_pose(collision)]
    return size, local_poses


def load_environment(config_file, sdf_directory):
    with open(config_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get("boxes"), list):
        raise ValueError("Environment config must contain a 'boxes' list")

    frame_id = str(config.get("frame_id", "")).strip()
    if not frame_id:
        raise ValueError("Environment frame_id cannot be empty")
    padding = float(config.get("planning_padding", 0.0))
    if not math.isfinite(padding) or padding < 0.0:
        raise ValueError("planning_padding must be finite and non-negative")

    boxes = []
    identifiers = set()
    for entry in config["boxes"]:
        identifier = str(entry.get("id", "")).strip()
        if not identifier or identifier in identifiers:
            raise ValueError(f"Invalid or duplicate environment box id: {identifier!r}")
        identifiers.add(identifier)
        model_pose = _six_values(entry.get("pose"), f"pose for {identifier}")
        sdf_file = os.path.join(sdf_directory, str(entry.get("sdf", "")))
        if not os.path.isfile(sdf_file):
            raise FileNotFoundError(f"Bounding Box SDF was not found: {sdf_file}")
        size, local_poses = _parse_sdf_box(sdf_file)
        pose = _compose_pose_chain([model_pose] + local_poses)
        padded_size = tuple(value + 2.0 * padding for value in size)
        boxes.append(BoxSpec(identifier, padded_size, pose))
    return frame_id, padding, boxes


def collision_object(spec, frame_id):
    collision = CollisionObject()
    collision.header.frame_id = frame_id
    collision.id = spec.identifier
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(spec.size)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = spec.pose[:3]
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = spec.pose[3:]
    collision.primitives.append(primitive)
    collision.primitive_poses.append(pose)
    collision.operation = CollisionObject.ADD
    return collision


class EnvironmentBoxesNode(Node):
    def __init__(self):
        super().__init__("moveit_environment_boxes")
        package_share = get_package_share_directory("robot_arm3")
        self.declare_parameter(
            "environment_config",
            os.path.join(package_share, "config", "environment_boxes.yaml"),
        )
        self.declare_parameter("sdf_directory", os.path.join(package_share, "urdf"))
        self.declare_parameter("apply_service", "/apply_planning_scene")

        config_file = str(self.get_parameter("environment_config").value)
        sdf_directory = str(self.get_parameter("sdf_directory").value)
        frame_id, padding, boxes = load_environment(config_file, sdf_directory)
        self._scene = PlanningScene()
        self._scene.is_diff = True
        self._scene.robot_state.is_diff = True
        self._scene.world.collision_objects = [
            collision_object(box, frame_id) for box in boxes
        ]
        self._client = self.create_client(
            ApplyPlanningScene,
            str(self.get_parameter("apply_service").value),
        )
        self._future = None
        self._timer = self.create_timer(1.0, self._try_apply)
        self.get_logger().info(
            f"Loaded {len(boxes)} environment boxes in {frame_id}; "
            f"planning padding={padding:.3f} m"
        )

    def _try_apply(self):
        if self._future is not None or not self._client.service_is_ready():
            return
        request = ApplyPlanningScene.Request()
        request.scene = self._scene
        self._future = self._client.call_async(request)
        self._future.add_done_callback(self._applied)

    def _applied(self, future):
        self._future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f"Failed to apply environment boxes: {error}")
            return
        if not response.success:
            self.get_logger().warning("MoveIt rejected the environment PlanningScene diff")
            return
        self._timer.cancel()
        self.get_logger().info("Environment boxes applied to the MoveIt PlanningScene")


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentBoxesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
