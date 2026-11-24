"""ROS 2 ENU/FLU telemetry and planar range bridge for localhost ArduCopter SITL."""

import math
import select
import socket
import time

from nav_msgs.msg import Odometry
from pymavlink.dialects.v20 import ardupilotmega as mav
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .mission import DemoMission


def scan_bins(scan):
    """Map ROS CCW rays to 72 forward-first clockwise sectors in centimetres."""
    if not (0 <= scan.range_min < scan.range_max < 655.34):
        raise ValueError('Invalid MAVLink sensor limits')
    minimum, maximum = round(100 * scan.range_min), round(100 * scan.range_max)
    distances = [65535] * 72  # MAVLink unknown/unobserved sector
    for index, value in enumerate(scan.ranges):
        if value == math.inf:
            distance = maximum + 1  # sector observed clear out to range_max
        elif scan.range_min <= value <= scan.range_max:
            distance = round(value * 100)
        else:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        sector = round(-math.degrees(angle) / 5) % 72
        distances[sector] = min(distances[sector], distance)
    return minimum, maximum, distances


def orientation_and_velocity(attitude, position):
    """Convert an NED/FRD attitude to ENU/FLU, and NED velocity to body FLU."""
    roll, pitch, yaw = attitude.roll, -attitude.pitch, math.pi / 2 - attitude.yaw
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    x, y = sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy
    z, w = cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy
    # Rotate ENU velocity into the child frame using the inverse quaternion.
    ve, vn, vu = position.vy, position.vx, -position.vz
    a, b, c = 2 * (z * vn - y * vu), 2 * (x * vu - z * ve), 2 * (y * ve - x * vn)
    velocity = (ve + w * a - y * c + z * b,
                vn + w * b - z * a + x * c,
                vu + w * c - x * b + y * a)
    return (x, y, z, w), velocity


class SitlBridge(Node):
    def __init__(self):
        super().__init__('sitl_bridge')
        enabled = self.declare_parameter('run_mission', False).value
        self.odom_pub = self.create_publisher(Odometry, '/uav/odom', 10)
        self.status_pub = self.create_publisher(String, '/uav/mission_status', 10)
        self.create_subscription(LaserScan, '/uav/scan', self.on_scan, qos_profile_sensor_data)
        self.sock = None
        self.system_id = self.component_id = 0
        self.received = {}
        self.position = self.global_position = self.attitude = None
        self.has_home = self.has_fix = self.armed = False
        self.mode = None
        self.next_connect = self.next_heartbeat = self.next_setup = 0.0
        self.mission = DemoMission(self, enabled, time.monotonic())
        self.create_timer(0.02, self.poll)
        self.get_logger().info('Waiting for ArduCopter SITL at tcp:127.0.0.1:5760')

    def report(self, log=True):
        phase = self.mission.phase
        if log:
            message = f'Mission {phase}'
            if math.isfinite(self.mission.clearance):
                message += f' | min clearance {self.mission.clearance:.2f} m, lateral detour {self.mission.detour:.2f} m'
            self.get_logger().info(message)
        self.status_pub.publish(String(data=phase))

    def close_link(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        self.system_id = 0
        self.received.clear()
        self.has_home = self.has_fix = False
        self.attitude = self.position = self.global_position = None

    def write(self, data):
        # pymavlink writes complete packets to this file-like interface.
        self.sock.sendall(data)

    def fresh(self, name, now, limit):
        return now - self.received.get(name, -math.inf) <= limit

    def link_alive(self, now):
        return self.fresh('HEARTBEAT', now, 5.0)

    def flight_data_fresh(self, now):
        return (self.link_alive(now) and self.fresh('LOCAL_POSITION_NED', now, 1.0)
                and self.fresh('GLOBAL_POSITION_INT', now, 2.0) and self.fresh('scan', now, 0.8))

    def ready(self, now):
        return self.flight_data_fresh(now) and self.has_home and self.has_fix and self.fresh('SIMSTATE', now, 4.0)

    def command(self, cmd, *params):
        values = list(params) + [0] * (7 - len(params))
        self.tx.command_long_send(self.system_id, self.component_id, cmd, 0, *values)

    def set_mode(self, mode):
        self.tx.set_mode_send(self.system_id, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)

    def setup_streams(self):
        for msg_id, hz in ((mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 15), (mav.MAVLINK_MSG_ID_ATTITUDE, 15),
                           (mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5), (mav.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
                           (mav.MAVLINK_MSG_ID_SIMSTATE, 2)):
            self.command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, round(1e6 / hz))
        if not self.has_home:
            self.command(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_HOME_POSITION)

    def on_scan(self, scan):
        if self.sock is None or not self.system_id or scan.header.frame_id != 'base_link':
            return
        stamp = scan.header.stamp
        stamp_us = stamp.sec * 1000000 + stamp.nanosec // 1000
        age = self.get_clock().now().nanoseconds / 1e9 - stamp_us / 1e6
        if not -0.05 <= age <= 0.5:
            return
        try:
            minimum, maximum, distances = scan_bins(scan)
            if min(distances) == 65535:
                return
            self.tx.obstacle_distance_send(stamp_us, mav.MAV_DISTANCE_SENSOR_LASER, distances,
                                           5, minimum, maximum, 5.0, 0.0, mav.MAV_FRAME_BODY_FRD)
            self.received['scan'] = time.monotonic()
        except ValueError:
            return
        except OSError as exc:
            self.link_lost(exc)

    def publish_odom(self):
        p, a = self.position, self.attitude
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        pos, rot = msg.pose.pose.position, msg.pose.pose.orientation
        pos.x, pos.y, pos.z = p.y, p.x, -p.z
        orientation, velocity = orientation_and_velocity(a, p)
        rot.x, rot.y, rot.z, rot.w = orientation
        linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
        linear.x, linear.y, linear.z = velocity
        angular.x, angular.y, angular.z = a.rollspeed, -a.pitchspeed, -a.yawspeed
        self.odom_pub.publish(msg)

    def on_mavlink(self, message, now):
        kind = message.get_type()
        if kind == 'HEARTBEAT' and message.autopilot == mav.MAV_AUTOPILOT_ARDUPILOTMEGA and message.type == mav.MAV_TYPE_QUADROTOR:
            self.system_id, self.component_id = message.get_srcSystem(), message.get_srcComponent()
            self.mode = message.custom_mode
            self.armed = bool(message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
        if not self.system_id or (message.get_srcSystem(), message.get_srcComponent()) != (self.system_id, self.component_id):
            return
        self.received[kind] = now
        if kind == 'LOCAL_POSITION_NED':
            self.position = message
            if self.fresh('ATTITUDE', now, 0.5):
                self.publish_odom()
        elif kind == 'ATTITUDE':
            self.attitude = message
        elif kind == 'GLOBAL_POSITION_INT':
            self.global_position = message
        elif kind == 'HOME_POSITION':
            self.has_home = True
        elif kind == 'EKF_STATUS_REPORT':
            self.has_fix = bool(message.flags & mav.EKF_POS_HORIZ_ABS)
        elif kind == 'STATUSTEXT':
            self.get_logger().info(str(message.text))

    def link_lost(self, error):
        self.get_logger().warning(f'SITL connection lost: {error}')
        self.close_link()
        if self.mission.phase in ('arming', 'takeoff', 'navigating', 'landing'):
            self.mission.fail('SITL disconnected during mission; landing unconfirmed', time.monotonic())

    def poll(self):
        now = time.monotonic()
        try:
            if self.sock is None and now >= self.next_connect:
                self.next_connect = now + 1
                try:
                    self.sock = socket.create_connection(('127.0.0.1', 5760), timeout=0.3)
                except OSError:
                    pass
                else:
                    self.tx = mav.MAVLink(self, srcSystem=255, srcComponent=mav.MAV_COMP_ID_MISSIONPLANNER)
                    self.tx.robust_parsing = True
                    self.next_setup = 0
                    self.connected_at = now
            if self.sock is not None:
                if select.select([self.sock], [], [], 0)[0]:
                    data = self.sock.recv(65536)
                    if not data:
                        raise ConnectionError('socket closed')
                    for msg in self.tx.parse_buffer(data) or []:
                        self.on_mavlink(msg, now)
                if now - self.received.get('HEARTBEAT', self.connected_at) > 6:
                    raise ConnectionError('heartbeat timed out')
                if self.system_id and now >= self.next_setup:
                    self.setup_streams()
                    self.next_setup = now + (15 if self.has_home else 2)
            if now >= self.next_heartbeat:
                if self.sock is not None:
                    self.tx.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE)
                self.report(log=False)
                self.next_heartbeat = now + 1
            self.mission.tick(now)
        except OSError as exc:
            self.link_lost(exc)


def main(args=None):
    rclpy.init(args=args)
    node = SitlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_link()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
