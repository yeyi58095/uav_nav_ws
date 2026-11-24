"""Synthetic horizontal lidar for a circular column in the local ENU frame."""

import math


BEAM_COUNT = 72
ANGLE_INCREMENT = 2.0 * math.pi / BEAM_COUNT
RANGE_MIN = 0.2
RANGE_MAX = 20.0


def quaternion_yaw(x, y, z, w):
    """Return ENU heading in radians for a body-to-world orientation."""
    norm_squared = x * x + y * y + z * z + w * w
    if not math.isfinite(norm_squared) or norm_squared < 1e-12:
        raise ValueError('Odometry must contain a finite, nonzero quaternion')
    return math.atan2(2.0 * (w * z + x * y), norm_squared - 2.0 * (y * y + z * z))


def circle_scan(east, north, yaw, obstacle_east=10.0, obstacle_north=0.0,
                obstacle_radius=2.0):
    """Return 72 body-frame ranges. A beam at index 0 points forward.

    The column spans every altitude. Hits inside the blind zone report the
    minimum range, and an origin inside the column reports contact on all beams.
    """
    if not all(math.isfinite(v) for v in
               (east, north, yaw, obstacle_east, obstacle_north, obstacle_radius)):
        raise ValueError('Lidar geometry must be finite')
    if obstacle_radius <= 0.0:
        raise ValueError('Obstacle radius must be positive')

    delta_east = obstacle_east - east
    delta_north = obstacle_north - north
    center_distance = math.hypot(delta_east, delta_north)
    if center_distance <= obstacle_radius:
        return [RANGE_MIN] * BEAM_COUNT
    ranges = [math.inf] * BEAM_COUNT
    if center_distance - obstacle_radius > RANGE_MAX:
        return ranges

    for i in range(BEAM_COUNT):
        angle = yaw + i * ANGLE_INCREMENT
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        along_ray = delta_east * cos_angle + delta_north * sin_angle
        cross_ray = delta_north * cos_angle - delta_east * sin_angle
        if along_ray <= 0.0 or abs(cross_ray) > obstacle_radius:
            continue
        half_chord = math.sqrt(max(0.0, obstacle_radius ** 2 - cross_ray ** 2))
        hit = along_ray - half_chord
        if hit <= RANGE_MAX:
            ranges[i] = max(RANGE_MIN, hit)
    return ranges


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init(args=args)
    node = rclpy.create_node('sim_lidar')
    try:
        fixed = ParameterDescriptor(read_only=True)
        east = node.declare_parameter('obstacle_east', 10.0, fixed).value
        north = node.declare_parameter('obstacle_north', 0.0, fixed).value
        radius = node.declare_parameter('obstacle_radius', 2.0, fixed).value
        if not all(map(math.isfinite, (east, north, radius))) or radius <= 0.0:
            raise ValueError('Obstacle coordinates must be finite and radius positive')
        publisher = node.create_publisher(LaserScan, '/uav/scan', 5)

        def on_odometry(odometry):
            pose = odometry.pose.pose
            q = pose.orientation
            try:
                yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
                ranges = circle_scan(pose.position.x, pose.position.y, yaw,
                                     east, north, radius)
            except ValueError:
                return
            scan = LaserScan()
            scan.header.stamp = odometry.header.stamp
            scan.header.frame_id = 'base_link'
            scan.angle_min = 0.0
            scan.angle_max = (BEAM_COUNT - 1) * ANGLE_INCREMENT
            scan.angle_increment = ANGLE_INCREMENT
            scan.range_min = RANGE_MIN
            scan.range_max = RANGE_MAX
            scan.ranges = ranges
            publisher.publish(scan)

        node.create_subscription(Odometry, '/uav/odom', on_odometry, qos_profile_sensor_data)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
