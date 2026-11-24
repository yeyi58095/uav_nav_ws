import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import rclpy

from uav_nav.bridge import SitlBridge, orientation_and_velocity, scan_bins
from uav_nav.mission import DemoMission


class BridgeTest(unittest.TestCase):
    def test_foxy_node_starts_in_observer_mode(self):
        rclpy.init()
        try:
            node = SitlBridge()
            try:
                self.assertEqual(node.mission.phase, 'observing')
                node.report(log=False)
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_left_obstacle_is_counterclockwise_in_ros(self):
        scan = SimpleNamespace(range_min=0.2, range_max=20.0, angle_min=0.0,
                               angle_increment=math.radians(5), ranges=[math.inf] * 72)
        scan.ranges[18] = 4.12  # 90 degrees left in ROS
        scan.ranges[36] = math.nan
        low, high, sectors = scan_bins(scan)
        self.assertEqual((low, high), (20, 2000))
        self.assertEqual(sectors[54], 412)
        self.assertEqual(sectors[36], 65535)
        self.assertEqual(sectors[18], 2001)

    def test_north_facing_vehicle_moving_east_moves_to_body_right(self):
        attitude = SimpleNamespace(roll=0., pitch=0., yaw=0.)
        position = SimpleNamespace(vx=0., vy=2., vz=-1.)
        (x, y, z, w), (forward, left, up) = orientation_and_velocity(attitude, position)
        for value, expected in zip((x, y, z, w, forward, left, up),
                                   (0, 0, math.sqrt(0.5), math.sqrt(0.5), 0, -2, 1)):
            self.assertAlmostEqual(value, expected)

    def test_success_waits_for_disarm_after_detour(self):
        bridge = Mock(armed=True, mode=DemoMission.GUIDED)
        bridge.global_position.relative_alt = 5000
        mission = DemoMission(bridge, enabled=True, now=0.0)
        mission.transition('navigating', 1.0)
        bridge.position.x, bridge.position.y = -4.0, 10.0
        mission.tick(2.0)
        self.assertEqual(mission.clearance, 2.0)
        bridge.position.x, bridge.position.y = 0.0, 19.5
        mission.tick(3.0)
        self.assertEqual(mission.phase, 'landing')
        mission.tick(4.0)
        bridge.set_mode.assert_called_once_with(DemoMission.LAND)
        bridge.armed = False
        mission.tick(5.0)
        self.assertEqual(mission.phase, 'succeeded')

    def test_direct_collision_fails_even_after_landing(self):
        bridge = Mock(armed=True, mode=DemoMission.GUIDED)
        mission = DemoMission(bridge, enabled=True, now=0.0)
        mission.transition('navigating', 1.0)
        bridge.position.x, bridge.position.y = 0.0, 8.0
        mission.tick(2.0)
        self.assertEqual(mission.phase, 'landing')
        bridge.armed = False
        mission.tick(3.0)
        self.assertEqual(mission.phase, 'failed')
