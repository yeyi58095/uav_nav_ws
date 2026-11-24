import math
import unittest

from uav_nav.lidar import BEAM_COUNT, RANGE_MAX, RANGE_MIN, circle_scan, quaternion_yaw


class LidarTest(unittest.TestCase):
    def test_origin_facing_east_hits_eight_metres_ahead(self):
        ranges = circle_scan(0.0, 0.0, 0.0)
        self.assertEqual(len(ranges), 72)
        self.assertEqual(ranges[0], 8.0)
        self.assertAlmostEqual(ranges[1], ranges[-1])
        self.assertEqual(ranges[BEAM_COUNT // 2], math.inf)

    def test_north_facing_sensor_sees_eastern_obstacle_to_the_right(self):
        yaw = quaternion_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        ranges = circle_scan(2.0, 0.0, yaw)
        self.assertEqual(ranges[0], math.inf)
        self.assertAlmostEqual(ranges[54], 6.0)

    def test_tangent_and_range_limit(self):
        self.assertAlmostEqual(circle_scan(0, 2, 0)[0], 10.0)
        self.assertEqual(circle_scan(-12, 0, 0)[0], RANGE_MAX)
        self.assertTrue(all(r == math.inf for r in circle_scan(-13, 0, 0)))

    def test_contact_and_blind_zone(self):
        self.assertEqual(circle_scan(10, 0, 0), [RANGE_MIN] * BEAM_COUNT)
        self.assertEqual(circle_scan(7.9, 0, 0)[0], RANGE_MIN)

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):
            quaternion_yaw(0, 0, 0, 0)
        with self.assertRaises(ValueError):
            circle_scan(0, 0, math.nan)
        with self.assertRaises(ValueError):
            circle_scan(0, 0, 0, obstacle_radius=0.0)
