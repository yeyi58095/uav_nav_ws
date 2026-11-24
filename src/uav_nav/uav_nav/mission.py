"""Opt-in flight and acceptance checks for the fixed SITL cylinder scenario."""

import math

from pymavlink.dialects.v20 import ardupilotmega as mav


class DemoMission:
    ALTITUDE = 5.0
    TARGET_EAST = 20.0
    GUIDED = 4
    LAND = 9

    def __init__(self, bridge, enabled, now):
        self.bridge = bridge
        self.phase = 'waiting' if enabled else 'observing'
        self.started = now
        self.last_command = -math.inf
        self.clearance = math.inf
        self.detour = 0.0
        self.error = ''

    def transition(self, phase, now):
        self.phase = phase
        self.started = now
        self.last_command = -math.inf
        self.bridge.report()

    def retry(self, now, callback):
        if now - self.last_command >= 2.0:
            callback()
            self.last_command = now

    def fail(self, reason, now):
        self.error = reason
        self.bridge.get_logger().error(reason)
        if self.bridge.armed and self.bridge.link_alive(now):
            self.transition('landing', now)
        else:
            self.transition('failed', now)

    def tick(self, now):
        b = self.bridge
        if self.phase in ('observing', 'succeeded', 'failed'):
            return
        elapsed = now - self.started
        if self.phase == 'landing':
            if b.link_alive(now) and not b.armed:
                self.transition('failed' if self.error else 'succeeded', now)
            elif elapsed > 75.0:
                self.error = 'Landing or disarm could not be confirmed'
                b.get_logger().error(self.error)
                self.transition('failed', now)
            elif b.link_alive(now) and b.mode != self.LAND:
                self.retry(now, lambda: b.set_mode(self.LAND))
            return

        if elapsed > {'waiting': 120, 'arming': 90, 'takeoff': 45, 'navigating': 150}[self.phase]:
            self.fail(f'Timed out in {self.phase}', now)
            return

        if self.phase == 'waiting':
            if not b.ready(now):
                return
            if b.armed or math.hypot(b.position.x, b.position.y) > 2.0:
                self.error = 'Demo requires a fresh disarmed SITL near the map origin'
                b.get_logger().error(self.error)
                self.transition('failed', now)
                return
            self.transition('arming', now)

        elif self.phase == 'arming':
            if not b.ready(now):
                return
            if b.mode != self.GUIDED:
                self.retry(now, lambda: b.set_mode(self.GUIDED))
            elif not b.armed:
                self.retry(now, lambda: b.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1))
            else:
                self.transition('takeoff', now)

        elif not b.armed or b.mode != self.GUIDED or not b.flight_data_fresh(now):
            self.fail('Lost GUIDED, arming, position telemetry or lidar during flight', now)

        elif self.phase == 'takeoff':
            alt = b.global_position.relative_alt * 0.001
            if alt >= self.ALTITUDE - 0.4:
                # Send the position once so the waypoint planner can finish its route.
                self.send_destination()
                self.transition('navigating', now)
            elif alt < 0.4:
                self.retry(now, lambda: b.command(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, self.ALTITUDE))

        elif self.phase == 'navigating':
            east, north = b.position.y, b.position.x
            distance = math.hypot(east - 10.0, north) - 2.0
            self.clearance = min(self.clearance, distance)
            if abs(east - 10.0) < 4.0:
                self.detour = max(self.detour, abs(north))
            if distance <= 0.5:
                self.fail(f'Cylinder clearance fell to {distance:.2f} m', now)
            elif (math.hypot(east - self.TARGET_EAST, north) <= 1.0
                  and abs(b.global_position.relative_alt * 0.001 - self.ALTITUDE) <= 0.8):
                if self.detour < 2.5:
                    self.fail('Goal reached without an observed lateral detour', now)
                else:
                    self.transition('landing', now)

    def send_destination(self):
        b = self.bridge
        # Match the ENU map goal to WGS84 using the current fused fixes. Altitude
        # in this MAVLink frame is always metres above the ArduCopter home point.
        lat = b.global_position.lat * 1e-7
        metres_per_degree = 6378137.0 * math.pi / 180.0
        target_lat = b.global_position.lat - round(b.position.x / metres_per_degree * 1e7)
        delta_east = self.TARGET_EAST - b.position.y
        target_lon = b.global_position.lon + round(delta_east / (metres_per_degree * math.cos(math.radians(lat))) * 1e7)
        b.tx.set_position_target_global_int_send(
            0, b.system_id, b.component_id, mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000, target_lat, target_lon, self.ALTITUDE, 0, 0, 0, 0, 0, 0, 0, 0,
        )
