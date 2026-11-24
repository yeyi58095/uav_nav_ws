#!/usr/bin/env python3
"""Record original Gazebo camera images from arming through confirmed landing."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Capture:
    def __init__(self, node, folder, args):
        self.folder = folder
        self.args = args
        self.ready = False
        self.recording = False
        self.stamps = []
        self.success_stamp = None
        self.complete = False
        self.image_size = None
        node.create_subscription(String, '/uav/mission_status', self.on_status, 10)
        node.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)

    def on_status(self, msg):
        if msg.data == 'failed':
            raise RuntimeError('Mission reported failure')
        if msg.data in ('arming', 'takeoff'):
            self.recording = True
        if msg.data == 'succeeded' and self.success_stamp is None:
            if not self.stamps:
                raise RuntimeError('Mission completed without recorded frames')
            self.success_stamp = self.stamps[-1]

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8') or not msg.width or not msg.height:
            raise ValueError(f'Unsupported Gazebo camera image: {msg.width}x{msg.height} {msg.encoding}')
        if msg.step < msg.width * 3 or len(msg.data) != msg.height * msg.step:
            raise ValueError('Invalid camera row stride or data length')
        if not self.ready:
            if msg.width % 2 or msg.height % 2:
                raise ValueError('H.264 yuv420p requires even camera dimensions')
            self.image_size = (msg.width, msg.height)
            self.ready = True
            if self.args.ready_file:
                self.args.ready_file.touch(exist_ok=False)
            print(f'Camera ready: {self.args.topic} ({msg.width}x{msg.height}, {msg.encoding})', flush=True)
        if not self.recording:
            return
        if (msg.width, msg.height) != self.image_size:
            raise ValueError('Camera dimensions changed during capture')
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.stamps and stamp <= self.stamps[-1]:
            if stamp == self.stamps[-1]:
                return
            raise ValueError('Gazebo camera time moved backwards')

        rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        pixels = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else pixels
        frame = self.folder / f'frame-{len(self.stamps):06d}.png'
        if not cv2.imwrite(str(frame), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
            raise OSError(f'Unable to write {frame}')
        # FFmpeg image2 can read nanosecond file times as per-frame presentation times.
        os.utime(frame, ns=(stamp, stamp))
        self.stamps.append(stamp)
        if self.success_stamp is not None and stamp - self.success_stamp >= self.args.postroll * 1e9:
            self.complete = True

    def encode(self):
        preview = self.folder / 'video.mp4'
        stamps = self.stamps
        subprocess.run([
            'ffmpeg', '-nostdin', '-v', 'error', '-f', 'image2', '-ts_from_file', '2',
            '-start_number', '0', '-i', str(self.folder / 'frame-%06d.png'),
            '-vsync', 'vfr', '-enc_time_base', '1:90000', '-c:v', 'libx264',
            '-preset', 'veryfast', '-crf', '18', '-bf', '0', '-pix_fmt', 'yuv420p',
            # FFmpeg 4.2 can otherwise discard the last VFR sample through an MP4 edit list.
            '-video_track_timescale', '90000', '-use_editlist', '0', '-movflags', '+faststart',
            '-metadata', f'comment=Gazebo camera {self.args.topic}, first image stamp {stamps[0]} ns',
            str(preview),
        ], check=True)
        decoded = subprocess.check_output([
            'ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
            'frame=best_effort_timestamp_time', '-of', 'json', str(preview),
        ], text=True)
        timecodes = [float(f['best_effort_timestamp_time']) for f in json.loads(decoded)['frames']]
        if len(timecodes) != len(stamps) or any(
            abs(t - (s - stamps[0]) / 1e9) > 0.0001 for t, s in zip(timecodes, stamps)
        ):
            raise RuntimeError('Encoded frame count or timing differs from the captured camera stream')
        preview.replace(self.args.output)
        print(f'Recorded {len(stamps)} frames spanning {timecodes[-1]:.3f} s of Gazebo time '
              f'({stamps[0]} to {stamps[-1]} ns) to {self.args.output}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='H.264 MP4 destination')
    parser.add_argument('--topic', default='/replay/camera/image_raw')
    parser.add_argument('--ready-file', type=Path, help='New file to create when the first image arrives')
    parser.add_argument('--timeout', type=float, default=300, help='Capture timeout in wall seconds')
    parser.add_argument('--postroll', type=float, default=0.75, help='Gazebo seconds to record after success')
    args = parser.parse_args()
    if not (0 < args.timeout < float('inf') and 0 <= args.postroll < float('inf')):
        parser.error('timeout must be positive and postroll must be nonnegative, both finite')
    if args.ready_file:
        if args.ready_file.exists() or args.ready_file.resolve() == args.output.resolve():
            parser.error('ready-file must be a new path distinct from the output')
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.gazebo-frames-', dir=args.output.parent) as folder:
        rclpy.init(args=[])
        node = rclpy.create_node('gazebo_camera_recorder')
        try:
            capture = Capture(node, Path(folder), args)
            deadline = time.monotonic() + args.timeout
            while not capture.complete:
                if not rclpy.ok() or time.monotonic() >= deadline:
                    raise RuntimeError('Camera capture ended before the mission and landing footage completed')
                rclpy.spin_once(node, timeout_sec=0.25)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        capture.encode()


if __name__ == '__main__':
    main()
