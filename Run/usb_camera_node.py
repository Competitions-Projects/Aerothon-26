#!/usr/bin/env python3
"""
Publishes a USB (UVC) webcam, e.g. Logitech Brio 100, as a ROS 2 image.

Default: 1920x1080 @ 30 fps MJPG from the camera, published as the camera's own JPEG on
/camera/image_raw/compressed (what corridor_nav.py listens to). For the raw mono8 topic
/camera/image_raw (drone_movement.py) pass -p encoding:=mono8.

Speed-ups:
  * Camera is told NOT to lower its frame rate in dim light (exposure_dynamic_framerate=0).
  * JPEG is decoded straight to greyscale (skips colour work) when encoding=mono8.
  * Pixels are handed to ROS as array('B') (avoids rclpy's slow per-byte check).

Run:
  python3 usb_camera_node.py                                    # auto-finds the webcam
  python3 usb_camera_node.py --ros-args -p exposure:=50         # manual exposure (5 ms), sharper in flight
  python3 usb_camera_node.py --ros-args -p encoding:=bgr8       # colour instead of grey
  python3 usb_camera_node.py --ros-args -p encoding:=jpeg       # FASTEST colour: camera's own JPEG,
                                                                #   published on <topic>/compressed (no decoding here)
  python3 usb_camera_node.py --ros-args -p gain:=150            # brighter without slowing the frame rate
  python3 usb_camera_node.py --ros-args -p device:=/dev/video2 -p topic:=/down_cam/image_raw
"""
import array
import glob
import os
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image


def find_webcam():
    """Pick the first USB video capture node that is NOT the Orbbec."""
    for path in sorted(glob.glob('/dev/v4l/by-id/*-video-index0')):
        if 'orbbec' not in path.lower():
            return path
    vids = sorted(glob.glob('/dev/video*'))
    return vids[0] if vids else None


def v4l2_set(dev, **controls):
    """Set camera controls with v4l2-ctl; silently skip ones this camera doesn't have."""
    for name, value in controls.items():
        subprocess.run(['v4l2-ctl', '-d', dev, '-c', f'{name}={value}'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class USBCamera(Node):
    def __init__(self):
        super().__init__('usb_camera')
        self.declare_parameter('device', '')            # empty = auto
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('encoding', 'jpeg')      # mono8, bgr8, or jpeg (compressed, fastest colour)
        self.declare_parameter('gain', -1)              # -1 = leave as is; 0..255
        self.declare_parameter('power_line_hz', 50)     # mains frequency for anti-flicker (India = 50)
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('exposure', 0)           # 0 = auto; otherwise units of 100 us (50 = 5 ms)
        self.declare_parameter('workers', 2)            # parallel decode+publish threads
        g = lambda n: self.get_parameter(n).value

        dev = g('device') or find_webcam()
        if not dev:
            raise RuntimeError('No webcam found. Check lsusb and ls /dev/video*')
        dev = os.path.realpath(dev)
        self.encoding = g('encoding')

        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # needed for 1080p30 over USB 2
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, g('width'))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, g('height'))
        self.cap.set(cv2.CAP_PROP_FPS, g('fps'))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)       # give us the raw JPEG; we decode it ourselves
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open {dev}')

        # Keep 30 fps in dim light (older kernels call it exposure_auto_priority).
        v4l2_set(dev, exposure_dynamic_framerate=0, exposure_auto_priority=0)
        exp = int(g('exposure'))
        if exp > 0:
            v4l2_set(dev, auto_exposure=1, exposure_auto=1, exposure_time_absolute=exp, exposure_absolute=exp)
        else:
            v4l2_set(dev, auto_exposure=3, exposure_auto=3)
        if int(g('gain')) >= 0:
            v4l2_set(dev, gain=int(g('gain')))
        v4l2_set(dev, power_line_frequency={50: 1, 60: 2}.get(int(g('power_line_hz')), 1))

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Opened {dev} at {w}x{h} @ {self.cap.get(cv2.CAP_PROP_FPS):.0f} fps, '
                               f'exposure={"auto" if exp <= 0 else f"{exp/10:.1f} ms"}, '
                               f'publishing {self.encoding} on {g("topic")}')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        if self.encoding == 'jpeg':
            self.pub = self.create_publisher(CompressedImage, g('topic') + '/compressed', qos)
        else:
            self.pub = self.create_publisher(Image, g('topic'), qos)
        self.running = True

        # Pipeline: one thread grabs, N workers decode + publish in parallel.
        self.cv = threading.Condition()
        self.latest = None                 # (stamp_ns, raw) newest grabbed frame not yet taken
        self.pub_lock = threading.Lock()
        self.last_pub_ns = 0
        self.stats = {'grab': 0, 'pub': 0, 'dec_ms': 0.0, 'pub_ms': 0.0}
        threading.Thread(target=self._grab_loop, daemon=True).start()
        for _ in range(max(1, int(g('workers')))):
            threading.Thread(target=self._work_loop, daemon=True).start()
        self.create_timer(5.0, self._report)

    def _decode(self, raw):
        if raw.ndim == 1 or raw.shape[0] == 1:          # raw JPEG bytes
            flag = cv2.IMREAD_GRAYSCALE if self.encoding == 'mono8' else cv2.IMREAD_COLOR
            return cv2.imdecode(np.asarray(raw).reshape(-1), flag)
        # Backend already decoded it (CONVERT_RGB ignored on some builds)
        if self.encoding == 'mono8' and raw.ndim == 3:
            return cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
        return raw

    def _grab_loop(self):
        """Only grabs frames, as fast as the camera delivers them."""
        while self.running and rclpy.ok():
            ok, raw = self.cap.read()
            if not ok or raw is None:
                self.get_logger().warn('Frame grab failed', throttle_duration_sec=2.0)
                continue
            stamp = self.get_clock().now()
            with self.cv:
                self.latest = (stamp, raw.copy())        # newest wins; old untaken frame is dropped
                self.stats['grab'] += 1
                self.cv.notify()

    def _work_loop(self):
        """Decode + publish. Several of these run in parallel."""
        while self.running and rclpy.ok():
            with self.cv:
                while self.latest is None and self.running:
                    self.cv.wait(timeout=0.5)
                if self.latest is None:
                    continue
                stamp, raw = self.latest
                self.latest = None

            t0 = time.perf_counter()
            if self.encoding == 'jpeg':
                msg = self._jpeg_msg(raw, stamp)
                if msg is None:
                    continue
                t1 = time.perf_counter()
                self._publish(msg, stamp, t0, t1)
                continue

            frame = self._decode(raw)
            if frame is None:
                continue
            t1 = time.perf_counter()

            msg = Image()
            msg.header.stamp = stamp.to_msg()            # time of capture, not of publishing
            msg.header.frame_id = 'camera'
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = self.encoding
            msg.step = msg.width * (1 if self.encoding == 'mono8' else 3)
            buf = array.array('B')                       # fast path for rclpy
            buf.frombytes(frame.tobytes())
            msg.data = buf
            self._publish(msg, stamp, t0, t1)

    def _jpeg_msg(self, raw, stamp):
        """Wrap the camera's own JPEG in a CompressedImage (no decoding)."""
        if raw.ndim == 1 or raw.shape[0] == 1:
            data = np.asarray(raw).reshape(-1).tobytes()
        else:                                             # backend already decoded it: re-encode
            ok, enc = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok:
                return None
            data = enc.tobytes()
        msg = CompressedImage()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        buf = array.array('B')
        buf.frombytes(data)
        msg.data = buf
        return msg

    def _publish(self, msg, stamp, t0, t1):
        with self.pub_lock:
            if stamp.nanoseconds <= self.last_pub_ns:    # never publish an older frame after a newer one
                return
            self.last_pub_ns = stamp.nanoseconds
            self.pub.publish(msg)
            t2 = time.perf_counter()
            self.stats['pub'] += 1
            self.stats['dec_ms'] += (t1 - t0) * 1000
            self.stats['pub_ms'] += (t2 - t1) * 1000

    def _report(self):
        s = self.stats
        n = max(s['pub'], 1)
        self.get_logger().info(
            f"camera {s['grab'] / 5:.1f} fps | publishing {s['pub'] / 5:.1f} fps | "
            f"decode {s['dec_ms'] / n:.0f} ms, publish {s['pub_ms'] / n:.0f} ms per frame")
        self.stats = {'grab': 0, 'pub': 0, 'dec_ms': 0.0, 'pub_ms': 0.0}

    def destroy_node(self):
        self.running = False
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = USBCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
