#!/usr/bin/env python3
import os
import csv
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Vector3Stamped

HAS_DISPLAY = bool(os.environ.get('DISPLAY'))
plt = None
if HAS_DISPLAY:
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        # smoke-test: try creating a figure to catch headless-only installs
        _test_fig = plt.figure()
        plt.close(_test_fig)
    except Exception:
        HAS_DISPLAY = False
        plt = None


class AngleErrorPlotter(Node):
    def __init__(self):
        super().__init__('angle_error_plotter')

        self.declare_parameter('topic_name', '/debug/angle_error')
        self.declare_parameter('window_sec', 20.0)
        self.declare_parameter('save_csv', True)
        self.declare_parameter('save_dir', 'logs/angle_error')

        topic = self.get_parameter('topic_name').get_parameter_value().string_value
        self.window_sec = self.get_parameter('window_sec').get_parameter_value().double_value
        self.save_csv = self.get_parameter('save_csv').get_parameter_value().bool_value
        save_dir = self.get_parameter('save_dir').get_parameter_value().string_value

        self.data = []  # list of (t, yaw_err_deg, pitch_err_deg, fire)
        self.t0 = None
        self.lock = threading.Lock()

        if self.save_csv:
            os.makedirs(save_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            self.csv_path = os.path.join(save_dir, f'angle_error_{ts}.csv')
            self.csv_file = open(self.csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['time_s', 'yaw_error_deg', 'pitch_error_deg', 'fire'])
            self.get_logger().info(f'CSV saving to {self.csv_path}')
        else:
            self.csv_file = None

        self.sub = self.create_subscription(
            Vector3Stamped, topic, self._callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            ))

        if HAS_DISPLAY:
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
            self.fig.canvas.manager.set_window_title('Angle Error')

            self.line_yaw, = self.ax1.plot([], [], 'b-', linewidth=1.0)
            self.ax1.set_ylabel('Yaw Error (deg)')
            self.ax1.set_title('Angle Error over Time')
            self.ax1.grid(True, alpha=0.3)

            self.line_pitch, = self.ax2.plot([], [], 'r-', linewidth=1.0)
            self.ax2.set_xlabel('Time (s)')
            self.ax2.set_ylabel('Pitch Error (deg)')
            self.ax2.grid(True, alpha=0.3)

            self.fig.tight_layout()
            self.get_logger().info('Angle error plotter started (GUI mode)')
        else:
            self.get_logger().warn(
                'No display available. Running in CSV-only mode.')
            self.timer = self.create_timer(1.0, self._log_status)

        self.get_logger().info(
            f'Subscribed to {topic}, window={self.window_sec}s')

    def _callback(self, msg: Vector3Stamped):
        t = time.time()
        if self.t0 is None:
            self.t0 = t
        with self.lock:
            self.data.append((t - self.t0, msg.vector.x, msg.vector.y, msg.vector.z))
            cutoff = t - self.t0 - self.window_sec
            while self.data and self.data[0][0] < cutoff:
                self.data.pop(0)
            if self.csv_writer:
                self.csv_writer.writerow(
                    [self.data[-1][0], msg.vector.x, msg.vector.y, msg.vector.z])

    def _refresh_gui(self):
        with self.lock:
            if not self.data:
                return
            ts = [d[0] for d in self.data]
            yaws = [d[1] for d in self.data]
            pitches = [d[2] for d in self.data]
            t_last = ts[-1]

        self.line_yaw.set_data(ts, yaws)
        self.line_pitch.set_data(ts, pitches)

        x_min = max(0, t_last - self.window_sec)
        x_max = max(t_last, self.window_sec)
        self.ax1.set_xlim(x_min, x_max)
        self.ax2.set_xlim(x_min, x_max)
        self.ax1.relim()
        self.ax2.relim()
        self.ax1.autoscale_view(scaley=True)
        self.ax2.autoscale_view(scaley=True)

    def _log_status(self):
        with self.lock:
            if self.data:
                d = self.data[-1]
                self.get_logger().info(
                    f'yaw_err={d[1]:.2f}deg, pitch_err={d[2]:.2f}deg, '
                    f'fire={d[3]:.0f}, samples={len(self.data)}')

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'CSV saved to {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AngleErrorPlotter()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if HAS_DISPLAY and plt:
            while plt.fignum_exists(node.fig.number):
                node._refresh_gui()
                plt.pause(0.05)
        else:
            while rclpy.ok():
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if HAS_DISPLAY and plt:
            plt.close('all')
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
