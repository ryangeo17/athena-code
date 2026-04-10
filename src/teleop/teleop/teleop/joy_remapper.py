#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory


class JoyRemapper(Node):

    def __init__(self):
        super().__init__('joy_remapper')

        self.declare_parameter('controller_profile', 'ps4')
        profile_name = self.get_parameter('controller_profile').value

        profile_path = os.path.join(
            get_package_share_directory('teleop'),
            'config',
            f'{profile_name}.yaml',
        )

        if not os.path.exists(profile_path):
            self.get_logger().fatal(f"Controller profile not found: {profile_path}")
            raise FileNotFoundError(profile_path)

        with open(profile_path, 'r') as f:
            profile = yaml.safe_load(f)

        self._deadband: float = profile.get('deadband', 0.0)
        self._axes_config: list = profile['axes']
        self._button_count: int = profile.get('button_count', 13)

        self.get_logger().info(
            f"Loaded controller profile '{profile_name}' "
            f"({len(self._axes_config)} axes, {self._button_count} buttons, "
            f"deadband={self._deadband})"
        )

        self._pub = self.create_publisher(Joy, 'controller_input', 10)
        self._sub = self.create_subscription(Joy, 'joy', self._joy_callback, 10)

    def _apply_axis(self, raw_axes: list, cfg: dict) -> float:
        src = cfg['src']
        invert = cfg.get('invert', False)
        is_trigger = cfg.get('trigger', False)

        if src >= len(raw_axes):
            return 0.0

        value = raw_axes[src]

        if is_trigger:
            # joy_node reports triggers as [-1, 1]; remap to [0, 1]
            value = (value + 1.0) / 2.0
        else:
            # Apply deadband and normalize to clean [-1, 1]
            db = self._deadband
            if abs(value) < db:
                value = 0.0
            elif value > 0.0:
                value = (value - db) / (1.0 - db)
            else:
                value = (value + db) / (1.0 - db)

        if invert:
            value = -value

        return value

    def _joy_callback(self, msg: Joy):
        out = Joy()
        out.header = msg.header

        out.axes = [self._apply_axis(msg.axes, cfg) for cfg in self._axes_config]

        out.buttons = list(msg.buttons[:self._button_count])
        # Pad with zeros if the controller reports fewer buttons than expected
        if len(out.buttons) < self._button_count:
            out.buttons += [0] * (self._button_count - len(out.buttons))

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JoyRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
