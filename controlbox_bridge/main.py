#!/usr/bin/env python
import rclpy
from controlbox_bridge.pressure_interface import PressureInterface

def main(args=None):
    rclpy.init(args=args)
    pressure_interface = PressureInterface()
    try:
        rclpy.spin(pressure_interface)
    except KeyboardInterrupt:
        pressure_interface.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        pressure_interface.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()