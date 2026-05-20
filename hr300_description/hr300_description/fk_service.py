#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# ВАЖНО: импортируем из ОТДЕЛЬНОГО пакета интерфейсов
from hr300_description.srv import FK


class FKService(Node):
    def __init__(self):
        super().__init__('fk_service')
        self.create_service(FK, 'fk', self.handle_fk)
        self.get_logger().info('FK service ready: call /fk(a,b,c,d,e) -> (z_sum, y_diff, z_prod)')

    def handle_fk(self, request, response):
        a, b, c, d, e = request.a, request.b, request.c, request.d, request.e
        response.z_sum  = float(a + b + c + d + e)
        response.y_diff = float(a - b - c - d - e)
        response.z_prod = float(a * b * c * d * e)
        return response


def main():
    rclpy.init()
    node = FKService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
