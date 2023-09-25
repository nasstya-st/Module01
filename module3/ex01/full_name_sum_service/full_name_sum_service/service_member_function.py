#from example_interfaces.srv import AddTwoInts
from name.srv import FullNameSumService

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(FullNameSumService, 'summ_full_name', self.summ_full_name_callback)

    def summ_full_name_callback(self, request, response):
        response.full_name = ' '.join([request.last_name, request.name, request.first_name])
        self.get_logger().info(f'Incoming request\nlast name: {request.last_name} name: {request.name} first name: {request.first_name}' )

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
