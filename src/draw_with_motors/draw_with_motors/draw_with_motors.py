import rclpy
import math
import numpy as np
import time
from rclpy.node import Node

from dynamixel_sdk_custom_interfaces.msg import SetPosition
from std_msgs.msg import Int32

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.publisher_ = self.create_publisher(SetPosition, 'set_position', 10)
        #timer_period = 2  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(Int32, 'draw_number', self.number_callback, 10)


        self.position_69 = 100
        self.position_105 = 603
        self.r_1 = 7.25
        self.r_2 = 7.9
        self.r_3 = 1.9

    def number_callback(self, msg):
        self.draw_number(msg.data)

    def timer_callback(self):
        self.draw_number(8)

    def angle_calculation_left(self, x,y):
        theta_1 = math.atan2(y,x) + math.acos((self.r_1**2 - self.r_2**2 + x**2 + y**2)/(2*self.r_1*math.sqrt(x**2 + y**2)))
        theta_1 = theta_1*180/np.pi
        self.get_logger().info(f'id=69, position={int(theta_1)}')
        angle_bits = self.position_69 + 320/95*(theta_1 - 60)
        return angle_bits

    def angle_calculation_right(self, x,y):
        theta_2 = math.atan2(y,x) - math.acos((self.r_1**2 - self.r_2**2 + x**2 + y**2)/(2*self.r_1*math.sqrt(x**2 + y**2)))
        theta_2 = theta_2*180/np.pi
        self.get_logger().info(f'id=105, position={int(theta_2)}')
        angle_bits = self.position_105 + 320/95 * (theta_2 - 22)
        return angle_bits

    def draw_number(self, number):
        coordinates = self.convert_number_to_coordinates(number)
        coordinates = self.pad_extra_points(coordinates, 10)
        coordinates_bits = self.convert_coordinates_to_bits(coordinates)

        for coordinate in coordinates_bits:
            msg = SetPosition()
            msg.id = 69
            msg.position = int(coordinate[0])
            self.publisher_.publish(msg)
            self.get_logger().info(f'id={msg.id}, position={msg.position}')

            msg = SetPosition()
            msg.id = 105
            msg.position = int(coordinate[1])
            self.publisher_.publish(msg)
            self.get_logger().info(f'id={msg.id}, position={msg.position}')
            time.sleep(0.01)            

    def convert_coordinates_to_bits(self, coordinates):
        bits = []
        for i in range(len(coordinates)):
            x = coordinates[i][0]
            y = coordinates[i][1]
            bits.append([self.angle_calculation_left(x+self.r_3, y), 
                         self.angle_calculation_right(x-self.r_3, y)])
        return bits

    def pad_extra_points(self, points, number_of_padding_points):
        line = []
        for i in range(len(points)-1):
            x1 = points[i][0]
            y1 = points[i][1]
            x2 = points[i+1][0]
            y2 = points[i+1][1]

            x = np.linspace(x1, x2, number_of_padding_points)
            y = np.linspace(y1, y2, number_of_padding_points)

            for j in range(len(x)):
                line.append([x[j], y[j]])

        return line

    def convert_number_to_coordinates(self, number):
        # x Coordinates
        LX = -1#-2.5
        RX =  1#2.5

        # y Coordinates
        TY = 14#13.0
        MY = 12.0
        BY = 10#3.0

        # Numbers 
        if number == 0:
            coordinates = [[RX, MY], [RX, TY], [LX, TY], [LX, MY], [LX, BY], [RX, BY], [RX, MY]]
        elif number == 1:
            coordinates = [[RX, MY], [RX, TY], [RX, MY], [RX, BY], [RX, MY]]
        elif number == 2:
            coordinates = [[RX, MY], [RX, TY], [LX, TY], [RX, TY], [RX, MY], [LX, MY], [LX, BY], [RX, BY], [LX, BY], [LX, MY], [RX, MY]]
        elif number == 3:
            coordinates = [[RX, MY], [RX, TY], [LX, TY], [RX, TY], [RX, MY], [LX, MY], [RX, MY], [RX, BY], [LX, BY], [RX, BY], [RX, MY]]
        elif number == 4:
            coordinates = [[RX, MY], [LX, MY], [LX, TY], [LX, MY], [RX, MY], [RX, BY], [RX, MY], [RX, TY], [RX, MY]]
        elif number == 5:
            coordinates = [[RX, MY], [LX, MY], [LX, TY], [RX, TY], [LX, TY], [LX, MY], [RX, MY], [RX, BY], [LX, BY], [RX, BY], [RX, MY]]
        elif number == 6:
            coordinates = [[RX, MY], [LX, MY], [LX, TY], [RX, TY], [LX, TY], [LX, MY], [LX, BY], [RX, BY], [RX, MY]]
        elif number == 7:
            coordinates = [[RX, MY], [RX, BY], [RX, MY], [RX, TY], [LX, TY], [RX, TY], [RX, MY]]
        elif number == 8:
            coordinates = [[RX, MY], [RX, TY], [LX, TY], [LX, MY], [RX, MY], [RX, BY], [LX, BY], [LX, MY], [RX, MY]]
        else:
            coordinates = [[RX, MY], [LX, MY], [LX, TY], [RX, TY], [RX, MY], [RX, BY], [RX, MY]]
        
        return coordinates

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()