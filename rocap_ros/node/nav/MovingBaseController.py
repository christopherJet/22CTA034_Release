#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class MovingBaseController(Node):

    def __init__(self):

        super().__init__('MovingBaseController')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.republish_cmd,
            10)
        
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.publisher_pos = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.timer_period = 0.5  # seconds
        self.last_msg_stamp = rclpy.time.Time()
        self.timer = self.create_timer(self.timer_period, self.keep_alive_check)
        self.posMsg = Float64MultiArray()
        
        self.posMsg.data.append(0)
        self.posMsg.data.append(0)
        self.posMsg.data.append(0)
        self.posMsg.data.append(0)
        self.posMsg.data.append(0)
        self.posMsg.data.append(0)

    def keep_alive_check(self):
        if (self.last_msg_stamp - rclpy.time.Time()) >  rclpy.duration.Duration(seconds=self.timer_period):
            self.send_cmd_to_controller(0,0,0,0,0,0) # software security stop
            

    def republish_cmd(self, msg:Twist):
        self.send_cmd_to_controller(msg.linear.x,
                                    msg.linear.y,
                                    msg.linear.z,
                                    msg.angular.x,
                                    msg.angular.y,
                                    msg.angular.z)

    def send_cmd_to_controller(self, x,y,z,rx,ry,rz):
        cmdMsg =Float64MultiArray()

        cmdMsg.data.append(x)
        cmdMsg.data.append(y)
        cmdMsg.data.append(z)
        cmdMsg.data.append(rx)
        cmdMsg.data.append(ry)
        cmdMsg.data.append(rz)

        self.posMsg.data[0] += x
        self.posMsg.data[1] += y
        self.posMsg.data[2] += z
        self.posMsg.data[3] += rx
        self.posMsg.data[4] += ry
        self.posMsg.data[5] += rz


        self.publisher_vel.publish(cmdMsg)
        self.publisher_pos.publish(self.posMsg)
        self.last_msg_stamp = rclpy.time.Time()
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MovingBaseController()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()