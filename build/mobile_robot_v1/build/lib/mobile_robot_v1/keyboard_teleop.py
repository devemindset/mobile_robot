#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.publisher_ = self.create_publisher(Twist,"cmd_vel",10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        #initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((300,300))

        #create a Twist message instance
        self.twist = Twist()

        #Define key mapping to velocity commands
        self.key_mapping = {
            K_UP:(0.0,0.5),
            K_DOWN:(0.0,-0.5),
            K_LEFT:(0.5,0.0),
            K_RIGHT:(-0.5,0.0),
            K_w:(0.0,0.5),
            K_s:(0.0,-0.5),
            K_a:(0.5,0.0),
            K_d:(-0.5,0.0)
        }
    def timer_callback(self):
        for event in pygame.event.get():
            if event.type == KEYDOWN or event.type == KEYUP:
                key = event.key
                if key in self.key_mapping:
                    v, omega = self.key_mapping[key]
                    if event.type ==KEYUP:
                        v = 0.0
                        omega = 0.0
                    self.twist.linear.x = v 
                    self.twist.angular.z = omega
                    self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    
    node = KeyboardTeleop()

    try : 
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        #Ensure that robot stops when the node is shut down
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.publisher_.publish(node.twist)
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == "__main__":
    main()