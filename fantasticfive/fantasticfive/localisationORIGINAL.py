import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from threading import Thread
from rclpy.executors import MultiThreadedExecutor

class MyNode(Node):
    def __init__(self):
        super().__init__('localisation')
        rclpy.get_default_context().on_shutdown(self.cleanup)
        
        # Initialize variables
        self.wl = 0.0
        self.wr = 0.0
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        # ROBOT CONSTANTS
        self.r = 0.05  # wheel radius [m]
        self.L = 0.19  # wheel separation [m]

        # Setup Subscriber
        self.create_subscription(Float32, "/VelocityEncl", self.wl_callback, 1)
        self.create_subscription(Float32, "/VelocityEncR", self.wr_callback, 1)

        # Setup Publisher
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Start the odometry loop in a separate thread
        self.odometry_thread = Thread(target=self.Odometry)
        self.odometry_thread.start()

    def wl_callback(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def cleanup(self):
        self.get_logger().info("Shutting down node.")

    def Odometry(self):
        while self.get_clock().now().nanoseconds == 0:
            self.get_logger().info("No simulated time has been received yet for localisation")
        
        self.get_logger().info("Got time localisation ")

        now = self.get_clock().now()
        previous_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9
       
        # Imprimir el estado
        rate = self.create_rate(20) 
        print("Node localisation initialized")
        while rclpy.ok():
            
            now1 = self.get_clock().now()
            current_t = now1.seconds_nanoseconds()[0] + now1.seconds_nanoseconds()[1] / 1e9
            dt = current_t - previous_time
            # Getting linear and angular velocities
            self.v = self.r * (self.wl + self.wr) / 2.0
            self.w = self.r * (self.wr - self.wl) / self.L

            # Updating the robot's pose
            self.x += self.v * np.cos(self.theta) * dt
            self.y += self.v * np.sin(self.theta) * dt
            self.theta += self.w * dt

            # Creating odometry message
            odometry_msg = Odometry()
            odometry_msg.header.stamp = now1.to_msg()  # Correct way to set the timestamp
            odometry_msg.pose.pose.position.x = self.x
            odometry_msg.pose.pose.position.y = self.y
            odometry_msg.pose.pose.orientation.z = self.theta
            odometry_msg.twist.twist.linear.x = self.v
            odometry_msg.twist.twist.angular.z = self.w
            self.odom_pub.publish(odometry_msg)
            now2 = self.get_clock().now()
            previous_time = now2.seconds_nanoseconds()[0] + now2.seconds_nanoseconds()[1] / 1e9
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    
    # Use MultiThreadedExecutor to handle callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
