import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Float32MultiArray
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import numpy as np
import time


class MyNode(Node):
    def __init__(self):
        super().__init__('time')
        #rclpy.spin_once(self)
        rclpy.get_default_context().on_shutdown(self.cleanup)
        self.declare_parameters(namespace = '', parameters = [('float_number', rclpy.Parameter.Type.DOUBLE)])
        self.Ts = self.get_parameter('float_number').value  # Almacenar el valor del parámetro en una variable de instancia
        self.get_logger().info(f"Obtained parameter: {self.Ts}") #MUESTRA ESTO

          # Almacenar el valor del parámetro en una variable de instancia
        ############ ROBOT CONSTANTS ################  
        r=0.05 #wheel radius [m] 
        L=0.19 #wheel separation [m] 
        ###########  INITIALIZE VARIABLES ################ 
        self.thetar = 0.0 #[rads]
        self.xr = 0.0 #[m] posicion del robot a lo largo del eje x
        self.yr = 0.0 #[m] posicion del robot a lo largo del eje y
        v = 0.0 #[m/s] velocidad lineal del robot
        w = 0.0 #[rad/s] velocidad angular del robot
        # WHEELS SPEED 
        self.wl = 0.0 #[rad/s] velocidad angular de la rueda izq
        self.wr = 0.0 #[rad/s] velocidad angular de la rueda derecha
        # DISTANCES
        self.d = 100000.0
        self.d_min = 0.05 #[m] min distance to the goal
        e_theta_min = np.pi/90
        # Vels max and Min
        w_max = 0.5
        w_min = 0.05
        v_max = 0.4
        v_min = 0.05
        #Flags to request the yaml positions
        self.flag= Int32()
        self.flag.data = 1 ################Modifique esto valor original 1 antes era sel.flag.data
        #Objective points
        self.xG = 0.0
        self.yG = 0.0
        #Linear velocity errors
        e_theta = 0.0
        e1 = 0.0
        e2 = 0.0
        #Angular velocity errors
        e3 = 0.0
        e4 = 0.0
        e5 = 0.0
        #Control input for angular velocity
        u0 = 0.0
        u1 = 0.0
        #Control input for linear velocity
        u2 = 0.0
        u3 = 0.0
        #Control gains for angular velocity
        kp = 2
        ki = 0.0
        kd = 0.0
        #Control gains for linear velocity
        kp1 = 0.3
        ki1 = 0.0
        kd1 = 0.0

        #Time for controlling angular velocity
        #CAMBIAR ESTO
        #Ts = rospy.get_param("/time", 0.1)
        Ts = self.Ts
        #Time for controlling linear velocity
        #Ts1 = rospy.get_param("/time", 0.1)
        Ts1 = self.Ts

        #Angular velocity controller
        K1 = kp + Ts * ki + kd/Ts
        K2 = -kp - 2.0 * kd / Ts
        K3 = kd/Ts

        #Linear velocity controller
        K4 = kp1 + Ts1 * ki1 + kd1/Ts1
        K5 = -kp1 - 2.0 * kd1 / Ts1
        K6 = kd1/Ts1

        ###########  PUBLISHERS AND SUBSCRIBERS ################
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.flag_pub = self.create_publisher(Int32, '/flag', 10)
        self.create_subscription(Float32, '/VelocityEncL', self.wl_cb, 1)
        self.create_subscription(Float32, '/VelocityEncR', self.wr_cb, 1)
        self.create_subscription(Pose, '/pose', self.pose_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odometry_cb, 1)

        #Velocity message
        v_msg = Twist()

        while self.get_clock().now().nanoseconds == 0:  #########################PREGUNTARLE A SAM 
            self.get_logger().info("No simulated time has been received yet") #NO MUESTRA
        
        self.get_logger().info("Got time controller ")
        now = self.get_clock().now()
        previous_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9


        self.get_logger().info(f'Current time: {now.to_msg()}')
        self.get_logger().info(f'Previous time (seconds): {previous_time}')
        # Imprimir el estado
        self.get_logger().info("Estado antes de iniciar") ################MUESTRA
        rate = self.create_rate(80)
        self.get_logger().info("Node initialized")

        while rclpy.ok(): 
            rclpy.spin_once(self)
            self.get_logger().info("rclply")
            self.get_logger().info("Valor de d: %s" % self.d) ###MUESTRA 0
            self.get_logger().info("Valor de d_min: %s" % self.d_min) ###MUESTRA 0
            self.get_logger().info("Valor de FLAG: %s" % self.flag.data) ###MUESTRA 0

            if self.d >= self.d_min and self.flag.data == 0:   
                self.get_logger().info("IF DEL CONTROLLER ")
                now1 = self.get_clock().now()
                current_t = now1.seconds_nanoseconds()[0] + now1.seconds_nanoseconds()[1] / 1e9
                delta_t = current_t- previous_time
                now2 = self.get_clock().now()
                previous_time = now2.seconds_nanoseconds()[0] + now2.seconds_nanoseconds()[1] / 1e9
                self.v = r*(self.wl+self.wr)/2.0
                self.w = r*(self.wr-self.wl)/L
                
                # Distance from the goal
                self.d = np.sqrt((self.xG - self.xr)**2 + (self.yG - self.yr)**2)
                self.get_logger().info("xG:" + str(self.xG))
                self.get_logger().info("yG:" + str(self.yG))
                #print("xG:" + str(self.xG))
                #print("yG:" + str(self.yG))
                e3 = self.d

                #Angle to reach the goal
                theta_g = np.arctan2(self.yG - self.yr, self.xG - self.xr)
                print("teta g" , theta_g)
                #Angle's error
                e_theta = theta_g - self.thetar 

                if(abs(e_theta)<0.03):
                    v = kp*self.d
                    w = 0
                else:
                    w = -kp1*e_theta
                    v = 0
                    print("w ",w )

                if(abs(self.d)<.03 and (abs(e_theta)<= .03)):
                    v = 0
                    w = 0
                              
                v_msg.linear.x = float(v)
                v_msg.angular.z = float(w)

            else:
                self.get_logger().info("ELSE DEL CONTROLLER")
                print("Stop")
                self.flag.data = 1 #MODIFIQUE ESTO
                self.flag_pub.publish(self.flag) #MODIFIQUE ESTO
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                self.d = 100000.0

            self.pub_cmd_vel.publish(v_msg)
            self.get_logger().info("Velocidad lineal: %s" % str(v))
            self.get_logger().info("Velocidad angular: %s" % str(w))
            rate.sleep()
            
    def cleanup(self):
        # This function is called just before finishing the node
        v_msg = Twist()
        self.pub_cmd_vel.publish(v_msg)
        #PREGUNTAR SI ESTO ES PARA LIMPIAR EL FLAG
        self.flag_pub.publish(self.flag)

    def wl_cb(self, wl):
        # This function receives the left wheel speed from the encoders
        self.wl = wl.data
    def wr_cb(self, wr):
        # This function receives the left wheel speed from the encoders
        self.wr = wr.data

    def pose_cb(self, msg): #NO HACE ESO 
        self.xG = msg.position.x
        self.yG = msg.position.y
        self.flag.data = 0 ##MODIFIQUE ESTO LE AGREGUE DATA
        self.flag_pub.publish(self.flag) ##MODIFIQUE ESTO LE AGREGUE DATA
        self.get_logger().info("Callback pose done in controller node")
        self.get_logger().info("x deseada: %s" % str(self.xG))
        self.get_logger().info("y deseada: %s" % str(self.yG))

    def odometry_cb(self, msg):
        global position_x, position_y, theta
        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        self.thetar = msg.pose.pose.orientation.z


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()