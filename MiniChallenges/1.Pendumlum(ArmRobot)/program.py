#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

#Define the callback functions
def callback_tau(msg):
     #rospy.loginfo
     global tau
     tau = msg.data

def dynamics():
    global k,m,l,g,tau,x1,x2,dt
    k = 0.01; #friction
    m = 0.75; #pendulum mass
    l = 0.36; #pendulum length
    g = 9.8;  #gravity
    tau = 0.0; #entrance of the system
    x1 = 0.0;  #ouput joint1
    x2 = 0.0;  #output joint2
    dt = 1/100.0 #time differential/step of integration
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # SLM governing equation
        a = l/2
        J = 4.0/3.0*(m*(a**2))
        x1 += x2 * dt  # Update x1 based on x2
        x2_dot = 1 / J * ((tau - m * g * a * np.cos(x1)) - (k * x2))
        x2 += x2_dot * dt  # Update x2 based on x2_dot

        # Publish joint state
        joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['joint2']
        joint_state_msg.position = [wrap_to_Pi(x1)]
        joint_state_msg.velocity = [x2]
        joint_state_pub.publish(joint_state_msg)
        rate.sleep()

  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))


    # Setup the Subscribers
    rospy.Subscriber('tau_topic', Float32, callback=callback_tau)


    #Setup de publishers

    print("The SLM sim is Running")
    try:
        dynamics()
        rospy.loginfo('x1: {}, x2: {}'.format(x1,x2))
        loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
