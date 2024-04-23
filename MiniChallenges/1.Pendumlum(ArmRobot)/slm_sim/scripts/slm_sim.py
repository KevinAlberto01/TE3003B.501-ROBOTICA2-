#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

def callback_message(msg):
    global tau
    tau = msg.data

def operation():
    global k,m,l,g,tau,x1,x2,dt

    k = 0.01
    m = 0.75
    l = 0.36
    g = 9.8
    tau = 0.0
    x1 = 0.0
    x2 = 0.0
    dt = 1/100.0 #Step integration 
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
          
        #Equations
        a = l/2
        J = (4/3) * m * (a**2)
        x1 += x2 * dt
        x2_dot = (1/J) * (-m * g * a * np.cos(x1) - k * x2 + tau)
        x2 += x2_dot * dt


        #Publish joint state 
        joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size = 10)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['joint2']
        joint_state_msg.position = [wrap_to_Pi(x1)]
        joint_state_msg.velocity = [x2]
        joint_state_pub.publish(joint_state_msg)
        rate.sleep()



def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi



if __name__=='__main__':
    rospy.init_node("SLM_Sim") #Initialise and Setup node

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Setup the Subscribers
    rospy.Subscriber('tau_topic', Float32, callback_message)

    #Setup de publishers
    print("The SLM sim is Running")
    try:
        operation()
        rospy.loginfo('x1: {}, x2: {}'.format(x1,x2))
        loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node