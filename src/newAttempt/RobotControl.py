#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import roboticstoolbox as rtb
import numpy as np

class ROSNode:
    joint_states =[]
    
    def __init__(self):
        rospy.init_node("robotController")
        rospy.loginfo("Starting ROSNode as robotController.")
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, callback=self.jointStatesCallback, queue_size=5)    
        self.velSub = rospy.Subscriber('/eeVel', Float32MultiArray, callback=self.control, queue_size=5)
        self.controlPub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size= 5)
        self.robot = rtb.models.DH.UR3()
        pass

    def jointStatesCallback(self, msg):
        self.joint_states.clear()
        self.joint_states.append(msg.position[2])
        self.joint_states.append(msg.position[1])
        self.joint_states.append(msg.position[0])
        self.joint_states.append(msg.position[3])
        self.joint_states.append(msg.position[4])
        self.joint_states.append(msg.position[5])
        # print(self.joint_states)

    def control(self,msg):
        dampMax = 0.1 
        thresh = 0.001
        # self.joint_states = rospy.wait_for_message('/joint_states', JointState)
        eevel = np.array(msg.data)
        ctrl_msg = Float64MultiArray()
        if np.linalg.norm(eevel) < 0.001:
            for i in range(6):
                ctrl_msg.data.append(0)
        else:
            jacobian = self.robot.jacob0(self.joint_states)
            mu = self.robot.manipulability(self.joint_states, jacobian, method='yoshikawa', axes='all')
            # print(self.joint_states)
            # print(mu)
            if mu > thresh:
                damp = 0
            else:
                damp = (1 - pow((mu/thresh),2))*dampMax
            term = (np.matmul(jacobian, jacobian.transpose()) + damp * np.identity(6))
            term = np.linalg.inv(term)
            pseudoInverse = np.matmul(jacobian.transpose(),term)
            pseudoInverse = pseudoInverse.transpose()
            # pseudoInverse = np.linalg.pinv(jacobian)
            pseudoInverse = pseudoInverse.transpose()
            # eevel = eevel[0:3]
            control = np.matmul(eevel,pseudoInverse)
            data = control.tolist()
            ctrl_msg.data = data
        print(ctrl_msg.data)
        self.controlPub.publish(ctrl_msg)
        



if __name__ == "__main__":
    robotController = ROSNode()
    rospy.spin()