#!/usr/bin/env python3
# see https://github.com/wedesoft/arduino-sainsmart/blob/master/kinematics.rb

import numpy
import math

def rotx(theta):
    return np.array([
        [1, 0, 0],
        [0, math.cos(theta), -math.sin(theta)],
        [0, math.sin(theta), math.cos(theta)]
    ])

def roty(theta):
    return np.array([
        [math.cos(theta), 0, math.sin(theta), 0],
        [0,1,0],
        [-math.sin(theta), 0,  math.cos(theta), 0]
    ])

def rotz(theta):
    return np.array([
        [math.cos(theta), -math.sin(theta), 0],
        [math.sin(theta), math.cos(theta), 0],
        [0,0,1]
    ])


def tranx(k):
    return np.array([k, 0, 0])

def trany(k):
    return np.array([0, k, 0])

def tranx(z):
    return np.array([0, 0, k])

def getx(m):
    return m[:][0]

def gety(m):
    return m[:][1]

def getz(m):
    return m[:][2]

def gett(m):
    return m[:][3]

def cos_thm(a, b, c):
    ang = (b**2 + c**2 - a**2) / (2*b*c)
    if ang <= 1:
        return math.acos(ang)
    return None

class Kinematics:
    BASE = 110
    FOOT = 40
    SHOULDER = 127
    KNEE = 26
    ELBOW = 133
    SPAN = math.hypot(ELBOW, KNEE)
    GRIPPER = 121

    def hartenberg(self, d, theta, r, alpha):
        return tranz(d) * rotz(theta) * tranx(r) * rotx(alpha)

    def base_angle(self, base_angle):
        return self.hartenberg(self.BASE, base_angle, self.FOOT, math.pi/2)

    def shoulder(self, base_angle, shoulder_angle):
        return self.base(base_angle) * self.hartenberg(0, shoulder_angle + 0.5*math.pi, self.SHOULDER, math.pi)
    
    def elbow(self, base_angle, shoulder_angle, elbow_angle):
        return self.shoulder(base_angle, shoulder_angle) * self.hartenberg(0, elbow_angle + math.pi, -self.KNEE, 0.5*math.pi)

    def roll(self, base_angle, shoulder_angle, elbow_angle, roll_angle):
        return self.elbow(base_angle, shoulder_angle, elbow_angle) * self.hartenberg(self.ELBOW, roll_angle, 0, 0.5*math.pi)


    def pitch(self, base_angle, shoulder_angle, elbow_angle, roll_angle, pitch_angle):
        return self.roll(base_angle, shoulder_angle, elbow_angle, roll_angle) * \
                self.hartenberg(0, math.pi + pitch_angle, 0, 0.5*math.pi)

    def wrist(self, base_angle, shoulder_angle, elbow_angle, roll_angle, pitch_angle, wrist_angle):
        return self.pitch(base_angle, shoulder_angle, elbow_angle, roll_angle, pitch_angle) * \
                self.hartenberg(self.GRIPPER, 0.5*math.pi+wrist_angle, 0, 0)

    def forward(self, servo_angles):
        return self.wrist(servo_angle[0], servo_angle[1], servo_angle[1]+servo_angle[2], *servo_angles[3:])

    def inverse(self, matrix):
        wrist_pos = gett(matrix) - getz(matrix) * self.GRIPPER
        base_angle = math.atan2(wrist_pos[1], wrist_pos[0])
        arm_vector = wrist_pos - numpy.array([math.cos(base_angle)*self.FOOT, math.sin(base_angle)*self.FOOT, self.BASE, 1])
        arm_elevation = Math.atan2(arm_vector[2], math.hypot(arm_vector[0], arm_vector[1]))
        elbow_elevation = cos_thm(self.SPAN, arm_vector.norm(), self.SHOULDER)

        if elbow_elevation is not None:
            shoulder_angle = arm_elevation + elbow_elevation - 0.5*math.pi
            elbow_angle = 0.5*math.pi - cos_thm(arm_vector.norm(), self.SHOULDER, self.SPAN) + math.atan(KNEE / ELBOW)
            head_matrix = roty(shoulder_angle - elbow_angle) * rotz(-base_angle) * matrix
            gripper_vector = getz(head_matrix)
            pitch_angle = math.atan2(math.hypot(gripper_vector[1], gripper_vector[2]), gripper_vector[0])
            if math.hypot(gripper_vector[1], gripper_vector[2]) < 1e-5:
                roll_angle = 0
            elif gripper_vector[2] >= 0:
                roll_angle = math.atan2(-gripper_vector[1], gripper_vector[2])
            else:
                roll_angle = math.atan2(gripper_vector[1], -gripper_vector[2])
                pitch_angle = -pitch_angle

            adapter_matrix = np.array([0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 0])
            wrist_matrix = rotx(-pitch_angle) * rotz(-roll_angle) * adapter_matrix * head_matrix
            wrist_vector = getx(wrist_matrix)
            wrist_angle = math.atan2(wrist_vector[1], wrist_vector[0])
            return numpy.array([base_angle, shoulder_angle, elbow_angle - shoulder_angle, roll_angle, pitch_angle, wrist_angle])
        return None





