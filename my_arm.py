

import math



class my_arm:
    def __init__(self, arm, timer):
        self.arm = arm
        self.time = timer
        self.current_theta1 = 0
        self.current_theta2 = 0

    def forward_kinematics(self, theta1, theta2):
        self.arm.go_to(1, theta1)
        self.arm.go_to(3, theta2)
        L1 = 0.4 # estimated using V-REP (joint2 - joint4)
        L2 = 0.7 # estimated using V-REP (joint4 - joint6)
        z = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2) + 0.3105
        x = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
        self.current_theta1 = theta1
        self.current_theta2 = theta2
        print("Go to {},{} deg, FK: [{},{},{}]".format(math.degrees(theta1), math.degrees(theta2), x, 0, z))

    def inverse_kinematics(self, x_i, z_i, smooth = False):
        L1 = 0.4 # estimated using V-REP (joint2 - joint4)
        L2 = 0.585 # estimated using V-REP (joint4 - joint6)
        # Corrections for our coordinate system
        z = z_i - 0.3105
        x = x_i
        # compute inverse kinematics
        r = math.sqrt(x*x + z*z)
        temp = (L1*L1 + L2*L2 - r*r) / (2*L1*L2)
        if temp > 1:
            temp = 1
        elif temp < -1:
            temp = -1
        alpha = math.acos(temp)
        theta2 = math.pi - alpha

        temp2 = (r*r + L1*L1 - L2*L2) / (2*L1*r)
        if temp2 > 1:
            temp2 = 1
        elif temp2 < -1:
            temp2 = -1
        beta = math.acos(temp2)
        theta1 = math.atan2(x, z) - beta
        if theta2 < -2*math.pi / 3.0 or theta2 > 2*math.pi / 3.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            theta2 = math.pi + alpha
            theta1 = math.atan2(x, z) + beta
        if theta2 < -2*math.pi / 3.0 or theta2 > 2*math.pi / 3.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            print("Not possible", math.degrees(theta1), math.degrees(theta2))
            return

        #smoothing of the motion
        if smooth:
            self.smooth_goto(theta1, theta2)
        else:
            self.arm.go_to(1, theta1)
            self.arm.go_to(3, theta2)

        self.current_theta1 = theta1
        self.current_theta2 = theta2
        print("Go to [{},{}], IK: [{} deg, {} deg]".format(x_i, z_i, math.degrees(theta1), math.degrees(theta2)))


    def level_effector(self, smooth = False):
        end_theta = (self.current_theta1 + self.current_theta2)
        if end_theta > math.pi/2:
            eff_theta = math.fmod(-math.pi/2-end_theta, math.pi/2)
        elif -0.0000001 < end_theta < 0.000001:
            eff_theta = math.pi/2
        elif end_theta > 0:
            eff_theta = math.fmod(math.pi/2-end_theta, math.pi/2)
        else: #end_theta < 0
            eff_theta = math.fmod(math.pi/2+end_theta, math.pi/2)
        print(math.degrees(end_theta), math.degrees(eff_theta))
        if smooth:
            self.arm.go_to(5, eff_theta/3)
            self.time.sleep(0.1)
            self.arm.go_to(5, 2*eff_theta/3)
            self.time.sleep(0.1)
            self.arm.go_to(5, eff_theta)
        else:
            self.arm.go_to(5, eff_theta)

    def smooth_goto(self, theta1, theta2):
        step = 15
        step_t1 = (theta1 - self.current_theta1)/step
        step_t2 = (theta2 - self.current_theta2)/step
        accumulate_t1 = self.current_theta1 + step_t1
        accumulate_t2 = self.current_theta2 + step_t2
        for i in range(step):
            self.current_theta1 = accumulate_t1
            self.current_theta2 = accumulate_t2
            self.arm.go_to(1, accumulate_t1)
            self.arm.go_to(3, accumulate_t2)
            self.time.sleep(0.001)
            self.level_effector()
            accumulate_t1 += step_t1
            accumulate_t2 += step_t2

    def smooth_rotation(self, radians):
        step = 25
        for i in range(1, step):
            self.arm.go_to(4, i * radians/step)
            self.time.sleep(0.001)
