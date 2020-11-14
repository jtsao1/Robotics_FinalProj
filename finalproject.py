from pyCreate2 import create2
import math
import odometry
import pid_controller
import lab8_map
import rrt_map
import particle_filter
import rrt
import numpy as np
import my_arm


""" FOR GRADING"""
armPos = [1.60, 3.3999]  # arm(x,y)



class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        x_init = self.create.sim_get_position()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.arm = factory.create_kuka_lbr4p()
        self.myarm = my_arm.my_arm(self.arm, self.time)
        self.virtual_create = factory.create_virtual_create()
        # self.virtual_create = factory.create_virtual_create("192.168.1.XXX")
        self.odometry = odometry.Odometry()
        self.mapJ = lab8_map.Map("lab8_map.json")
        self.map = rrt_map.Map("configuration_space.png")
        self.rrt = rrt.RRT(self.map)

        # TODO identify good PID controller gains
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter(self.mapJ, 600, (*self.create.sim_get_position(), 0))

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_goal(self, goal_x, goal_y):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        while math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2)) > 0.05:
            state = self.create.update()
            if state is not None:
                # go to p(x,y) in path - ODOMETRY -
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)
        self.create.drive_direct(0 , 0)
        # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

    def go_to_angle(self, goal_theta): # turn to angle in place, moves pf
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
            math.cos(goal_theta - self.odometry.theta))) > 0.05:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self): # moves forward 0.5, moves pf
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        while True:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))

            # stop if close enough to goal
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def visualize(self): # shows bot estimates and pf updates
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def take_measurements(self):
        angle = -90
        while angle <= 90:
            self.servo.go_to(angle)
            self.time.sleep(2.0)
            distance = self.sonar.get_distance()
            self.pf.move_by(0, 0, 0)     #introduce randomness
            self.pf.measure(distance, math.radians(angle))
            self.update_odo()
            self.visualize()
            angle += 45
        self.servo.go_to(0)     #servo return back to default position
        self.time.sleep(1.0)

    def update_odo(self, it = 4, path_len = 100, always_update = False):
        # it and path_len are used in the particles filter
        est = self.pf.get_estimate()
        if always_update:   #only used when approaching
            self.odometry.x += (est[0] - self.odometry.x )/4
            self.odometry.y += (est[1] - self.odometry.y )/4
            self.odometry.theta += (est[2] - self.odometry.theta )/4
            print("UPDATE")
        elif self.pf.need_update() and  (3 < it < (path_len - 3)):     #used during the path following to avoid obstacles
            self.odometry.x += (est[0] - self.odometry.x )/2
            self.odometry.y += (est[1] - self.odometry.y )/2
            self.odometry.theta += (est[2] - self.odometry.theta )/2
            print("UPDATE")
        print(self.pf.get_effectiness())
        print("@ [{},{}]".format(self.odometry.x, self.odometry.y))
        self.create.sim_get_position()
        print("Actual ", self.create.sim_get_position())
        print()

    def run(self):
        self.create.start()
        self.create.safe()
        self.arm.open_gripper()
        self.create.drive_direct(0, 0)
        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        self.visualize()
        #self.virtual_create.enable_buttons()


        '''
        ' 1 - FIND PATH
        '''
        # build rrt
        x_init = self.create.sim_get_position()
        print(x_init)
        print("bot initial pos: " + str(x_init))
        x_mapInit = [x_init[0]*100, 300 - x_init[1]*100]
        self.rrt.build(x_mapInit, 650, 20)      #(starting location, iteration, step length)
        # given arm position & dimensions
        L0 = 0.31
        L1 = 0.4
        L2 = 0.39
        Lgrip = 0.16
        # calculate x_goal for robot
        d2wall = 0.6 # set end distance to wall=0.8 (inside maze, from wall to robot)
        offset = 0.4 # set end distance to wall=0.4, where
        x_goal = armPos[:]  #goal location of rrt (x,y, theta)
        x_final = armPos[:] #goal location that is reachable from arm (x,y)
        appraoch_pos = None # an iterable that returns the approching location
        approach_step = 0.6    #step length for each approaching step
        if armPos[0] < 0:
            x_goal[0] = d2wall  #left side of the maze
            x_goal.append(math.pi)  # goal theta
            x_final[0] = offset
        elif armPos[1] < 0:
            x_goal[1] = d2wall  #bottom side of the maze
            x_goal.append(-math.pi/2)
            x_final[1] = offset
        elif armPos[0] > 3:
            x_goal[0] = 3-d2wall    #right side of the maze
            x_goal.append(0)
            x_final[0] = 3-offset
        elif armPos[1] > 3:
            x_goal[1] = 3-d2wall    #top side of the maze
            x_goal.append(math.pi/2)
            x_final[1] = 3-offset

        # rrt planing
        x_pixel_goal = [x_goal[0]*100, 300-x_goal[1]*100] #convert to pixel size
        #x_mapGoal = self.rrt.nearest_neighbor(x_pixel_goal)
        path = self.rrt.shortest_path(x_pixel_goal)
        for v in self.rrt.tree:             # draw the tree
            for u in v.neighbors:
                self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0,0,0))
        for idx in range(0, len(path)-1):           # draw the path
            self.map.draw_line((path[idx][0], path[idx][1]), (path[idx+1][0], path[idx+1][1]), (0,255,0))
        self.map.save("fp_rrt.png")

        '''
        ' 2 - FOLLOW PATH
        '''
        self.odometry.x = x_init[0]
        self.odometry.y = x_init[1]

        base_speed = 100
        it = 0   # steps until updating PF
        for p in path:
            it+=1
            goal_x = p[0] / 100.0 # conversion between px -> m
            goal_y = 3 - p[1] / 100.0 ###### NOT SURE WHAT 3.35 IS
            # print("goto_", goal_x, goal_y)
            self.go_to_goal(goal_x, goal_y)

            '''
            ' 2.5 - LOCALIZE
            '''
            distance = self.sonar.get_distance()
            if distance != 3.33:                # invalid reading
                self.pf.measure(distance, 0)
            self.visualize()
            self.update_odo(it, len(path))

        '''
        ' 2.7 - APPROACHING
        '''
        self.create.drive_direct(0, 0)
        self.take_measurements()    #rotate servo 180degrees around and measure
        # go to final location
        self.go_to_angle(x_goal[2])
        self.take_measurements()
        self.go_to_angle(x_goal[2])

        if armPos[0] < 0:
            appraoch_pos = [(i, x_goal[1]) for i in np.arange(self.odometry.x-approach_step, x_final[0], -approach_step)]
        elif armPos[1] < 0:
            appraoch_pos = [(x_goal[0], i) for i in np.arange(self.odometry.y-approach_step, x_final[1], -approach_step)]
        elif armPos[0] > 3:
            appraoch_pos = [(i, x_goal[1]) for i in np.arange(self.odometry.x+approach_step, x_final[0], approach_step)]
        elif armPos[1] > 3:
            appraoch_pos = [(x_goal[0], i) for i in np.arange(self.odometry.y+approach_step, x_final[1], approach_step)]


        for x,y in appraoch_pos:
            print("Approaching", x,y)
            self.go_to_goal(x, y)
            distance = self.sonar.get_distance()
            if distance != 3.33:
                self.pf.measure(distance, 0)
            self.visualize()
            self.update_odo(always_update = True)


        self.time.sleep(1)
        print("ODO ", self.odometry.x, self.odometry.y)
        print("Actual ", self.create.sim_get_position(), "\n")

        '''
        ' 3 - ARM Ops
        '''
        cup_height = 0.18
        d2reach = 0     #used for inverser kinematics
        if armPos[0] < 0 or armPos[0] > 3:
            d2reach = abs(self.odometry.x - armPos[0])  #left/right side of the maze
        elif armPos[1] < 0 or armPos[1] > 3:
            d2reach = abs(self.odometry.y - armPos[1])  #bottom/top side of the maze


        #pass robot location to arm
        #(x/y+wall-distance, cup_height)
        self.myarm.inverse_kinematics(d2reach, cup_height)

        self.time.sleep(2)

        self.arm.close_gripper()
        self.time.sleep(6)

        self.myarm.level_effector(True)
        self.time.sleep(2)


        self.myarm.inverse_kinematics(-0.3, 1.2, True)  #shelf 2
        self.time.sleep(2)
        self.myarm.smooth_rotation(math.radians(-180))
        self.time.sleep(2)
        #self.myarm.inverse_kinematics(-0.25, 1.35)   #shelf 3
        #self.myarm.inverse_kinematics(-0.3, 0.9, True)   #shelf 1

        self.arm.open_gripper()
        self.time.sleep(20)
