from pyCreate2 import create2
import math
import odometry
%import pd_controller
import pid_controller
from enum import Enum

class Mode(Enum):
    GoToGoal   = 1
    WallFollow = 2



class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.odometry = odometry.Odometry()
        # self.pidTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        # self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.pdWF = pid_controller.PIDController(200, 50, 0, [0,0], [-50, 50])

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
                print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
            math.cos(goal_theta - self.odometry.theta))) > 0.1:
            print("Go TO: " + str(goal_theta) + " " + str(self.odometry.theta))
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)



    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        waypoints = [
            [2.0, 0.0],
            [3.0, 2.0],
            [2.5, 2.0],
            [0.0, 1.5],
            [0.0, 0.0]
        ]


        # goal_x = 0.5
        # goal_y = -0.5
        base_speed = 150
        start_time = self.time.time()
        goal_distance = 0.5

        mode = Mode.GoToGoal

        # with open('output.csv', 'w') as f:
        for waypoint in waypoints:
            while True:
                self.sleep(0)
                distance = self.sonar.get_distance()
                print(mode)

                if mode == Mode.GoToGoal:
                    if distance < goal_distance:
                        # switch to Wall following
                        mode = Mode.WallFollow
                        self.servo.go_to(75)
                        self.go_to_angle(self.odometry.theta-math.pi/2)
                        last_check = self.time.time()
                    else:
                        # Go to goal
                        goal_theta = math.atan2(waypoint[1] - self.odometry.y, waypoint[0] - self.odometry.x)
                        theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                        output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                        # Go with constant velocity
                        self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))

                        # Switch to next goal if close enough to goal
                        distance = math.sqrt(math.pow(waypoint[0] - self.odometry.x, 2) + math.pow(waypoint[1] - self.odometry.y, 2))
                        if distance < 0.1:
                            break

                elif mode == Mode.WallFollow:
                    if self.time.time() > last_check + 5.0:
                        # check if path to goal is free
                        # self.create.drive_direct(0, 0)
                        self.servo.go_to(-55)
                        theta = self.odometry.theta
                        goal_theta = math.atan2(waypoint[1] - self.odometry.y, waypoint[0] - self.odometry.x)
                        self.go_to_angle(goal_theta)

                        is_visible = True
                        for offset in [-55,0,55]:
                            self.servo.go_to(offset)
                            self.sleep(2.0)
                            distance = self.sonar.get_distance()
                            if distance < 0.75:
                                is_visible = False
                                break
                        if is_visible:
                            mode = Mode.GoToGoal
                            self.servo.go_to(0)
                            self.sleep(2.0)
                        else:
                            self.servo.go_to(75)
                            self.go_to_angle(theta)
                            self.sleep(2.0)
                        last_check = self.time.time()
                    else:
                        # wall follow
                        output = self.pdWF.update(distance, goal_distance, self.time.time())
                        self.create.drive_direct(int(base_speed - output), int(base_speed + output))