from pyCreate2 import create2
import math
import odometry
import pid_controller
import lab8_map
import particle_filter


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
        self.virtual_create = factory.create_virtual_create()
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.map = lab8_map.Map("lab8_map.json")
        self.pf = particle_filter.ParticleFilter(self.map, 1000, 0.01, 0.05, 0.1)

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
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
            math.cos(goal_theta - self.odometry.theta))) > 0.05:
            # print("Go TO: " + str(goal_theta) + " " + str(self.odometry.theta))
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)
        
    def go_to_goal(self, goal_x, goal_y):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2)) > 0.1:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            output_distance = self.pidDistance.update(0, distance, self.time.time())
            self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        print(goal_x, goal_y)
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

    def visualize(self):
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
            print(distance)
            self.pf.measure(distance, math.radians(angle))
           
            self.visualize()
    
            angle += 45
            
    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        self.sleep(0.5)
        self.visualize()

        self.go_to_goal(0.5, 0.5)
        distance = self.sonar.get_distance()
        self.pf.measure(distance, 0)
        self.visualize()
        self.go_to_goal(0.5, 0.75)
        distance = self.sonar.get_distance()
        self.pf.measure(distance, 0)
        self.visualize()
        self.go_to_goal(1.0, 2.0)
        distance = self.sonar.get_distance()
        self.pf.measure(distance, 0)
        self.visualize()
        self.take_measurements()