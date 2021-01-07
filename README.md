# Robotics_FinalProj
Final Project for CSCI 445 at USC.

In this program, a Create 2 robot carries a cup through a maze without colliding with its walls to reach a robotic arm.
The robotic arm picks up the cup with its gripper end effector and places the cup onto the shelf.

### PROJECT GOALS
Create2 robot:
1. Plan a path to the goal in a maze
1. Follow the path to get to the goal
1. Localize itself in the maze while moving in the maze
1. Arrive in front of the robotic arm

Robotic arm:
1. Reach to pick up the cup set on top of the Create2 robot with inverse kinematics
1. Land cup on desired shelf level, leveling the effector throughout the trajectory to ensure the cup does not fall

### SETUP
Main components of the simulation include: 
* Stationary components: 
  1. closed maze with stationary obstacles (ex: walls)
  1. Shelf with multiple levels
* Manipulatable robots:
  1. Create2 robot (equipped with sonar sensor and odometry sensors)
  1. Manipulator arm (equipped with 2 sagittal revolute joints, 1 horizontal revolute joint, and a gripper end effector)
* Transfer target object:
  1. Cup

Positioning
* Create2 is placed within the bounds of the maze.
* The transfer object (cup) is placed on top of the Create2.
* Manipulator arm is placed on the outside of the maze, at a distance where its end effector can still reach within the boundaries of the maze.
* Self is placed on the other side of the manipulator arm from the maze, where the end effector of the arm can still reach several of the shelf levels.

### PROCEDURES
#### Create2
##### 1 - Path planning : RRT
    A rapidly exploring randomized tree algorithm is used to find a clear path from given start and finish points in the maze.
    Parameters: number of particles (K) = 650, delta = 20.
##### 2 - Path following : PID controller
    A PID controller guides the Create2 robot smoothly through the found path.
    Parameters: P=300, I=5, D=50; set speed = 100
##### 3 - Localization : particle filtering
    A particle filter helps Create2 localize itself while moving throuh the path to gain better accuracy than its 
    odometry estimate. 
    This particle filter is based on its estimated distance to walls sonar readings, and probability.
##### 4 - Reach precise goal location : PID controller, particle filtering, extensive sonar measurements
    A more precise path following procedure for the last stride from the end of the RRT, following a laid out path with 
    smaller intervals that allow the robot to tune its following accuracy through reaching the goal. Extensive sonar 
    sensing (a 180 scan of the robot's surroundings) was used.

#### Manipulator Arm and Gripper
##### 1 - Cup pick-up : inverse kinematics
    2D Inverse kinematics formulae are used to calculate joint angles to reach (x/y,z) positions along the sagittal plane
    of the arm. With location passed from the create2 to the arm, the arm reaches down to grip the cup.
##### 2 - Level effector : inverse kinematics
    To prevent the cup from dropping, the effector has to stay leveled at all times. A separate smooth movement procedure
    was implemented to include effector leveling at each movement increment. Once the desired height is reached, a 3rd
    horizontal revolute joint smoothly spins the effector and the cup 180 degrees on the horizontal plane to place it on
    the shelf level behind the arm.

