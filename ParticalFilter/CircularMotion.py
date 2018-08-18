# -----------------
# USER INSTRUCTIONS
#
# Write a function in the class robot called move()
#
# that takes self and a motion vector (this
# motion vector contains a steering* angle and a
# distance) as input and returns an instance of the class
# robot with the appropriate x, y, and orientation
# for the given motion.
#
# *steering is defined in the video
# which accompanies this problem.
#
# For now, please do NOT add noise to your move function.
#
# Please do not modify anything except where indicated
# below.
#
# There are test cases which you are free to use at the
# bottom. If you uncomment them for testing, make sure you
# re-comment them before you submit.

from math import *
import random

# --------
#
# some top level parameters
#

max_steering_angle = pi / 4.0 # You do not need to use this value, but keep in mind the limitations of a real car.
bearing_noise = 0.1 # Noise parameter: should be included in sense function.
steering_noise = 0.1 # Noise parameter: should be included in move function.
distance_noise = 5.0 # Noise parameter: should be included in move function.

tolerance_xy = 15.0 # Tolerance for localization in the x and y directions.
tolerance_orientation = 0.25 # Tolerance for orientation.


# --------
#
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!

landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks in (y, x) format.
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"

# ------------------------------------------------
#
# this is the robot class
#
class robot:

    # --------

    # init:
    #	creates robot and initializes location/orientation
    #

    def __init__(self, length=10.0):
        self.x = random.random() * world_size  # initial x position
        self.y = random.random() * world_size  # initial y position
        self.orientation = random.random() * 2.0 * pi  # initial orientation
        self.length = length  # length of robot
        self.bearing_noise = 0.0  # initialize bearing noise to zero
        self.steering_noise = 0.0  # initialize steering noise to zero
        self.distance_noise = 0.0  # initialize distance noise to zero

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))

    # --------
    # set:
    #	sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    # --------
    # set_noise:
    #	sets the noise parameters
    #

    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################

    # --------
    # move:
    #   move along a section of a circular path according to motion
    #
    def move(self, motion):  # Do not change the name of this function
        steering = motion[0]
        distance = motion[1]
        tolerance = 0.001

        if abs(steering) > max_steering_angle:
            raise ValueError, 'Exceeding max steering angle.'

        if distance < 0.0:
            raise ValueError, 'Moving backwards is not valid.'

        res = robot()
        res.length = self.length
        res.bearing_noise = self.bearing_noise
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise

        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        turn = tan(steering2) * distance2 / res.length

        if abs(turn) < tolerance:
            res.x = self.x + distance2 * cos(self.orientation)
            res.y = self.y + distance2 * sin(self.orientation)
            res.orientation = (self.orientation + turn) % (2 * pi)

        else:
            radius = distance2 / turn
            cx = self.x - sin(self.orientation) * radius
            cy = self.y + cos(self.orientation) * radius
            res.orientation = (self.orientation + turn) % (2 * pi)
            res.x = cx + sin(res.orientation) * radius
            res.y = cy - cos(res.orientation) * radius


        return res  # make sure your move function returns an instance
        # of the robot class with the correct coordinates.

    # --------
    # sense:
    #   obtains bearings from positions
    #
    def sense(self, is_noise):  # do not change the name of this function
        Z = []

        # ENTER CODE HERE
        # HINT: You will probably need to use the function atan2()
        for i in range(len(landmarks)):
            dx = landmarks[i][1] - self.x
            dy = landmarks[i][0] - self.y
            bearing = atan2(dy, dx) - self.orientation
            if is_noise:
                bearing = random.gauss(bearing, self.bearing_noise)

            Z.append(bearing% (2 * pi))

        return Z  # Leave this line here. Return vector Z of 4 bearings

    # -----
    # measurement
    # calculates how likely a measurement should be
    def measurement_prob(self, measurement):
        pred_measurements = self.sense(False)

        prob = 1.0;
        for i in range(len(measurement)):
            prob *= self.Gaussian(pred_measurements[i], self.bearing_noise, measurement[i])

        return prob


    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        # Make sure that the range of angle is [0, 2PI]
        delta = (abs(mu - x) + pi)  % (2.0 * pi) - pi
        return exp(- ((delta) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    ############## ONLY ADD/MODIFY CODE ABOVE HERE ####################

####################################################################
#   TEST CASE: FINAL PF
####################################################################
# --------
#
# extract position from a particle set
#
def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi))
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]


# --------
#
# The following code generates the measurements vector
# You can use it to develop your solution.
#
def generate_ground_truth(motions):
    myrobot = robot()
    myrobot.set_noise(bearing_noise, steering_noise, distance_noise)

    Z = []
    T = len(motions)

    for t in range(T):
        myrobot = myrobot.move(motions[t])
        Z.append(myrobot.sense())
    # print 'Robot:    ', myrobot
    return [myrobot, Z]


# --------
#
# The following code prints the measurements associated
# with generate_ground_truth
#

def print_measurements(Z):
    T = len(Z)

    print 'measurements = [[%.8s, %.8s, %.8s, %.8s],' % \
          (str(Z[0][0]), str(Z[0][1]), str(Z[0][2]), str(Z[0][3]))
    for t in range(1, T - 1):
        print '                [%.8s, %.8s, %.8s, %.8s],' % \
              (str(Z[t][0]), str(Z[t][1]), str(Z[t][2]), str(Z[t][3]))
    print '                [%.8s, %.8s, %.8s, %.8s]]' % \
          (str(Z[T - 1][0]), str(Z[T - 1][1]), str(Z[T - 1][2]), str(Z[T - 1][3]))


# --------
#
# The following code checks to see if your particle filter
# localizes the robot to within the desired tolerances
# of the true position. The tolerances are defined at the top.
#
def check_output(final_robot, estimated_position):
    error_x = abs(final_robot.x - estimated_position[0])
    error_y = abs(final_robot.y - estimated_position[1])
    error_orientation = abs(final_robot.orientation - estimated_position[2])
    error_orientation = (error_orientation + pi) % (2.0 * pi) - pi
    correct = error_x < tolerance_xy and error_y < tolerance_xy and error_orientation < tolerance_orientation
    return correct


def particle_filter(motions, measurements, N=500):  # I know it's tempting, but don't change N!
    # --------
    #
    # Make particles
    #
    p = []
    for i in range(N):
        r = robot()
        r.set_noise(bearing_noise, steering_noise, distance_noise)
        p.append(r)

    # --------
    #
    # Update particles
    #
    for t in range(len(motions)):

        # motion update (prediction)
        p2 = []
        for i in range(N):
            p2.append(p[i].move(motions[t]))
        p = p2

        # measurement update
        w = []
        for i in range(N):
            w.append(p[i].measurement_prob(measurements[t]))

        # resampling
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(p[index])
        p = p3

    return get_position(p)

## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out.
##
## You can test whether your particle filter works using the
## function check_output (see test case 2). We will be using a similar
## function. Note: Even for a well-implemented particle filter this
## function occasionally returns False. This is because a particle
## filter is a randomized algorithm. We will be testing your code
## multiple times. Make sure check_output returns True at least 80%
## of the time.


## --------
## TEST CASES:
##
##1) Calling the particle_filter function with the following
##    motions and measurements should return a [x,y,orientation]
##    vector near [x=93.476 y=75.186 orient=5.2664], that is, the
##    robot's true location.
##
motions = [[2. * pi / 10, 20.] for row in range(8)]
measurements = [[4.746936, 3.859782, 3.045217, 2.045506],
               [3.510067, 2.916300, 2.146394, 1.598332],
               [2.972469, 2.407489, 1.588474, 1.611094],
               [1.906178, 1.193329, 0.619356, 0.807930],
               [1.352825, 0.662233, 0.144927, 0.799090],
               [0.856150, 0.214590, 5.651497, 1.062401],
               [0.194460, 5.660382, 4.761072, 2.471682],
               [5.717342, 4.736780, 3.909599, 2.342536]]

print particle_filter(motions, measurements)

## 2) You can generate your own test cases by generating
##    measurements using the generate_ground_truth function.
##    It will print the robot's last location when calling it.
##
##
##number_of_iterations = 6
##motions = [[2. * pi / 20, 12.] for row in range(number_of_iterations)]
##
##x = generate_ground_truth(motions)
##final_robot = x[0]
##measurements = x[1]
##estimated_position = particle_filter(motions, measurements)
##print_measurements(measurements)
##print 'Ground truth:    ', final_robot
##print 'Particle filter: ', estimated_position
##print 'Code check:      ', check_output(final_robot, estimated_



####################################################################
#   TEST CASE: MOVE
####################################################################
## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.

## --------
## TEST CASE:
##
## 1) The following code should print:
##       Robot:     [x=0.0 y=0.0 orient=0.0]
##       Robot:     [x=10.0 y=0.0 orient=0.0]
##       Robot:     [x=19.861 y=1.4333 orient=0.2886]
##       Robot:     [x=39.034 y=7.1270 orient=0.2886]
##
##
# length = 20.
# bearing_noise  = 0.0
# steering_noise = 0.0
# distance_noise = 0.0
#
# myrobot = robot(length)
# myrobot.set(0.0, 0.0, 0.0)
# myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
#
# motions = [[0.0, 10.0], [pi / 6.0, 10], [0.0, 20.0]]
#
# T = len(motions)
#
# print 'Robot:    ', myrobot
# for t in range(T):
#    myrobot = myrobot.move(motions[t])
#    print 'Robot:    ', myrobot

##

## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.


## 2) The following code should print:
##      Robot:     [x=0.0 y=0.0 orient=0.0]
##      Robot:     [x=9.9828 y=0.5063 orient=0.1013]
##      Robot:     [x=19.863 y=2.0201 orient=0.2027]
##      Robot:     [x=29.539 y=4.5259 orient=0.3040]
##      Robot:     [x=38.913 y=7.9979 orient=0.4054]
##      Robot:     [x=47.887 y=12.400 orient=0.5067]
##      Robot:     [x=56.369 y=17.688 orient=0.6081]
##      Robot:     [x=64.273 y=23.807 orient=0.7094]
##      Robot:     [x=71.517 y=30.695 orient=0.8108]
##      Robot:     [x=78.027 y=38.280 orient=0.9121]
##      Robot:     [x=83.736 y=46.485 orient=1.0135]
##
##
#length = 20.
#bearing_noise  = 0.0
#steering_noise = 0.0
#distance_noise = 0.0
#
#myrobot = robot(length)
#myrobot.set(0.0, 0.0, 0.0)
#myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
#
#motions = [[0.2, 10.] for row in range(10)]
#
#T = len(motions)
#
#print 'Robot:    ', myrobot
#for t in range(T):
#    myrobot = myrobot.move(motions[t])
#    print 'Robot:    ', myrobot

## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.



####################################################################
#   TEST CASE: SENSE
####################################################################
## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## sense function with randomized initial robot coordinates.

## --------
## TEST CASES:


##
## 1) The following code should print the list [6.004885648174475, 3.7295952571373605, 1.9295669970654687, 0.8519663271732721]
##
##
# length = 20.
# bearing_noise  = 0.0
# steering_noise = 0.0
# distance_noise = 0.0
#
# myrobot = robot(length)
# myrobot.set(30.0, 20.0, 0.0)
# myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
#
# print 'Robot:        ', myrobot
# print 'Measurements: ', myrobot.sense()
##

## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## sense function with randomized initial robot coordinates.


##
## 2) The following code should print the list [5.376567117456516, 3.101276726419402, 1.3012484663475101, 0.22364779645531352]
##
##
##length = 20.
##bearing_noise  = 0.0
##steering_noise = 0.0
##distance_noise = 0.0
##
##myrobot = robot(length)
##myrobot.set(30.0, 20.0, pi / 5.0)
##myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
##
##print 'Robot:        ', myrobot
##print 'Measurements: ', myrobot.sense()
##


## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## sense function with randomized initial robot coordinates.
