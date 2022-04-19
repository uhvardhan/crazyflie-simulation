import numpy as np

from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro

from utils import external_dynamics

# Constant for Mass and Gravity
MASS = 0.027
GRAVITY = 9.81

# Create the ROBOT instance
robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())

# Initialize Motors
m1_motor = robot.getDevice("m1_motor")
m1_motor.setPosition(float('+inf'))
m1_motor.setVelocity(-1)
m2_motor = robot.getDevice("m2_motor")
m2_motor.setPosition(float('+inf'))
m2_motor.setVelocity(1)
m3_motor = robot.getDevice("m3_motor")
m3_motor.setPosition(float('+inf'))
m3_motor.setVelocity(-1)
m4_motor = robot.getDevice("m4_motor")
m4_motor.setPosition(float('+inf'))
m4_motor.setVelocity(1)

## Initialize Sensors
imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)
gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)

# Initialize variables
actPos = np.zeros(shape=(3,))
actVel = np.zeros(shape=(3,))
actAcc = np.zeros(shape=(3,))
actJerk = np.zeros(shape=(3,))
pastPosGlobal = np.zeros(shape=(3,))
pastVelGlobal = np.zeros(shape=(3,))
pastAccGlobal = np.zeros(shape=(3,))
pastPhiDotGlobal = np.zeros(shape=(3,))
pastThetaDotGlobal = np.zeros(shape=(3,))
pastPsiDotGlobal = np.zeros(shape=(3,))
past_time = robot.getTime()

# Initialize Controller Gains
K = np.array([10, 20, 35, 40])

# Desired Values
desPos = np.array([0.0, 0.0, 1.0])
desVel = np.array([0.0, 0.0, 0.0])
desAcc = np.array([0.0, 0.0, 0.0])
desJerk = np.array([0.0, 0.0, 0.0])

while robot.step(TIME_STEP) != -1:

    dt = robot.getTime() - past_time;

    # Get measurements
    posGlobal = np.array(gps.getValues())
    velGlobal = (posGlobal - pastPosGlobal)/dt
    accGlobal = (velGlobal - pastVelGlobal)/dt
    jerkGlobal = (accGlobal - pastAccGlobal)/dt
    
    matrixV, nextPos, nextVel, nextAcc, nextJerk = external_dynamics(K, desPos, desVel, desAcc, desJerk, posGlobal, velGlobal, accGlobal, jerkGlobal, dt)
    
    phiGlobal = imu.getRollPitchYaw()[0]
    thetaGlobal = imu.getRollPitchYaw()[1]
    psiGlobal = imu.getRollPitchYaw()[2]
    
    phiDotGlobal = gyro.getValues()[0]
    thetaDotGlobal = gyro.getValues()[1]
    psiDotGlobal = gyro.getValues()[2]
    
    phiDdotGlobal = (phiDotGlobal - pastPhiDotGlobal)/dt
    thetaDdotGlobal = (thetaDotGlobal - pastThetaDotGlobal)/dt
    psiDdotGlobal = (psiDotGlobal - pastPsiDotGlobal)/dt
    
    matrixV = matrixV.reshape(-1, 1)
    
    
    
    
    
