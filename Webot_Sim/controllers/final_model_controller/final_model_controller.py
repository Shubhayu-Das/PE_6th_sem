"""final_model_controller controller."""

from math import pi
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt

import constants
from controller import Robot, Motor, PositionSensor

signum = lambda x: -1 if x < 0 else 1

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The left knee joint is not being actuated currently
# leftKneeMotor = robot.getDevice('left_knee_motor')
# leftKneeMotor.enableTorqueFeedback(timestep)
# leftEncoder = robot.getDevice('left_knee_sensor')
# leftEncoder.enable(timestep)

# Commands for the right knee joint
rightKneeMotor = robot.getDevice('right_knee_motor')
rightKneeMotor.enableTorqueFeedback(timestep)
rightEncoder = robot.getDevice('right_knee_sensor')
rightEncoder.enable(timestep)

counter = -100
direction = -1
stepSize = constants.MAX_STEP_SIZE
tolerance = constants.MAX_TOLERANCE

lowerLimit = 0
upperLimit = -round(pi/2, constants.PRECISION)
setPoint = upperLimit

tracking = pd.DataFrame([], columns=[
    "right_knee_angular_position",
    "right_motor_torque",
    "right_motor_delta"
])

rightMotorPositionHistory = []
rightMotorDeltaHistory = []
rightMotorTorque = []
# leftMotorTorque = []

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if counter < 0:
        counter += 1
        continue

    rightPosition = round(rightEncoder.getValue(), constants.PRECISION)
    # leftPosition = round(leftEncoder.getValue(), constants.PRECISION)

    desiredPosition = rightPosition
    delta = round(abs(rightPosition - setPoint), constants.PRECISION)

    # If object is not within a tolerance of the set point, then decrease step size
    # This will be replaced with proper PID control later
    if delta > tolerance and delta < stepSize and stepSize > tolerance:
        stepSize /= 2

    # If object is too far from set point, move towards the set point
    if delta > stepSize:
        rightTorque = round(rightKneeMotor.getTorqueFeedback(), constants.PRECISION)
        tracking = tracking.append({
        "right_knee_angular_position": rightPosition,
        "right_motor_torque": rightTorque,
        "right_motor_delta": delta
        }, ignore_index=True)

        rightMotorPositionHistory.append(rightPosition)
        rightMotorDeltaHistory.append(delta)
        rightMotorTorque.append(rightTorque)

        desiredPosition = rightPosition + signum(setPoint - rightPosition) * stepSize
        desiredPosition = round(desiredPosition, constants.PRECISION)
    
    # Otherwise, if near set point(within tolerance), then reverse direction
    else:
        setPoint = lowerLimit + upperLimit - rightPosition
        stepSize = constants.MAX_STEP_SIZE

    # Ask the motor to make the correct move
    rightKneeMotor.setPosition(desiredPosition)

    if counter == 0 and constants.DEBUG:
        print(f"Encodzer positions:\n\tLeft knee: <disabled>{0}\tRight knee: {rightPosition}")
        print(f"Angular distance to set point: {delta}\tstep size: {stepSize}")

    counter = (counter + 1) % 8

# Enter here exit cleanup code.
tracking.to_csv(f"data/sim_results_{datetime.now().strftime('%d_%m_%Y_%H_%M')}.csv", index=False)
