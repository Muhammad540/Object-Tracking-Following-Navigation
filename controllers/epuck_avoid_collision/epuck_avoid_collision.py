"""epuck_avoid_collision controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor
time_step=64  #milli seconds
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())

# initialize device
ps=[]
psNames=['ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(time_step)
    
leftmotor=robot.getDevice('left wheel motor')
rightmotor=robot.getDevice('right wheel motor')
leftmotor.setPosition(float('inf'))
rightmotor.setPosition(float('inf'))

leftmotor.setVelocity(0.0)
rightmotor.setVelocity(0.0)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(time_step) != -1:
    # Read the sensors:
    psValues=[]
    for i in range(8):
        psValues.append(ps[i].getValue())
        
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    right_obstacle = psValues[0] > 79.0 or psValues[1] > 79.0 or psValues[2] > 79.0
    left_obstacle = psValues[5] > 79.0 or psValues[6] > 79.0 or psValues[7] > 79.0
    
    max_speed=6.28
    # initialize motor speed by 50% max
    leftspeed=0.5*max_speed
    rightspeed=0.5*max_speed
    if left_obstacle:
        leftspeed = 0.5 * max_speed
        rightspeed= -0.5 *max_speed
    elif right_obstacle:
        leftspeed = -0.5 * max_speed
        rightspeed= 0.5 *max_speed
    if left_obstacle and right_obstacle:
        leftspeed=0.5*max_speed
        rightspeed=0
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    leftmotor.setVelocity(leftspeed)
    rightmotor.setVelocity(rightspeed)

# Enter here exit cleanup code.
