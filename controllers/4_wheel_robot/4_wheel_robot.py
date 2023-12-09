"""4_wheel_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor , DistanceSensor
Time_step=64
# create the Robot instance.
robot = Robot()
wheels=[]
wheelNames=['wheel1','wheel2','wheel3','wheel4']
for name in wheelNames:
    wheels.append(robot.getDevice(name))
#for distance sensor`
ps=[]
psNames=['left_ps','right_ps']
for i in range(2):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(Time_step)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


speed=0.0
for i in range(4):
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(speed)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
avoidObstacleCounter = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(Time_step) != -1:
    left=ps[0].getValue()
    right=ps[1].getValue()
    #print('SECOND ROBOT Distance Reading left sensor: ', left ,' SECOND ROBOT Distance Reading right sensor: ', right )
    print(' ')
    leftspeed = 3.0
    rightspeed = 3.0
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftspeed = -3.0
        rightspeed = 3.0
    else:  # read sensors
        for i in range(2):
            if ps[i].getValue() < 950.0:
                avoidObstacleCounter =50
    wheels[0].setVelocity(leftspeed)
    wheels[1].setVelocity(rightspeed)
    wheels[2].setVelocity(leftspeed)
    wheels[3].setVelocity(rightspeed)

# Enter here exit cleanup code.
