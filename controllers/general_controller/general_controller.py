"""4_wheel_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor , DistanceSensor, Keyboard, GPS , InertialUnit , Camera
Time_step=64
# create the Robot instance.
robot = Robot()
kb= Keyboard()
gp= GPS('global')
iu= InertialUnit('imu')
lr = Motor('linear')
rm = Motor('RM')
cm = Camera('CAM')
#initialize Camera
#Camera.enable('CAM',Time_step)
cm=robot.getCamera('CAM')
cm.enable(Time_step)
cm.recognitionEnable(Time_step)
# initialize rotary motor
rm= robot.getMotor('RM')
rotate=0.0
# initialize motor
lr=robot.getMotor("linear")
linear=0.0
#Wheels initialization
wheels=[]
wheelNames=['wheel1','wheel2','wheel3','wheel4']
for name in wheelNames:
    wheels.append(robot.getDevice(name))
#for distance sensor`
ps=[]
psNames=['ds_left','ds_right']
for i in range(2):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(Time_step)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
#Enabling the keyboard
kb.enable(Time_step)
leftspeed=0.0
rightspeed=0.0
speed=0.0
# setting up the gps
gp=robot.getGPS('global')
gp.enable(Time_step)
for i in range(4):
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(speed)

#setting up the IMU
iu=robot.getInertialUnit('imu')
iu.enable(Time_step)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
#avoidObstacleCounter = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller

while robot.step(Time_step) != -1:
    key=kb.getKey()
    # Image processing
    #reading distance sensor values
    d_left=ps[0].getValue()
    d_right=ps[1].getValue()
    print('Distance Reading left sensor: ', d_left ,' Distance Reading right sensor: ', d_right )
    print(' ')
    
    # reading gps values
    gps_x=gp.getValues()[0]
    gps_y=gp.getValues()[1]
    gps_z=gp.getValues()[2]
    # reading imu values
    gps_angle_x=iu.getRollPitchYaw()[0]
    gps_angle_y=iu.getRollPitchYaw()[1]
    gps_angle_z=iu.getRollPitchYaw()[2]
    print('X: ', gps_x, 'Y: ', gps_y , 'Z: ', gps_z)
    print(' ')
    print('Angle X is: ', gps_angle_x, 'Angle y is: ', gps_angle_y, 'Angle Z is: ', gps_angle_z)
    print(' ')
    print("this is rotate position", rotate)
    print(' ')
    print("##############################-------------------##############################")
    if (key==315):
        leftspeed = 5.0
        rightspeed = 5.0
    elif (key==317):
        leftspeed = -3.0
        rightspeed = -3.0
    elif (key==316):
        leftspeed = 5.0
        rightspeed = -3.0
    elif (key==314):
        leftspeed = -3.0
        rightspeed = 5.0
    else:
        leftspeed = 0.0
        rightspeed = 0.0
    # for the linear arm
    if (key==87):
        if linear<0.18:
            linear=linear+0.002
        else:
            linear=linear-0.02
    elif (key==83):
        if linear<0.18:
            linear=linear-0.002
        else:
            linear=linear-0.02
    else:
        linear=linear+0.0
    #if avoidObstacleCounter > 0:
    #    avoidObstacleCounter -= 1
    #    leftspeed = -1.0
    #    rightspeed = 1.0
    #else:  # read sensors
    #    for i in range(2):
    #        if ps[i].getValue() < 950.0:
    #            avoidObstacleCounter =50
    wheels[0].setVelocity(leftspeed)
    wheels[1].setVelocity(rightspeed)
    wheels[2].setVelocity(leftspeed)
    wheels[3].setVelocity(rightspeed)
    # set the position of linear actuator
    lr.setPosition(linear)
    if (key==65) and (rotate<1.57):
        rotate=rotate+0.05
    elif (key==68) and (rotate>-1.57):
        rotate=rotate-0.05
    else:
        rotate=rotate+0.0
    
    if rotate > 1.57:
        rotate=rotate-0.05
    rm.setPosition(rotate)
# Enter here exit cleanup code.
