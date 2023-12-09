"""4_wheel_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor , DistanceSensor, Keyboard, GPS , InertialUnit , Camera
import cv2
import numpy as np
Time_step=200
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
cm.enableRecognitionSegmentation()
# initialize rotary motor
rm= robot.getMotor('RM')
rotate=1.57
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


def get_image_from_camera():
    """
    Take an image from the camera device and prepare it for OpenCV processing:
    - convert data type,
    - convert to RGB format (from BGRA), and
    - rotate & flip to match the actual image.
    """
    img = cm.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return cv2.flip(img, 1)

obstacle=False
detection=False
while robot.step(Time_step) != -1:
    key=kb.getKey()
    # Image processing
    #img = cm.getRecognitionSegmentationImage()
    imge= cm.getImage()
    # get the red component of the pixel (5,10)
    red=cm.imageGetRed(imge, cm.getWidth(),320,320)
    green=cm.imageGetGreen(imge, cm.getWidth(),320,320)
    blue=cm.imageGetBlue(imge, cm.getWidth(),320,320)
    #
    img = get_image_from_camera()
    # Segment the image by color in HSV color space
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img, np.array([50, 150, 0]), np.array([200, 230, 255]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    largest_contour = max(contours, key=cv2.contourArea, default=1)
    largest_contour_center = cv2.moments(largest_contour)
    if largest_contour_center['m00']!=0:
        center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])
        center_y = int(largest_contour_center['m01'] / largest_contour_center['m00'])
    print("    Readings from Image processing of Superior Robot    ")
    print(' ')
    print("This is the centre x position",center_x)
    print("This is the centre y position",center_y)
    print('Image Frame width', cm.getWidth())
    print('Image Frame height' , cm.getHeight())
    # Find error (ball distance from image center)
    error_x = cm.getWidth() / 2 - center_x
    print("We need to minmize X error, Following is the X error is: ",error_x)
    print(' ')
    
    #Implementing the robot motion
    incremental_speed=2
    decremental_speed=-2
    zero=0

    
    if error_x < 60 and error_x >-60 and obstacle==False:
        detection=True
        wheels[0].setVelocity(incremental_speed)
        wheels[1].setVelocity(incremental_speed)
        wheels[2].setVelocity(incremental_speed)
        wheels[3].setVelocity(incremental_speed)
    if error_x <-60 and obstacle==False:
        detection=True
        wheels[0].setVelocity(incremental_speed)
        wheels[1].setVelocity(decremental_speed)
        wheels[2].setVelocity(incremental_speed)
        wheels[3].setVelocity(decremental_speed)
    if error_x >60 and obstacle==False :
        detection=True
        wheels[0].setVelocity(decremental_speed)
        wheels[1].setVelocity(incremental_speed)
        wheels[2].setVelocity(decremental_speed)
        wheels[3].setVelocity(incremental_speed)


    #reading distance sensor values
    d_left=ps[0].getValue()
    d_right=ps[1].getValue()
    d_speed=-1
    i_speed=1
    #print('Distance Reading left sensor: ', d_left ,' Distance Reading right sensor: ', d_right )
    print("    Sensor Readings from Superior Robot    ")
    print('Reading from the left sensor', d_left)
    print('Reading from the right sensor', d_right)
    print(' ')
    if d_left<995 and d_right>995 and detection==False:
        obstacle=True
        wheels[0].setVelocity(i_speed)
        wheels[1].setVelocity(d_speed)
        wheels[2].setVelocity(i_speed)
        wheels[3].setVelocity(d_speed)
    if d_right<995 and d_left>995 and detection==False:
        obstacle=True
        wheels[0].setVelocity(d_speed)
        wheels[1].setVelocity(i_speed)
        wheels[2].setVelocity(d_speed)
        wheels[3].setVelocity(i_speed)


    # reading gps values
    gps_x=gp.getValues()[0]
    gps_y=gp.getValues()[1]
    gps_z=gp.getValues()[2]
    # reading imu values
    gps_angle_x=iu.getRollPitchYaw()[0]
    gps_angle_y=iu.getRollPitchYaw()[1]
    gps_angle_z=iu.getRollPitchYaw()[2]
    print("    GPS position readings of superior Robot    " )
    print('X: ', gps_x, 'Y: ', gps_y , 'Z: ', gps_z)
    print(' ')
    print("    GPS angular readings of superior Robot    " )
    print('Angle X is: ', gps_angle_x, 'Angle y is: ', gps_angle_y, 'Angle Z is: ', gps_angle_z)
    print(' ')
    print("This is rotational position of the camera: ", rotate)
    print(' ')
    print("#####################################################################################")
    
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
