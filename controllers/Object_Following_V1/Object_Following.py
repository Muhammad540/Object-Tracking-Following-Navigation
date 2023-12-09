from controller import Robot, Motor , DistanceSensor, Keyboard, GPS , InertialUnit , Camera ,Node
import base64
import os
import sys
import tempfile
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
width = cm.getWidth()
height = cm.getHeight()
cm.recognitionEnable(Time_step)
cm.enableRecognitionSegmentation()
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
import cv2
import numpy as np
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")
try:
    import cv2
except ImportError:
    sys.exit("Warning: 'cv2' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")


def cleanup():
    """Remove device image files."""
    # Ignore errors if file doesn't exist.
    try:
        os.remove(deviceImagePath + '/display.jpg')
    except OSError:
        pass
    try:
        os.remove(deviceImagePath + '/camera.jpg')
    except OSError:
        pass
def sendDeviceImage(robot, device):
    """Send the rendering device image to the robot window."""
    if device.getNodeType() == Node.DISPLAY:
        deviceName = 'display'
        fileName = deviceName + '.jpg'
        device.imageSave(None, deviceImagePath + '/' + fileName)
    elif device.getNodeType() == Node.CAMERA:
        deviceName = 'CAM'
        fileName = deviceName + '.jpg'
        device.saveImage(deviceImagePath + '/' + fileName, 80)
    else:
        return
    with open(deviceImagePath + '/' + fileName, 'rb') as f:
        fileString = f.read()
        fileString64 = base64.b64encode(fileString).decode()
        robot.wwiSendText("image[" + deviceName + "]:data:image/jpeg;base64," + fileString64)
deviceImagePath = os.getcwd()
try:
    imageFile = open(deviceImagePath + "/image.jpg", 'w')
    imageFile.close()
except IOError:
    deviceImagePath = tempfile.gettempdir()
    
display = robot.getDevice('display')
# Show camera image in the display background.
display.attachCamera(camera)
display.setColor(0xFF0000)

# Variables needed to draw the target on the display.
targetPoint = []
targetRadius = 0    
    
    
while robot.step(timestep) != -1:
    rawString = cm.getImage()

    # Create mask for yellow pixels based on the camera image.
    index = 0
    maskRGB = np.zeros([height, width], np.uint8)
    for j in range(0, height):
        for i in range(0, width):
            # Camera image pixel format
            if sys.version_info.major > 2:  # Python 3 code
                b = rawString[index]
                g = rawString[index + 1]
                r = rawString[index + 2]
            else:  # Python 2.7 code
                b = ord(rawString[index])
                g = ord(rawString[index + 1])
                r = ord(rawString[index + 2])
            index += 4
            # Yellow color threshold.
            if b < 50 and g > 180 and r > 180:
                maskRGB[j][i] = True

    # Find blobs contours in the mask.
    contours = cv2.findContours(maskRGB.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Only proceed if at least one blob is found.
    if not contours:
        continue

    # Choose the largest blob.
    blob = max(contours, key=cv2.contourArea)

    # Compute the minimum enclosing circle and centroid of the blob.
    ((x, y), radius) = cv2.minEnclosingCircle(blob)
    targetPoint = [int(x), int(y)]
    targetRadius = int(radius)

    # Show detected blob in the display: draw the circle and centroid.
    display.setAlpha(1.0)
    if targetRadius > 0:
        display.setColor(0x00FFFF)
        display.drawOval(targetPoint[0], targetPoint[1], targetRadius, targetRadius)
    display.setColor(0xFF0000)
    display.fillOval(int(targetPoint[0]), int(targetPoint[1]), 5, 5)
    # Send the display image to the robot window.
    sendDeviceImage(robot, display)