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

while robot.step(timestep) != -1:
    import numpy as np
    import cv2
    cap= get_image_from_camera()
    #kernel window for morphological operations
    kernel = np.ones((5,5),np.uint8)
    
    #resize the capture window to 640 x 480
    ret = cap.set(3,640)
    ret = cap.set(4,480)
    
    #upper and lower limits for the color yellow in HSV color space
    lower_red = np.array([50, 150, 0])
    upper_red = np.array([200, 230, 255])
    
    #begin capture
    while(True):
        ret, frame = cap.read()
    
        #Smooth the frame
        frame = cv2.GaussianBlur(frame,(11,11),0)
    
        #Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        #Mask to extract just the yellow pixels
        mask = cv2.inRange(hsv,lower_red,upper_red)
    
        #morphological opening
        mask = cv2.erode(mask,kernel,iterations=2)
        mask = cv2.dilate(mask,kernel,iterations=2)
    
        #morphological closing
        mask = cv2.dilate(mask,kernel,iterations=2)
        mask = cv2.erode(mask,kernel,iterations=2)
    
        #Detect contours from the mask
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    		cv2.CHAIN_APPROX_SIMPLE)[-2]
    
        if(len(cnts) > 0):
            #Contour with greatest area
            c = max(cnts,key=cv2.contourArea)
            #Radius and center pixel coordinate of the largest contour
            ((x,y),radius) = cv2.minEnclosingCircle(c)
    
            if radius > 5:
                #Draw an enclosing circle
                cv2.circle(frame,(int(x), int(y)), int(radius),(0, 255, 255), 2)
    
                #Draw a line from the center of the frame to the center of the contour
                cv2.line(frame,(320,240),(int(x), int(y)),(0, 0, 255), 1)
                #Reference line
                cv2.line(frame,(320,0),(320,480),(0,255,0),1)
    
                radius = int(radius)
    
                #distance of the 'x' coordinate from the center of the frame
                #wdith of frame is 640, hence 320
                length = 320-(int(x))
    
    
        #display the image
        cv2.imshow('frame',frame)
        #Mask image
        cv2.imshow('mask',mask)
        #Quit if user presses 'q'
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    
        #Release the capture
    cap.release()
    cv2.destroyAllWindows()