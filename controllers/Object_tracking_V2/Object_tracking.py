while robot.step(Time_step) != -1:
    key=kb.getKey()
    # Image processing
    #img = cm.getRecognitionSegmentationImage()
    imge= cm.getImage()
    # get the red component of the pixel (5,10)
    red=cm.imageGetRed(imge, cm.getWidth(),320,320)
    green=cm.imageGetGreen(imge, cm.getWidth(),320,320)
    blue=cm.imageGetBlue(imge, cm.getWidth(),320,320)
    print('Red :',red,'Green :',green,'Blue :',blue)
    #
    img = get_image_from_camera()
    # Segment the image by color in HSV color space
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img, np.array([50, 150, 0]), np.array([200, 230, 255]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    largest_contour = max(contours, key=cv2.contourArea, default=1)
    largest_contour_center = cv2.moments(largest_contour)
    center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])
    center_y = int(largest_contour_center['m01'] / largest_contour_center['m00'])
    print("This is the centre x position",center_x)
    print("This is the centre y position",center_y)
    print('width', cm.getWidth())
    print('height' , cm.getHeight())
    # Find error (ball distance from image center)
    error_x = cm.getWidth() / 2 - center_x
    print("This is the X error",np.abs(error_x))
    
    #Implementing the robot motion
    incremental_speed=2
    decremental_speed=-2
    zero=0
 
    if error_x < 50 and error_x >-50 and center_x!=0:
        wheels[0].setVelocity(incremental_speed)
        wheels[1].setVelocity(incremental_speed)
        wheels[2].setVelocity(incremental_speed)
        wheels[3].setVelocity(incremental_speed)
    if error_x <-50 and center_x!=0:
        wheels[0].setVelocity(incremental_speed)
        wheels[1].setVelocity(decremental_speed)
        wheels[2].setVelocity(incremental_speed)
        wheels[3].setVelocity(decremental_speed)
    if error_x >50 and center_x!=0:
        wheels[0].setVelocity(decremental_speed)
        wheels[1].setVelocity(incremental_speed)
        wheels[2].setVelocity(decremental_speed)
        wheels[3].setVelocity(incremental_speed)



    
    
    #reading distance sensor values
    d_left=ps[0].getValue()
    d_right=ps[1].getValue()
    #print('Distance Reading left sensor: ', d_left ,' Distance Reading right sensor: ', d_right )
    print('Reading from the left sensor', d_left)
    print('Reading from the right sensor', d_right)
    if d_left<995 and d_right>995:
    
    # reading gps values
    gps_x=gp.getValues()[0]
    gps_y=gp.getValues()[1]
    gps_z=gp.getValues()[2]
    # reading imu values
    gps_angle_x=iu.getRollPitchYaw()[0]
    gps_angle_y=iu.getRollPitchYaw()[1]
    gps_angle_z=iu.getRollPitchYaw()[2]
    #print('X: ', gps_x, 'Y: ', gps_y , 'Z: ', gps_z)
    #print(' ')
    #print('Angle X is: ', gps_angle_x, 'Angle y is: ', gps_angle_y, 'Angle Z is: ', gps_angle_z)
    #print(' ')
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