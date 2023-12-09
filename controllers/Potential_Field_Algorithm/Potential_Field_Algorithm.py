from controller import Robot
from controller import Keyboard
import math

cruise_speed=5
turn_speed= 4
time_step= 64
wheel_radius=0.06
wheel_circum=2*3.14*wheel_radius
encoder_unit=wheel_circum/6.28
distance_between_wheels= 0.34
robot = Robot()

left_wheel= robot.getDevice('left wheel')
right_wheel= robot.getDevice('right wheel')
left_wheel.setPosition(float('Inf'))
right_wheel.setPosition(float('Inf'))
left_wheel.setVelocity(0.0)
right_wheel.setVelocity(0.0)

keyboard= Keyboard()
keyboard.enable(time_step)
#position sensor
left_ps=robot.getPositionSensor('left wheel sensor')
left_ps.enable(time_step)

right_ps=robot.getPositionSensor('right wheel sensor')
right_ps.enable(time_step)

ps_values=[0, 0]
dist_values=[0, 0]
robot_pose=[0,0,0] # x,y,theta
last_ps_value=[0,0]

motor_cmd= {
    ord('W'): ( cruise_speed ,cruise_speed),
    ord('S'): ( -cruise_speed, -cruise_speed),
    ord('A'): ( -turn_speed , turn_speed),
    ord('D'): ( turn_speed , -turn_speed),
    ord('P'): (0.0 , 0.0)
}   
#LIDAR
lidar=robot.getLidar("lidar")
lidar.enable(time_step)
lidar.enablePointCloud()

#gps
gps=robot.getDevice('gps')
gps.enable(time_step)

def command_motors(cmd):
    left_wheel.setVelocity(cmd[0])
    right_wheel.setVelocity(cmd[1])



def attractive_distance(agent_x,agent_y,goal_x,goal_y):
    return math.sqrt((agent_x - goal_x)**2 + (agent_y - goal_y)**2)

# input delta x and delta y for goal
goal_cord=[0.21,0.37]
#goal_cord=[0.0173889,-0.481134]
k_attractive=10  
d_goal_threshold=0.2
force=[0,0]
alpha=0.001
kp=6

#calculating the attractive potential and attractice force
def attractive_potential_gradient(k_attractive,d_goal_threshold,agent_x,agent_y,goal_x,goal_y):
    dist_to_goal = math.sqrt((agent_x-goal_x)**2+(agent_y-goal_y)**2)
    if dist_to_goal<=d_goal_threshold:
        force[0]=(k_attractive*(agent_x-goal_x))   
        force[1]=(k_attractive*(agent_y-goal_y))
    else:
        force[0]=d_goal_threshold*((k_attractive*(agent_x-goal_x))/dist_to_goal)
        force[1]=d_goal_threshold*((k_attractive*(agent_y-goal_y))/dist_to_goal)
    return force
#calcualting the repulsive potential and repulsive force
rforce=[0,0]
def repulsive_potential_gradient(agent_x,agent_y,minimum,angle):
    radius_of_influence=0.07
    k_rep=1
    obs_x = agent_x + minimum * math.cos(angle)
    obs_y = agent_y + minimum * math.sin(angle)
    obstacle_dist=math.sqrt((agent_x - obs_x)**2 + (agent_y - obs_y)**2)
    #del_ro_x=(agent_x-obs_x)/minimum
    #del_ro_y=(agent_y-obs_y)/minimum
    del_ro_x=(agent_x-obs_x)
    del_ro_y=(agent_y-obs_y)

    if minimum<=radius_of_influence:
        #rforce[0]=(-k_rep)*((1/minimum)-(1/radius_of_influence))*(del_ro_x/(minimum)**2)
        #rforce[1]=(-k_rep)*((1/minimum)-(1/radius_of_influence))*(del_ro_y/(minimum)**2)
        rforce[0]=(-k_rep)*(1-(obstacle_dist/radius_of_influence))*(del_ro_x/(minimum**3))
        rforce[1]=(-k_rep)*(1-(obstacle_dist/radius_of_influence))*(del_ro_y/(minimum**3))
    else:
        rforce[0]=0
        rforce[1]=0
    return rforce

def angle_to_goal(temp_goal_x,temp_goal_y,agent_x,agent_y):
    return math.atan2((temp_goal_y-agent_y),(temp_goal_x - agent_x))

left_wheel.setVelocity(cruise_speed)
right_wheel.setVelocity(cruise_speed)
total_force=[0,0]
x_new=0.21
y_new=0.37
dep=1
count=0
while robot.step(time_step) != -1:
    key = keyboard.getKey()
    if key == ord('='):
        turn_speed=turn_speed + 0.1
    elif key == ord('-'):
        turn_speed=turn_speed - 0.1
    #print('turn rate is', turn_speed)
    if key in motor_cmd.keys():
        command_motors(motor_cmd[key])

    #lidar computation
    range_image=lidar.getRangeImage()
    minimum=min(range_image)
    print("Lidar Reading : {}".format(minimum))
    
    #agent position
    gps_value=gps.getValues()
    agent_cord=[gps_value[0],gps_value[1]]
    agent_x=agent_cord[0]
    agent_y=agent_cord[1]
    if key==-1:
        key = keyboard.getKey()
    #calculate the distance to goal
        dist_to_goal=attractive_distance(agent_x,agent_y,goal_cord[0],goal_cord[1])
        print("this is the distance to goal", dist_to_goal)
        #distance = '%.10f'%(dist_to_goal)

    #calcualte the angle of robot
        angle=kp*(angle_to_goal(x_new,y_new,agent_x,agent_y))

    #update x and y position of robot
    #attractive force
        force=attractive_potential_gradient(k_attractive,d_goal_threshold,agent_x,agent_y,goal_cord[0],goal_cord[1])
        force[0]=force[0]*dep
        force[1]=force[0]*dep
        print('this is attractive force', force)
    #repulsive force
        rforce=repulsive_potential_gradient(agent_x,agent_y,minimum,angle)
        print('this is repulsive force', rforce)
        if rforce[0]>200 or rforce[1]>200:
            count=count+1
        if dist_to_goal <0.21:
            dep=1
            rforce[0]=0
            rforce[1]=0
    #total force
        total_force[0]=force[0]+rforce[0]
        total_force[1]=force[1]+rforce[1]
        print('this is total force', total_force)
    #updating x and y
        x_new=agent_x-(alpha*(total_force[0]))
        print('this is new x cord', x_new)
        y_new=agent_y-(alpha*(total_force[1]))
        print('this is new y cord', y_new)
    # ensuring that agent quickly converges when in vicinity of goal

    # calculate angle to goal
        print("Angle to goal", (angle_to_goal(x_new,y_new,agent_x,agent_y))*(180/math.pi))
        if rforce[0]==0 and rforce[1]==0:
            kt=0.25
            if dist_to_goal <0.21:
                dep=1
                kt=2
            print('this is attractive gain', kt)
            #speed and angle
            angle=kt*(angle_to_goal(goal_cord[0],goal_cord[1],agent_x,agent_y))
            #compute motion of wheel to reach p
            v_r=(cruise_speed+((distance_between_wheels)*angle))
            v_l=(cruise_speed-((distance_between_wheels)*angle))
            print("This is right wheel velocity ",v_r,"This is left wheel velocity ", v_l)
            left_wheel.setVelocity(v_l)
            right_wheel.setVelocity(v_r)
        else:
            angle=kp*(angle_to_goal(x_new,y_new,agent_x,agent_y))
            #compute motion of wheel to reach p
            v_r=(cruise_speed+((distance_between_wheels)*angle))
            v_l=(cruise_speed-((distance_between_wheels)*angle))
            print("This is right wheel velocity ",v_r,"This is left wheel velocity ", v_l)
            left_wheel.setVelocity(v_l)
            right_wheel.setVelocity(v_r)

        
        
        #checking if stuck in local minima
        

        #gps values
        print('X coordinate', gps_value[0])
        print('Y coordinate', gps_value[1])
        print('Z coordinate', gps_value[2])

        for i in range(2):
            last_ps_value[i] = ps_values[i]
        print("---------------------------------------------------------------------")
        #dist_to_p=[]
        if dist_to_goal < 0.1:
            print('Robot has Reached its Goal')
            left_wheel.setVelocity(0)
            right_wheel.setVelocity(0)
            print("Number of collisions/intervensions detected ", count)
            break

        elif key!=-1:
            break
    if dist_to_goal < 0.1:
        print('Robot has Reached its Goal')
        left_wheel.setVelocity(0)
        right_wheel.setVelocity(0)
        print("Number of collisions ", count)
        break
    
            