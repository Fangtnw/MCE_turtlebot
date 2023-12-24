# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
from dis import dis
from socket import TIPC_SUBSCR_TIMEOUT
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from std_msgs.msg import String
import math
import random
turncase = ''
startdegree = 0.0
b4dif = 0.0
bount = 0
step = 1 
deltaturn = 0.0
totalturn = 0.0
previousturn = 0.0
previousx = 0.0
previousy = 0.0
totaldistance = 0.0
deltadistance = 0.0
goaldistance = 0.0
count = 0
direction = 0
turncal = 0.0
gocal = 0.0

linear = 0.0
angular = 0.0
deltaturn = 0.0
totalturn = 0.0
previousturn = 0.0
previousx = 0.0
previousy = 0.0
totaldistance = 0.0
deltadistance = 0.0
goaldistance = 0.0
count = 0
direction = 0
forcal = 0.0
forwalk = 0.0
remaindistance = 0.0

Sfront = 0
Sback = 0
Sleft = 0
Sright = 0
Senfront = []
Senback = []
Senleft = []
Senright = []
Sfront1 = 0
Sback1 = 0
Sleft1 = 0
Sright1 = 0

start = 1
state = 1
totaldegree = 0
predegree = 0
nowdegree = 0
count = 0

wandstate = 1
target_angle = None
Orien_help_turn = 0
all_wall= [0,0,0,0]
Nowposi = "0"
check = "0000"
check_1 = 0
check_2 = 0
check_3 = 0
condition = 5
firstcheck = 1
startposi = 1


class Turtlebot3Controller(Node):

    def __init__(self):
        super().__init__('turtlebot3_controller')   #node name
        self.cmdVelPublisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scanSubscriber = self.create_subscription(LaserScan, 'scan', self.scanCallback, qos_profile=qos_profile_sensor_data)
        self.batteryStateSubscriber = self.create_subscription(BatteryState, 'battery_state', self.batteryStateCallback, 1)
        self.odomSubscriber = self.create_subscription(Odometry, 'odom', self.odomCallback, 1)
        self.valueLaserRaw = {
            'range_min':0.0,
            'range_max':0.0,
            'ranges':[0.0]*360,
        }
        self.valueBatteryState = None
        self.valueOdometry = {
            'position':None,        #Datatype: geometry_msg/Point   (x,y,z)
            'orientation':None,     #Datatype: geometry_msg/Quaternion (x,y,z,w)
            'linearVelocity':None,  #Datatype: geometry_msg/Vector3 (x,y,z)
            'angularVelocity':None, #Datatype: geometry_msg/Vector3 (x,y,z)
        }
        
        self.state = 0
        #Use this timer for the job that should be looping until interrupted
        self.timer = self.create_timer(0.1,self.timerCallback)

    def publishVelocityCommand(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity
        self.cmdVelPublisher.publish(msg)
        #self.get_logger().info('Publishing cmd_vel: "%s", "%s"' % linearVelocity, angularVelocity)

    def scanCallback(self, msg):
        self.valueLaserRaw = {
            'range_min':msg.range_min,
            'range_max':msg.range_max,
            'ranges':list(msg.ranges),
        }
    
    def batteryStateCallback(self, msg):
        self.valueBatteryState = msg

    def odomCallback(self, msg):
        self.valueOdometry = {
            'position':msg.pose.pose.position,
            'positionX':msg.pose.pose.position.x,
            'positionY':msg.pose.pose.position.y,
            'positionZ':msg.pose.pose.position.z,
            'orientation':msg.pose.pose.orientation,
            'orientationX':msg.pose.pose.orientation.x,
            'orientationY':msg.pose.pose.orientation.y,
            'orientationZ':msg.pose.pose.orientation.z,
            'orientationW':msg.pose.pose.orientation.w,
            'linearVelocity':msg.twist.twist.linear,
            'angularVelocity':msg.twist.twist.angular,
        }

    def timerCallback(self):
        global laser,recentx,recenty,dataPosix,dataPosiy,linear,angular,orienx,orieny,orienz,orienw,theta,Sfront,Senfront
        dataPosix = self.valueOdometry['positionX']
        dataPosiy = self.valueOdometry['positionY']
        orienx = self.valueOdometry['orientationX']
        orieny = self.valueOdometry['orientationY']
        orienz = self.valueOdometry['orientationZ']
        orienw = self.valueOdometry['orientationW']
        laser = self.valueLaserRaw['ranges']
        recentx = self.valueOdometry['positionX']
        recenty = self.valueOdometry['positionY']
        siny_cosp =  2 * ((self.valueOdometry["orientationW"]*self.valueOdometry["orientationZ"]) + (self.valueOdometry["orientationX"]*self.valueOdometry["orientationY"]))
        cosy_cosp = 1 - ( 2 * ((self.valueOdometry["orientationY"]*self.valueOdometry["orientationY"]) + (self.valueOdometry["orientationZ"]*self.valueOdometry["orientationZ"])))
        theta = math.atan2(siny_cosp,cosy_cosp)
        linearVelocity = linear #m/s
        angularVelocity = angular #rad/s
        # linearVelocity,angularVelocity = robotLoop()
        SensorUpdate()
        if state == 1:
            GoHome(6)
        # WhatDoISee()
        # SmartWander()
        # linear,angular = robotLoop()
        self.publishVelocityCommand(linearVelocity,angularVelocity)

def DumbWander():
    global linear
    global angular
    front_laser = laser[0:15]+laser[345:360]
    if any((r < 0.3 and r > 0) for r in front_laser):
        print('Obstacle detected. Stopping.')
        linear = 0.0
        angular = 0.0
    else:
        linear = 0.1
        angular = 0.0
        print('The way is clear sir')

def SmartWander():
    #wandstate guide || 1 = patrol || 0 = obstacle found || 2 = turning left
    global linear, angular, wandstate, passcal, theta, target_angle
    front_laser = laser[0:8] + laser[352:360]
    avoid_laser = laser[0:15] + laser[345:360]

    # if 0.005 < laser[24] < 0.4  or 0.005 < laser[25] < 0.4:
    #         linear = 0.0
    #         angular = -0.1
    #         print('Rim avoided')
    # if 0.005 < laser[334] < 0.4  or 0.005 < laser[335] < 0.4:
    #         linear = 0.0
    #         angular = 0.1
    #         print('Rim avoided')

    if wandstate == 3:
        GoTo(-10)
        
    elif wandstate == 0 or wandstate == 2:
        print('Turn to random angle.')
        random_angle1 = random.uniform(40, 100)
        random_angle2 = random.uniform(-100, -40)
        selected_angle = random.choice([random_angle1, random_angle2])
        linear, angular, passcal = TurnTo(selected_angle)
        # linear, angular, passcal = TurnTo(90)
    if any((r < 0.18 and r > 0) for r in laser[0:25] + laser[335:360]):
        print('Obstacle detected. Avoiding.')
        if wandstate == 1: #detect obstacle during patrol
            linear = 0.0
            angular = 0.0
            print('STOP')
            # wandstate = 0
            wandstate = 3
    elif wandstate == 1: # patroling
        print('The way is clear sir')
        linear = 0.4 # Move forward when the way is clear
        angular = 0.0
        
def WhatDoISee():
    global linear
    global angular
    front_laser = laser[0:10]+laser[350:360]
    left_laser = laser[70:110]
    right_laser = laser[250:290]
    back_laser = laser[170:190]

    obstacle_char = 'X'
    linear = 0.0
    angular = 0.0

    # Create a 2D grid to represent the environment
    grid = [[' ' for _ in range(6)] for _ in range(6)]

    if any((r < 0.3 and r > 0) for r in front_laser):
        grid[0][2] = obstacle_char
    if any((r < 0.3 and r > 0) for r in left_laser):
        grid[2][0] = obstacle_char
    if any((r < 0.3 and r > 0) for r in right_laser):
        grid[2][4] = obstacle_char
    if any((r < 0.3 and r > 0) for r in back_laser):
        grid[4][2] = obstacle_char
        
    # grid[5][0] = '_'
    # grid[5][1] = '_'
    # grid[5][2] = '_'
    # grid[5][3] = '_'
    # grid[5][4] = '_'

    row_index = 5
    for i in range(5):
        grid[row_index][i] = '_'

    # Display the grid in the terminal
    for row in grid:
        print(' '.join(row))

def CorridorFollow(centimeter):
    global linear , angular, Sfront , Sback , Sleft , Sright , goaldistance , previousx, previousy , deltadistance , remaindistance ,count , goaldistance, totaldistance , laser ,state
    goaldistance = centimeter/100
    deltaX = 0.35
    deltaY = 1.0
    maxturn = 0.30 #0.32 0.37
    maxspeed = 0.05
    if Sleft >= deltaX:
        Sleft = deltaX
    if Sright >= deltaX:
        Sright = deltaX
    slop = deltaY/deltaX
    leftfar = slop*Sleft
    leftnear = -(slop*Sleft) +1
    rightfar = slop*Sright
    rightnear = -(slop*Sright) +1
    while totaldistance < goaldistance :  
        print('Speed = ',linear,'  ','Speed Turn = ',angular)
        if any((r < 0.3 and r > 0) for r in laser[0:7]+laser[353:360]):  
            linear = 0.0
            print('obstacle found!!') 
            print('remaining distance:',remaindistance*100, 'cm')
            totaldistance = goaldistance
        else:
            deltadistance = math.sqrt(((recentx-previousx)*(recentx-previousx))+((recenty-previousy)*(recenty-previousy))) 
            totaldistance = totaldistance + deltadistance
            previousx = recentx
            previousy = recenty
            remaindistance = goaldistance - totaldistance
            print('total distance:',totaldistance)

            if(deltadistance==0.0) and count != 1:
                count = 1
            else:
                pass

            if count == 1:
                angular = (leftnear*rightfar*(-maxturn)) + (leftfar*rightnear*maxturn)
                linear = (-((deltaY/maxturn)*angular)+1)*maxspeed

            else:
                totaldistance = 0.0
                linear = 0.0 #m/s
                angular = 0.0

            if linear >= maxspeed:
                linear = maxspeed
            if linear <= 0.0:
                linear = 0.0
            # if Sleft >= 0.25 and Sright >= 0.25:
            #     linear = 0.0
            #     angular = 0.0
        return linear,angular
    else:
        print("destination reached")
        linear = 0.0 #m/s
        angular = 0.0
        state = state+1
    totaldistance = 0.0
    return linear,angular

def SensorUpdate():
    global Sfront
    global Sback
    global Sleft
    global Sright
    global Senfront
    global Senback
    global Senleft
    global Senright
    global Sfront1
    global Sback1
    global Sleft1
    global Sright1

    for DATA1 in laser[0:20]+ laser[340:360]:
            Senfront.append(DATA1)
            x = len(Senfront)
            Sfront1 = Sfront1 + DATA1
            if x == 41 :
                #print('len',x)
                Sfront = Sfront1/x
                Senfront = []
                Sfront1 = 0
                #print (Sfront,'  ',Senfront)
    for DATA2 in laser[70:110]: #data[55:95]:
            Senleft.append(DATA2)
            x = len(Senleft)
            Sleft1 = Sleft1 + DATA2
            if x == 41 :
                #print('len',x)
                Sleft = Sleft1/x
                Senleft = []
                Sleft1 = 0
                #print (Sfront,'  ',Senfront)
    for DATA3 in laser[160:200]:
            Senback.append(DATA3)
            x = len(Senback)
            Sback1 = Sback1 + DATA3
            if x == 41 :
                #print('len',x)
                Sback = Sback1/x
                Senback = []
                Sback1 = 0
                #print (Sfront,'  ',Senfront)
    for DATA4 in laser[250:290]:#data[265:305]:
            Senright.append(DATA4)
            x = len(Senright)
            Sright1 = Sright1 + DATA4
            if x == 41 :
                #print('len',x)
                Sright = Sright1/x
                Senright = []
                Sright1 = 0

def TurnClosest():
    global linear
    global angular
    linear=0.0
    angular=0.0
    min = 5.0
    r = 0.0
    for i in laser[0:360]:
        if min > i and i > 0.0:
            min = i
        r = laser.index(min)
    if 350 < r < 360 or 0 < r < 10:
        linear = 0.0
        angular = 0.0
    elif 11 < r < 180:
        linear = 0.0
        angular = -((r/180)-1)*1.5
    elif 181 < r < 349:
        linear = 0.0
        angular = (((r-180)/180)-1)*1.5

def GoTo(centimeter):
    global direction,previousx,previousy,totaldistance,deltadistance,goaldistance,count,linear,angular,state,wandstate
    goaldistance = centimeter/100
    if centimeter >= 0:
        direction = 1
        while totaldistance < goaldistance :    
            deltadistance = math.sqrt(((recentx-previousx)*(recentx-previousx))+((recenty-previousy)*(recenty-previousy)))  
            #print('deltadistane = ',deltadistance)
            #print('recentx = ',recentx)
            #print('recenty = ',recenty)
            #print('previousx = ',previousx)
            #print('previousy = ',previousy)
            totaldistance = totaldistance + deltadistance
            previousx = recentx
            previousy = recenty
            print(totaldistance)

            if(deltadistance==0.0) and count != 1:
                count = 1
            else:
                pass

            if count == 1:
                linear = direction * 0.1 #m/s
                angular = 0.0

            else:
                totaldistance = 0.0
                linear = 0.0 #m/s
                angular = 0.0

            return linear,angular#,0
        else:
            wandstate = 0
            print("destination reached")
            linear = 0.0 #m/s
            angular = 0.0
            count = 0
            direction = 0
            state = state+1
        passwalk = goaldistance - totaldistance
        totaldistance = 0.0    
        return linear,angular#,passwalk
    else:
        direction = -1
        while totaldistance > goaldistance :    
            deltadistance = math.sqrt(((recentx-previousx)*(recentx-previousx))+((recenty-previousy)*(recenty-previousy)))  
            #print('deltadistane = ',deltadistance)
            #print('recentx = ',recentx)
            #print('recenty = ',recenty)
            #print('previousx = ',previousx)
            #print('previousy = ',previousy)
            totaldistance = totaldistance - deltadistance
            previousx = recentx
            previousy = recenty
            print(totaldistance)

            if(deltadistance==0.0) and count != 1:
                count = 1
            else:
                pass

            if count == 1:
                linear = direction * 0.1 #m/s
                angular = 0.0

            else:
                totaldistance = 0.0
                linear = 0.0 #m/s
                angular = 0.0

            return linear,angular#,0
        else:
            print("destination reached")
            wandstate = 0
            linear = 0.0 #m/s
            angular = 0.0
            count = 0
            direction = 0
            state = state+1
        passwalk = goaldistance - totaldistance
        totaldistance = 0.0    
        return linear,angular#,passwalk

def turn(angle): #senpai
    global linear
    global angular
    global state
    global totaldegree
    global predegree
    global nowdegree

    an = 0

    ##if totaldegree == 0:
    if angle>360 and an == 0:
        angle = angle-((math.floor(angle/360))*360)
        an = 1
        if angle > 180:
            angle = angle-360
    elif angle < -360 and an == 0:
        angle = angle+((math.floor(abs(angle)/360))*360)
        an = 1
        if angle < -180:
            angle = 360 + angle
    try:
        euler_from_quaternion(orienx, orieny, orienz, orienw) ## ใช้ degreeZ :หมุนซ้าย 0 ถึง 180 ต่อด้วย -180 ถึง 0
        linear=0.0
        angular=0.0
        nowdegree = degreeZ
        if predegree == 0:
            predegree = degreeZ
        if totaldegree<abs(angle) :
            totaldegree = totaldegree + abs(abs(nowdegree)-abs(predegree))
            #print('pre = ' , abs(predegree))
            ##print('now = ' , abs(nowdegree))
            #print('dis = ' , abs(abs(nowdegree)-abs(predegree)))
            #print('totaldegree' , totaldegree)
            predegree = nowdegree
            if angle > 0:
                if angle - totaldegree < 9:
                    angular = 0.05
                else:
                    angular = 0.01
            elif angle < 0:
                if abs(angle) - totaldegree < 9:
                    angular = -0.05
                else:                
                    angular = -0.01
        else:
            totaldegree = 0
            predegree = 0
            state = state + 1
    except NameError:
        print('Check Node or 00.lunch')
    except:
        print("Something else went wrong")
    return linear,angular

def TurnTo(degrees_to_turn):
    global wandstate, linear, angular, theta, passcal,target_angle,kp
    passcal = 0.0

    if wandstate == 0:
        kp = 6

        # Calculate the target angle by adding the desired turn to the current angle
        # Calculate the target angle by adding the desired turn to the current angle
        if degrees_to_turn >= 0:
            target_angle = math.degrees(theta) + degrees_to_turn
        else:
            target_angle = math.degrees(theta) - abs(degrees_to_turn)

        # Normalize the target angle to be in the range of [-180, 180] degrees
        target_angle = (target_angle + 180) % 360 - 180

        wandstate = 2

    # Calculate the error as the difference between the target angle and the current angle
    error = target_angle - math.degrees(theta)

    # Set the proportional control gain (kp) and limit the maximum angular velocity
    max_angular_velocity = 1.8  # You can adjust this value
    angular = kp * error


    if abs(angular) > max_angular_velocity:
            angular = max_angular_velocity if angular > 0 else -max_angular_velocity

    # Check if the desired angle is reached
    if abs(error) <= 10.0:  # You can adjust this threshold
        linear = 0.0
        angular = 0.0
        totalturn = 0.0
        passcal = error
        print('TurnDone')
        wandstate = 1
    else:
        print(f'Turning: Current angle: {math.degrees(theta)}, Desired turn: {degrees_to_turn} degrees, Error: {error}')

    return linear, angular, passcal

def GTNN(Nnode):
    global linear
    global angular
    global Sfront
    global Sback
    global Sleft
    global Sright
    global totaldegree
    global predegree
    global nowdegree
    global totaldistance
    global state
    global Orien_help_turn

    SensorUpdate()
    nodedistance = 0.3 # in meter
    Nnodes = abs(Nnode)
    total_walk = nodedistance*Nnodes*100 # in centimeter
    number_wall = 0
    min = 0.05
    speed_turn = 0.0 #0.1
    speed = 0.1#0.08
    error_detect_wall = 1.8
    error_walk_nearwall = 0.97
    error_walk_farwall = 2-error_walk_nearwall
    output = 0

    ####### Orientation to turn #########
    euler_from_quaternion(orienx, orieny, orienz, orienw) ## ใช้ degreeZ :หมุนซ้าย 0 ถึง 180 ต่อด้วย -180 ถึง 0
    nowdegree = degreeZ 
    if predegree == 0:
        predegree = nowdegree
    totaldegree = totaldegree + nowdegree-predegree
    predegree = nowdegree
    if totaldegree != 0 :
        #print('totaldegree = ' , totaldegree , totaldistance)
        if totaldegree > 0:
            Orien_help_turn = -0.1
        elif totaldegree < 0:
            Orien_help_turn = 0.1

    ####### condition that robot see wall ###########
    if min <= Sleft <= (nodedistance*error_detect_wall): #### wall in left
        number_wall = 1
    elif min <= Sright <= (nodedistance*error_detect_wall): #### wall in right
        number_wall = 2
    elif min <= Sleft <= (nodedistance*error_detect_wall) and min <= Sright <= (nodedistance*error_detect_wall): #### wall in left and right
        number_wall = 3
    else : number_wall = 0
    print('wall GTNN: ', number_wall)
    ######## walk @###############
    #if output == 0:
    #    if Nnode >0:
    if Sfront >= (nodedistance/2):
        #print('walk')
        if totaldistance <= (total_walk/100):
            linear,angular = GoTo(total_walk)
            if number_wall == 0: ##### No wall
                linear,angular = GoTo(total_walk)
            if number_wall == 1: #### wall in left
                if min <= Sleft <= (nodedistance*error_walk_nearwall):
                    angular = -speed_turn
                if Sleft >= (nodedistance*error_walk_farwall):
                    angular = speed_turn
                else: angular = 0.0
            if number_wall == 2: #### wall in right
                if min <= Sright <= (nodedistance*error_walk_nearwall):
                    angular = speed_turn
                if Sright >= (nodedistance*error_walk_farwall):
                    angular = -speed_turn
                else: A = 0.0
            if number_wall == 3: #### wall in left and right
                if min <= Sleft <= (nodedistance*error_walk_nearwall):
                    angular = -speed_turn
                if min <= Sright <= (nodedistance*error_walk_nearwall):
                    angular = speed_turn
                else: angular = 0.0
            angular = angular + Orien_help_turn
        else: output = 1
        if Nnode<0:
            linear = 0-linear
            
        if output == 1: 
            Out = round(totaldistance/nodedistance)
            print('Node that robot pass = ', Out)
            linear = 0.0 
            angular = 0.0
            Out = 0
            output = 0
            totaldistance = 0.0
            totaldegree = 0
            predegree = 0
            nowdegree = 0
            state = state +1
        print("state:",state)
    #print('Out', output)
    # V,A = 0.0,0.0
    #if Nnode <0:
    #    V,A = -V,A
    return linear,angular

def robotStop():
    node = rclpy.create_node('tb3Stop')
    publisher = node.create_publisher(Twist, 'cmd_vel', 1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    publisher.publish(msg)

def GoHome(nnn):
    global linear
    global angular
    global state
    global start
    linear = 0
    linear = 0
    if start == 1:
        if nnn == 1:
            if state == 1:
                linear,angular = GTNN(2)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 2:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 3:
                linear,angular = GTNN(2)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 4:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 5:
                linear,angular = GTNN(2)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 6:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 7:
                linear,angular = GTNN(1)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 8:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 9:
                linear,angular = GTNN(1)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 10:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0 #rad/s

        elif nnn == 2:
            if state == 1:
                linear,angular = turn(90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 2:
                linear,angular = GTNN(1)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 3:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 4:
                linear,angular = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 5:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s

        elif nnn == 3:
            if state == 1:
                linear,angular = turn(90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 2:
                linear,angular = GTNN(2)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 3:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 4:
                linear,angular = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 5:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 6:
                linear,angular = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 7:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s

        elif nnn == 4:
            if state == 1:
                linear,angular = GTNN(4)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 2:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 3:
                linear,angular = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 4:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 5:
                linear,angular = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 6:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s

        elif nnn == 5:
            if state == 1:
                linear,angular = GTNN(2)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 2:
                linear,angular = turn(90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 3:
                linear,angular = GTNN(2)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 4:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 5:
                linear,angular = GTNN(2)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 6:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 7:
                linear,angular = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 8:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 9:
                linear,angular = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 10:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s
        
        elif nnn == 6:
            if state == 1:
                linear,angular = GTNN(2)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 2:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 3:
                linear,angular = GTNN(2)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 4:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 5:
                linear,angular = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 6:
                linear,angular = turn(-90)
                ##print('start')
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 7:
                linear,angular = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = linear #m/s
                angularVelocity = angular #rad/s
            elif state == 8:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s

    else:
    ##print('stop')
        linearVelocity = 0.0 #m/s
        angularVelocity = 0.0 #rad/s
    print('state', state)
    return linearVelocity,angularVelocity        

def UniPlan():
    global linear
    global angular
    global state
    global start
    global Sfront
    global Sback
    global Sleft
    global Sright
    global all_wall
    global check_1
    global check_2
    global check_3
    global condition 

    linear = 0.0
    angular = 0.0
    min = 0.05
    speed_turn = 0.5
    speed = 0.08
    DisNode = 0.28 #in meter
    DisNodeSen = DisNode
    error_detect_wall = 1.2
    DisNodeSen_with_error = DisNodeSen*error_detect_wall
    

    SensorUpdate()
    ################ Detect Wall #################
    if min < Sfront <= DisNodeSen_with_error:
        all_wall[0] = 1
    else: all_wall[0] = 0
    if min < Sleft <= DisNodeSen_with_error:
        all_wall[1] = 1
    else: all_wall[1] = 0
    if min < Sback <= DisNodeSen_with_error:
        all_wall[2] = 1
    else: all_wall[2] = 0
    if min < Sright <= DisNodeSen_with_error:
        all_wall[3] = 1
    else: all_wall[3] = 0 
    wall_front = all_wall[0]
    wall_left = all_wall[1]
    wall_back = all_wall[2]
    wall_right = all_wall[3]
    ################ condition that reach goal ######################
    if check_1 == 0:
        if wall_front + wall_back + wall_left + wall_right >= 3:  # Goal
            start = 0 ##start = 0 code to stop
            condition = 0
        elif DisNode*1.4 <= Sleft <= DisNode*2 and Sright < DisNode  and Sback > DisNode*0.95:
            condition = 1 #### left to goal
        elif DisNode*1.4 <= Sright <= DisNode*2 and Sleft < DisNode and Sback > DisNode*0.95:
            condition = 2 #### right to goal
        else: condition = 3 ### random walk
        check_1 = 1
        if wall_front == 0:
            check_2 = 1
        if wall_front == 1:
            check_2 = 2
        if wall_right == 0:
            check_3 = 1
        if wall_right == 1:
            check_3 = 2
    ############### If not reach Goal ########################
        print("condition: ", condition ," check: ", check_1)
    if check_1 == 1:
        print("condition: ", condition ," check: ", check_1, check_2, check_3)
        if start == 1: 
            if condition == 0: # Set all Condition Except 2&3
                state ,linear,angular = 1 ,0.0,0.0 
                #state = 1
                check_1 = 0
                check_2 = 0
                check_3 = 0

            #### left to goal ####
            if condition == 1:    
                if state == 1:
                    linear,angular = turn(90) #m/s , rad/s 
                elif state == 2:
                    linear,angular = GTNN(1) #m/s , rad/s 
                elif state == 3:
                    condition = 0  
            ##### right to goal ####
            if condition == 2:    
                if state == 1:
                    linear,angular = turn(-90) #m/s , rad/s 
                elif state == 2:
                    linear,angular = GTNN(1) #m/s , rad/s 
                elif state == 3:
                    condition = 0   
            ####### alway forward ,turn right, turn left ######## 
            if condition == 3 :
                if check_2 == 1:
                    if state == 1:
                        linear,angular = GTNN(1)
                    elif state == 2:
                        condition = 0 
                if check_2 == 2:
                    if check_3 == 1:
                        if state == 1:
                            linear,angular = turn(-90)
                        elif state == 2:
                            condition = 0 
                    elif check_3 == 2:
                        if state == 1:
                            linear,angular = turn(90)
                        elif state == 2:
                            condition = 0
                    else: condition = 0
        else:
        ##print('stop')
            print("Goal")
            linear = 0.0 #m/s
            angular = 0.0 #rad/s
    print(linear,angular)
    print("all wall: ", all_wall)
    ##V,A = 0.0,0.0
    return linear,angular

def main(args=None):
    rclpy.init(args=args)
    tb3ControllerNode = Turtlebot3Controller()
    print('tb3ControllerNode created')
    try:
        rclpy.spin(tb3ControllerNode)
    except KeyboardInterrupt:
        print('Keyboard interrupt detected. Stopping...')
    finally:
        tb3ControllerNode.publishVelocityCommand(0.0, 0.0)
        tb3ControllerNode.destroy_node()
        robotStop()
        rclpy.shutdown()

def euler_from_quaternion(x, y, z, w):
        global degreeZ
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        degreeZ = (yaw_z/math.pi)*180 ##หมุนซ้าย 0 ถึง 180 ต่อด้วย -180 ถึง 0
        ##return roll_x, pitch_y, yaw_z # in radians

if __name__ == '__main__':
    main()
