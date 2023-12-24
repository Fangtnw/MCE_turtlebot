# Author : Khantaphon Chaiyo & 

#from tkinter import font
from optparse import check_builtin
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from std_msgs.msg import String
import time
import math
import random

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
            'positionx':None,
            'positiony':None,
            'positionz':None,
            'orientation':None,     #Datatype: geometry_msg/Quaternion (x,y,z,w)
            'orientationx':None,
            'orientationy':None,
            'orientationz':None,
            'orientationw':None,
            'linearVelocity':None,  #Datatype: geometry_msg/Vector3 (x,y,z)
            'angularVelocity':None, #Datatype: geometry_msg/Vector3 (x,y,z)
        }

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
            'positionx':msg.pose.pose.position.x,
            'positiony':msg.pose.pose.position.y,
            'positionz':msg.pose.pose.position.z,
            'orientation':msg.pose.pose.orientation,
            'orientationx':msg.pose.pose.orientation.x,
            'orientationy':msg.pose.pose.orientation.y,
            'orientationz':msg.pose.pose.orientation.z,
            'orientationw':msg.pose.pose.orientation.w,
            'linearVelocity':msg.twist.twist.linear,
            'angularVelocity':msg.twist.twist.angular,
        }
    def timerCallback(self):
        ##print('timer triggered')
        global V
        global A
        global data
        global dataPosix
        global dataPosiy
        global start
        global orienx
        global orieny
        global orienz
        global orienw
        global Sfront
        global Senfront

        data = self.valueLaserRaw['ranges']
        dataPosix = self.valueOdometry['positionx']
        dataPosiy = self.valueOdometry['positiony']
        orienx = self.valueOdometry['orientationx']
        orieny = self.valueOdometry['orientationy']
        orienz = self.valueOdometry['orientationz']
        orienw = self.valueOdometry['orientationw']
                
        #read sensors values
        ##print(self.valueBatteryState)
        ##print(self.valueLaserRaw)
        ##print(self.valueOdometry)
        #calculate command movement
        ###print(self.valueOdometry)
        ##print(self.valueOdometry['positionx'])

        '''linearVelocity = 0.1 #m/s
        angularVelocity = 0.0 #rad/s'''
        Sensor()
        ##HW00()
        linearVelocity,angularVelocity = HW42()
        self.publishVelocityCommand(linearVelocity,angularVelocity)
        
        ###try:
        """except NameError:
            print('Check Node or 00.lunch')
        except:
            print("Something else went wrong main")"""
def walk(speed,centimeter):
    global V
    global A
    global direction
    global previousx
    global previousy
    global totaldistance
    global deltadistance
    global goaldistance 
    global count
    global totaldegree
    global predegree
    global nowdegree
    global state
    goaldistance = centimeter/100
    if totaldistance < goaldistance :   
        deltadistance = math.sqrt(((dataPosix-previousx)*(dataPosix-previousx))+((dataPosiy-previousy)*(dataPosiy-previousy)))  
        #print('deltadistane = ',deltadistance)
        #print('recentx = ',recentx)
        #print('recenty = ',recenty)
        #print('previousx = ',previousx)
        #print('previousy = ',previousy)
        totaldistance = totaldistance + deltadistance
        previousx = dataPosix
        previousy = dataPosiy
        #print(totaldistance)

        if(deltadistance==0.0) and count != 1:
            count = 1
        else:
            pass

        if count == 1:
            linearVelocity =  speed #m/s
            angularVelocity = 0.0


        else:
            totaldistance = 0.0
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
        return linearVelocity,angularVelocity#,0
    else:
            #print("walkreach")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            count = 0
            direction = 0
            state = state + 1
            ###updatestep()
        ##passwalk = goaldistance - totaldistance
            totaldistance = 0.0    
    return linearVelocity,angularVelocity#,passwalk
def turn(Aspeed,angle):
    global V
    global A
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
        V=0.0
        A=0.0
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
                    A = 0.05
                else:
                    A = Aspeed
            elif angle < 0:
                if abs(angle) - totaldegree < 9:
                    A = -0.05
                else:                
                    A = -Aspeed
        else:
            totaldegree = 0
            predegree = 0
            state = state + 1
    except NameError:
        print('Check Node or 00.lunch')
    except:
        print("Something else went wrong")
    return V,A
def Sensor():
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

    for DATA1 in data[0:20]+ data[340:360]:
            Senfront.append(DATA1)
            x = len(Senfront)
            Sfront1 = Sfront1 + DATA1
            if x == 41 :
                #print('len',x)
                Sfront = Sfront1/x
                Senfront = []
                Sfront1 = 0
                #print (Sfront,'  ',Senfront)
    for DATA2 in data[60:120]: #data[55:95]:
            Senleft.append(DATA2)
            x = len(Senleft)
            Sleft1 = Sleft1 + DATA2
            if x == 61 :
                #print('len',x)
                Sleft = Sleft1/x
                Senleft = []
                Sleft1 = 0
                #print (Sfront,'  ',Senfront)
    for DATA3 in data[160:200]:
            Senback.append(DATA3)
            x = len(Senback)
            Sback1 = Sback1 + DATA3
            if x == 41 :
                #print('len',x)
                Sback = Sback1/x
                Senback = []
                Sback1 = 0
                #print (Sfront,'  ',Senfront)
    for DATA4 in data[240:300]:#data[265:305]:
            Senright.append(DATA4)
            x = len(Senright)
            Sright1 = Sright1 + DATA4
            if x == 61 :
                #print('len',x)
                Sright = Sright1/x
                Senright = []
                Sright1 = 0
                #print (Sfront,'  ',Senfront)
    #print('front= ',Sfront,'left= ',Sleft,'back= ',Sback,'right= ',Sright)
def walktest():
    global V
    global A
    global state
    global start
    V = 0
    A = 0
    if start == 1:
        
        """
        if state == 1:
            V,A = walk(0.1,30)
            print('start')
            linearVelocity = V #m/s
            angularVelocity = A #rad/s
        elif state == 2:
            turn(0.2,90)
            ##print('start')
            linearVelocity = V #m/s
            angularVelocity = A #rad/s
        elif state == 3:
            V,A = walk(0.1,10)
            ##print('start')
            linearVelocity = V #m/s
            angularVelocity = A #rad/s
        elif state == 4:
            turn(0.2,90)
            ##print('start')
            linearVelocity = V #m/s
            angularVelocity = A #rad/s
        elif state == 5:
            V,A = walk(0.1,30)
            ##print('start')
            linearVelocity = V #m/s
            angularVelocity = A #rad/s
        elif state == 6:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0 #rad/s
        """
        if state == 1:
            V,A = HW30(1)
            ##print('start')
            linearVelocity = V #m/s
            angularVelocity = A #rad/s
        elif state == 2:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0 #rad/s
    else:
    ##print('stop')
        linearVelocity = 0.0 #m/s
        angularVelocity = 0.0 #rad/s
    
    return linearVelocity,angularVelocity

def HW00():
    global V
    global A
    for DATA1 in data[0:30]+ data[330:360]:
        if DATA1 < 0.3:
            print('Stop')
            V = 0.0
            A = 0.0
        else:
            print('forword')
            V = 0.1
            A = 0.0
def HW01():
    global V
    global A
    global state
    V=0.0
    A=0.0
    min = 5.0
    for i in data[0:360]:
        if min > i and i > 0.0:
            min = i
    r = data.index(min)
    if 350 < r < 360 or 0 < r < 10:
        V = 0.0
        A = 0.0
    elif 11 < r < 180:
        V = 0.0
        A = -((r/180)-1)*1.5
    elif 181 < r < 349:
        V = 0.0
        A = (((r-180)/180)-1)*1.5
def HW10(Aspeed,angle):
    global V
    global A
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
        V=0.0
        A=0.0
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
                    A = 0.05
                else:
                    A = Aspeed
            elif angle < 0:
                if abs(angle) - totaldegree < 9:
                    A = -0.05
                else:                
                    A = -Aspeed
        else:
            totaldegree = 0
            predegree = 0
            state = state + 1
    except NameError:
        print('Check Node or 00.lunch')
    except:
        print("Something else went wrong")
    return V,A
def HW11(V,centimeter):
    global direction
    global previousx
    global previousy
    global totaldistance
    global deltadistance
    global goaldistance 
    global count
    global totaldegree
    global predegree
    global nowdegree
    global state
    goaldistance = centimeter/100
    if centimeter >= 0:
        direction = 1
        while totaldistance < goaldistance :   
            euler_from_quaternion(orienx, orieny, orienz, orienw) ## ใช้ degreeZ :หมุนซ้าย 0 ถึง 180 ต่อด้วย -180 ถึง 0
            ###nowdis = dataPosix
            nowdegree = degreeZ 
            if predegree == 0:
                predegree = nowdegree
            totaldegree = totaldegree + nowdegree-predegree
            predegree = nowdegree
            if totaldegree != 0 :
                print('totaldegree = ' , totaldegree)
                if totaldegree > 0:
                    A = -0.05
                elif totaldegree < 0:
                    A = 0.05
            deltadistance = math.sqrt(((dataPosix-previousx)*(dataPosix-previousx))+((dataPosiy-previousy)*(dataPosiy-previousy)))  
            #print('deltadistane = ',deltadistance)
            #print('recentx = ',recentx)
            #print('recenty = ',recenty)
            #print('previousx = ',previousx)
            #print('previousy = ',previousy)
            totaldistance = totaldistance + deltadistance
            previousx = dataPosix
            previousy = dataPosiy
            print(totaldistance)

            if(deltadistance==0.0) and count != 1:
                count = 1
            else:
                pass

            if count == 1:
                linearVelocity = direction * V #m/s
                angularVelocity = 0.0


            else:
                totaldistance = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0

            return linearVelocity,angularVelocity#,0
        else:
            print("walkreach")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            count = 0
            direction = 0
            state = state + 1
            ###updatestep()
        ##passwalk = goaldistance - totaldistance
        totaldistance = 0.0    
        return linearVelocity,angularVelocity#,passwalk
    else:
        direction = -1
        while totaldistance > goaldistance :    
            deltadistance = math.sqrt(((dataPosix-previousx)*(dataPosix-previousx))+((dataPosiy-previousy)*(dataPosiy-previousy)))  
            #print('deltadistane = ',deltadistance)
            #print('recentx = ',recentx)
            #print('recenty = ',recenty)
            #print('previousx = ',previousx)
            #print('previousy = ',previousy)
            totaldistance = totaldistance - deltadistance
            previousx = dataPosix
            previousy = dataPosiy
            print(totaldistance)

            if(deltadistance==0.0) and count != 1:
                count = 1
            else:
                pass

            if count == 1:
                linearVelocity = direction * V #m/s
                angularVelocity = A

            else:
                totaldistance = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0

            return linearVelocity,angularVelocity#,0
        else:
            print("walkreach")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            count = 0
            direction = 0
            totaldegree = 0
            predegree = 0
            nowdegree = 0
            state = state +1
            ###updatestep()
        ###passwalk = goaldistance - totaldistance
        totaldistance = 0.0    
        return linearVelocity,angularVelocity#,passwalk
def HW1012():
        global V
        global A
        global state
        global start
        V = 0
        A = 0
        if start == 1:
            
            if state == 1:
                V,A = HW11(0.1,300)
                print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 2:
                HW10(0.2,-1210)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 3:
                V,A = HW11(0.1,250)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 4:
                HW10(0.2,270)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 5:
                V,A = HW11(0.1,50)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 6:
                HW10(0.2,-450)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 7:
                V,A = HW11(0.1,50)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 8:
                HW10(0.2,-650)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 9:
                V,A = HW11(0.1,100)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 10:
                HW10(0.2,850)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 11:
                V,A = HW11(0.1,100)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 12:
                HW10(0.5,-500)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 13:
                V,A = HW11(0.1,120)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 14:
                HW10(0.5,250)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 15:
                start = 0 ##start = 0 code to stop
                """
            if state == 1:
                V,A = HW11(0.1,100)
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
                self.publishVelocityCommand(linearVelocity,angularVelocity)
            elif state == 2:
                HW10(0.5,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
                self.publishVelocityCommand(linearVelocity,angularVelocity)
            elif state == 3:
                V,A = HW11(0.1,100)
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
                self.publishVelocityCommand(linearVelocity,angularVelocity)
            elif state == 4:
                HW10(0.5,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
                self.publishVelocityCommand(linearVelocity,angularVelocity)
            elif state == 5:
                V,A = HW11(0.1,100)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
                self.publishVelocityCommand(linearVelocity,angularVelocity)
            elif state == 6:
                start = 0 ##start = 0 code to stop
                """
        else:
        ##print('stop')
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0 #rad/s
        
        return linearVelocity,angularVelocity
def HW12():
    global V
    global A
    global totaldistance
    global previousx
    global previousy
    global ran
    global K
    global ran2
  
    for DATA1 in data[0:37]+ data[323:360]:
        out = random.randint(0,5)
        if 0.005 < data[38] < 0.4  or 0.005 < data[39] < 0.4:
            V = 0.0
            A = -0.1
        if 0.005 < data[324] < 0.4  or 0.005 < data[325] < 0.4:
            V = 0.0
            A = 0.1
        if 0.005 < DATA1 < 0.24:
            if K == 0.0:
                print('Stop')
                V = 0.0
                A = 0.0
                if out == 1 or out == 2:
                    K = 1.0
                    ran = random.randint(-130,-40)
                else: 
                    K = 1.0
                    ran = random.randint(40,130)
        if 0.24 <= DATA1 < 0.26:
            V = 0.1
            A = 0.0
        if K == 1.0:
            if abs(totaldegree) >= (abs(ran)-5):
                K = 0.0
                print('change k' , K)
            V,A = HW10(2.0,ran)
            print('turntry', K, ran, totaldegree)
        else:
            print('forword')
            V = 0.4
            A = 0.0
            if previousx == 0:
                previousx = dataPosix
            if previousy == 0:
                previousy = dataPosiy
            dis = math.sqrt(((dataPosix-previousx)*(dataPosix-previousx))+((dataPosiy-previousy)*(dataPosiy-previousy)))
            totaldistance = totaldistance + dis
            print('pre = ' , previousx,previousy)
            previousx = dataPosix
            previousy = dataPosiy
            print('now = ' , dataPosix ,dataPosix)
            print('dis = ' , dis)
            print('total' , totaldistance)
            if totaldistance >= 2.5 :
                V = 0
                A = 0
                K = 1
                ran = random.randint(-150,150)
                print('turn')
                previousx = 0
                previousy = 0
                totaldistance = 0
    return V,A
def HW20():
    global V
    global A
    global front
    global right
    global left
    global back
    V = 0.0
    A = 0.0
    range = 0.25
    min = 0.001
    for DATA1 in data[0:30]+ data[331:360]: #data[0:45]+ data[316:360]:
        if min< DATA1 < range:
            front = 1
        else: front = 0
    for DATA1 in data[61:120]: #data[46:135]:
        if min< DATA1 < range:
            left = 1
        else: left = 0
    for DATA1 in data[160:201]: #data[136:225]:
        if min< DATA1 < range:
            back = 1
        else: back = 0
    for DATA1 in data[250:290]: #data[226:315]:
        if min< DATA1 < range:
            right = 1
        else: right = 0
    print('front=', front ,'  right=',right,'  back=',back,'  left=',left)

    return V,A
def HW200():
    global V
    global A
    global front
    global right
    global left
    global back
    global Sfront
    global Sback
    global Sleft
    global Sright
    V = 0.0
    A = 0.0
    range = 0.25
    min = 0.01
    if min< Sfront < range:
        front = 1
    else: front = 0
    if min< Sleft < range:
        left = 1
    else: left = 0
    if min< Sback < range:
        back = 1
    else: back = 0
    if min< Sright < range:
        right = 1
    else: right = 0
    print('front=', front ,'  right=',right,'  back=',back,'  left=',left)

    return V,A
def HW21():
    global V
    global A
    global Sfront
    global Sback
    global Sleft
    global Sright
    Sensor()
    Range_To_Stop = 0.9
    deltaX = 0.7
    deltaY = 1
    maxturn = 0.4 #0.32 0.37
    maxspeed = 0.078
    if Sleft >= deltaX:
        Sleft = deltaX
    if Sright >= deltaX:
        Sright = deltaX
    slop = deltaY/deltaX
    leftfar = slop*Sleft
    leftnear = -(slop*Sleft) +1
    rightfar = slop*Sright
    rightnear = -(slop*Sright) +1
    if Sfront <= 0.2:
        V = 0.0 
        A = 0.0
    else:
        A = (leftnear*rightfar*(-maxturn)) + (leftfar*rightnear*maxturn)
        V = (-((deltaY/maxturn)*A)+1)*maxspeed
        if V >= maxspeed:
            V = maxspeed
        if V <= 0.0:
            V = 0.0
        if Sleft >= Range_To_Stop and Sright >= Range_To_Stop:
            V = 0.0
            A = 0.0
    print('Speed = ',V,'  ','Speed Turn = ',A)
    return V,A

def GTNN(Nnode):
    global V
    global A
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

    Sensor()
    nodedistance = 0.295 # in meter
    Nnodes = abs(Nnode)
    total_walk = nodedistance*Nnodes*100 # in centimeter
    number_wall = 0
    min = 0.05
    speed_turn = 0.005 #0.1
    speed = 0.1 #0.08
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
            V,A = walk(speed,total_walk)
            if number_wall == 0: ##### No wall
                V,A = walk(speed,total_walk)
            if number_wall == 1: #### wall in left
                if min <= Sleft <= (nodedistance*error_walk_nearwall):
                    A = -speed_turn
                if Sleft >= (nodedistance*error_walk_farwall):
                    A = speed_turn
                else: A = 0.0
            if number_wall == 2: #### wall in right
                if min <= Sright <= (nodedistance*error_walk_nearwall):
                    A = speed_turn
                if Sright >= (nodedistance*error_walk_farwall):
                    A = -speed_turn
                else: A = 0.0
            if number_wall == 3: #### wall in left and right
                if min <= Sleft <= (nodedistance*error_walk_nearwall):
                    A = -speed_turn
                if min <= Sright <= (nodedistance*error_walk_nearwall):
                    A = speed_turn
                else: A = 0.0
            A = A + Orien_help_turn
        else: output = 1
        if Nnode<0:
            V = 0-V
            
        if output == 1: 
            Out = round(totaldistance/nodedistance)
            print('Node that robot pass = ', Out)
            V = 0.0 
            A = 0.0
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
    return V,A
def HW40(nnn):
    global V
    global A
    global state
    global start
    V = 0
    A = 0
    if start == 1:
        if nnn == 1:
            if state == 1:
                V,A = GTNN(2)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 2:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 3:
                V,A = GTNN(2)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 4:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 5:
                V,A = GTNN(2)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 6:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 7:
                V,A = GTNN(1)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 8:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 9:
                V,A = GTNN(1)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 10:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0 #rad/s

        elif nnn == 2:
            if state == 1:
                V,A = turn(0.3,90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 2:
                V,A = GTNN(1)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 3:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 4:
                V,A = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 5:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s

        elif nnn == 3:
            if state == 1:
                V,A = turn(0.3,90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 2:
                V,A = GTNN(2)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 3:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 4:
                V,A = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 5:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 6:
                V,A = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 7:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s

        elif nnn == 4:
            if state == 1:
                V,A = GTNN(4)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 2:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 3:
                V,A = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 4:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 5:
                V,A = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 6:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s

        elif nnn == 5:
            if state == 1:
                V,A = GTNN(2)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 2:
                V,A = turn(0.3,90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 3:
                V,A = GTNN(2)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 4:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 5:
                V,A = GTNN(2)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 6:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 7:
                V,A = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 8:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 9:
                V,A = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 10:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s
        
        elif nnn == 6:
            if state == 1:
                V,A = GTNN(2)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 2:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 3:
                V,A = GTNN(2)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 4:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 5:
                V,A = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 6:
                V,A = turn(0.3,-90)
                ##print('start')
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 7:
                V,A = GTNN(1)
                ##print('start')/s
                angularVelocity = 0.0 #
                linearVelocity = V #m/s
                angularVelocity = A #rad/s
            elif state == 8:
                start = 0 ##start = 0 code to stop
                linearVelocity = 0.0 #mrad/s

    else:
    ##print('stop')
        linearVelocity = 0.0 #m/s
        angularVelocity = 0.0 #rad/s
    print('state', state)
    return linearVelocity,angularVelocity

def HW41():
    global V
    global A
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

    V = 0.0
    A = 0.0
    min = 0.05
    speed_turn = 0.5
    speed = 0.08
    DisNode = 0.28 #in meter
    DisNodeSen = DisNode
    error_detect_wall = 1.35
    DisNodeSen_with_error = DisNodeSen*error_detect_wall

    Sensor()
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
        elif DisNode*1.25 <= Sleft <= DisNode*2 and Sright < DisNode  and Sback > DisNode*0.95:
            condition = 1 #### left to goal
        elif DisNode*1.25 <= Sright <= DisNode*2 and Sleft < DisNode and Sback > DisNode*0.95:
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
                state ,V,A = 1 ,0.0,0.0 
                #state = 1
                check_1 = 0
                check_2 = 0
                check_3 = 0

            #### left to goal ####
            if condition == 1:    
                if state == 1:
                    V,A = turn(speed_turn,90) #m/s , rad/s 
                elif state == 2:
                    V,A = GTNN(1) #m/s , rad/s 
                elif state == 3:
                    condition = 0  
            ##### right to goal ####
            if condition == 2:    
                if state == 1:
                    V,A = turn(speed_turn,-90) #m/s , rad/s 
                elif state == 2:
                    V,A = GTNN(1) #m/s , rad/s 
                elif state == 3:
                    condition = 0   
            ####### alway forward ,turn right, turn left ######## 
            if condition == 3 :
                if check_2 == 1:
                    if state == 1:
                        V,A = GTNN(1)
                    elif state == 2:
                        print("condition0")
                        condition = 0
                if check_2 == 2:
                    if check_3 == 1:
                        if state == 1:
                            V,A = turn(speed_turn,-90)
                        elif state == 2:
                            condition = 0 
                    elif check_3 == 2:
                        if state == 1:
                            V,A = turn(speed_turn,90)
                        elif state == 2:
                            condition = 0
                    else: condition = 0

        else:
        ##print('stop')
            print("Goal")
            V = 0.0 #m/s
            A = 0.0 #rad/s
    print(V,A)
    print("all wall: ", all_wall)
    ##V,A = 0.0,0.0
    return V,A
def HW42():
    global V
    global A
    global state
    global start
    global Sfront
    global Sback
    global Sleft
    global Sright
    global all_wall
    global Nowposi
    global check
    global check_1
    global check_2
    global check_3
    global firstcheck
    global startposi

    V = 0.0
    A = 0.0
    min = 0.05
    speed_turn = 0.7
    speed = 0.08
    DisNode = 0.3 #in meter
    DisNodeSen = DisNode
    error_detect_wall = 1.2
    DisNodeSen_with_error = DisNodeSen*error_detect_wall
    Sensor()
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

    ######### Find Position #####
    if start == 1:
        if firstcheck == 1: #### check first condition and quit to not update all_wall
            if all_wall == [1,0,0,0]: ## x2F y0L x3F y1F y2R
                check = "1000"
                firstcheck = 0
            elif all_wall == [0,0,1,0]: 
                check = "0010"
                firstcheck = 0
            elif all_wall == [0,1,0,0]: 
                check = "0100"
                firstcheck = 0
            elif all_wall == [0,0,0,1]: 
                check = "0001"
                firstcheck = 0
            elif all_wall == [1,0,1,0]: 
                check = "1010"
                firstcheck = 0
            elif all_wall == [1,1,0,0]: 
                check = "1100"
                firstcheck = 0
            elif all_wall == [1,0,0,1]: 
                check = "1001"
                firstcheck = 0
            elif all_wall == [0,1,0,1]: 
                check = "0101"
                firstcheck = 0
            elif all_wall == [0,0,1,1]: 
                check = "0011"
                firstcheck = 0
            elif all_wall == [0,1,1,0]: 
                check = "0110"
                firstcheck = 0
        print("wall_See: ", all_wall,", state_Check: ", check,", state: ",state, ", Nowposu: ", Nowposi)
        if check == "1000":
            if state == 1:
                V,A = GTNN(-1) # back
            elif state == 2:  #### check
                if all_wall == [0,0,0,1]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [0,1,1,1]:
                    start = 0
                    Nowposi = "y3F"
                elif all_wall == [0,0,1,1]:
                    start = 0
                    Nowposi = "z1F"
                elif all_wall == [0,1,0,0]:
                    start = 0
                    Nowposi = "y1R"
            if check_1 == 1:
                if state == 3:
                    V,A = GTNN(-1) #back
                elif state == 4: #### check
                    if all_wall == [0,1,1,0]:
                        start = 0
                        Nowposi = "z2F"
                    elif all_wall == [0,0,1,0]:
                        start = 0
                        Nowposi = "y2L"
        
        if check == "0010":
            if state == 1:
                V,A = GTNN(1) #Forward
            elif state == 2:  #### check
                if all_wall == [0,1,0,0]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [1,1,0,1]:
                    start = 0
                    Nowposi = "y3B"
                elif all_wall == [1,1,0,0]:
                    start = 0
                    Nowposi = "z1B"
                elif all_wall == [0,0,0,1]:
                    start = 0
                    Nowposi = "y1L"
            if check_1 == 1:
                if state == 3:
                    V,A = GTNN(1) #Forward
                elif state == 4: #### check
                    if all_wall == [1,0,0,1]:
                        start = 0
                        Nowposi = "z2B"
                    elif all_wall == [1,0,0,0]:
                        start = 0
                        Nowposi = "y2R"
        
        if check == "0100":
            if state == 1:
                V,A = GTNN(-1) #Back
            elif state == 2:  #### check
                if all_wall == [0,0,1,0]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [0,1,1,0]:
                    start = 0
                    Nowposi = "z0F"
                elif all_wall == [0,1,0,0]:
                    start = 0
                    Nowposi = "x2R"
                elif all_wall == [0,1,0,1]:
                    start = 0
                    Nowposi = "x1R"
            if check_1 == 1:
                if state == 3:
                    V,A = turn(speed_turn,90) #Left
                elif state == 4:
                    V,A = GTNN(1) #Forward
                elif state == 5: #### check
                    if all_wall == [1,1,0,0]:
                        start = 0
                        Nowposi = "x0F"
                    elif all_wall == [0,1,0,0]:
                        start = 0
                        Nowposi = "x3R"         

        if check == "0001":
            if state == 1:
                V,A = GTNN(1) #Forward
            elif state == 2:  #### check
                if all_wall == [1,0,0,0]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [1,0,0,1]:
                    start = 0
                    Nowposi = "z0B"
                elif all_wall == [0,0,0,1]:
                    start = 0
                    Nowposi = "x2L"
                elif all_wall == [0,1,0,1]:
                    start = 0
                    Nowposi = "x1L"
            if check_1 == 1:
                if state == 3:
                    V,A = turn(speed_turn,90) #Left
                elif state == 4:
                    V,A = GTNN(1) #Forward
                elif state == 5: #### check
                    if all_wall == [1,0,0,1]:
                        start = 0
                        Nowposi = "z0B"
                    elif all_wall == [0,1,0,1]:
                        start = 0
                        Nowposi = "x1L"         
        
        if check == "1010":
            if state == 1:
                V,A = turn(speed_turn,-90) #Right
            if state == 2:  #### check
                V,A = GTNN(1) #Forward
            elif state == 3:  #### check
                if all_wall == [1,0,0,1]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [1,1,0,0]:
                    check_1 = 2
                    state = state + 1

            if check_1 == 1:
                if state == 4:
                    V,A = turn(speed_turn,90) #Left
                elif state == 5:
                    V,A = GTNN(1) #Forward
                elif state == 6: #### check
                    if all_wall == [0,1,0,1]:
                        start = 0
                        Nowposi = "y4F"
                    elif all_wall == [0,0,0,1]:
                        check_1 = 3
                        state = state + 1 
            
            if check_1 == 3:
                if state == 7:
                    V,A = turn(speed_turn,90) #Left
                elif state == 8:
                    V,A = GTNN(1) #Forward
                elif state == 9: #### check
                    if all_wall == [0,1,0,0]:
                        start = 0
                        Nowposi = "y1R"
                    elif all_wall == [1,1,0,1]:
                        start = 0
                        Nowposi = "y3B"

            if check_1 == 2:
                if state == 4:
                    V,A = turn(speed_turn,-90) #Left
                elif state == 5:
                    V,A = GTNN(1) #Forward
                elif state == 6: #### check
                    if all_wall == [0,1,0,1]:
                        start = 0
                        Nowposi = "z3L"
                    elif all_wall == [0,0,0,1]:
                        start = 0
                        Nowposi = "y2F" 
        
        if check == "1100":
            if state == 1:
                V,A = turn(speed_turn,-90) #Right
            if state == 2:  #### check
                V,A = GTNN(1) #Forward
            elif state == 3:  #### check
                if all_wall == [0,1,0,1]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [0,1,0,0]:
                        start = 0
                        Nowposi = "y0F" 
                elif all_wall == [1,1,0,0]:
                        start = 0
                        Nowposi = "z0L"
                elif all_wall == [0,0,0,1]:
                        start = 0
                        Nowposi = "y2F"  
                    
            if check_1 == 1:
                if state == 4:
                    V,A = GTNN(1) #Forward
                elif state == 5: #### check
                    if all_wall == [0,1,0,0]:
                        start = 0
                        Nowposi = "x2R"
                    elif all_wall == [1,1,0,0]:
                        check_1 = 2
                        state = state + 1 
            
            if check_1 == 2:
                if state == 6:
                    V,A = turn(speed_turn,-90) #right
                elif state == 7:
                    V,A = GTNN(1) #Forward
                elif state == 8: #### check
                    if all_wall == [0,1,0,1]:
                        start = 0
                        Nowposi = "z3L"
                    elif all_wall == [0,0,0,1]:
                        start = 0
                        Nowposi = "y2F"
        
        if check == "1001":
            if state == 1:
                V,A = GTNN(-1) #Back
            elif state == 2:  #### check
                if all_wall == [0,1,0,1]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [0,0,0,1]:
                        start = 0
                        Nowposi = "y0B" 
                elif all_wall == [0,0,1,1]:
                        start = 0
                        Nowposi = "z0R"
                elif all_wall == [0,1,0,0]:
                        start = 0
                        Nowposi = "y2B"  
                    
            if check_1 == 1:
                if state == 3:
                    V,A = GTNN(-1) #back
                elif state == 4: #### check
                    if all_wall == [0,0,0,1]:
                        start = 0
                        Nowposi = "x2L"
                    elif all_wall == [0,0,1,1]:
                        check_1 = 2
                        state = state + 1 
            
            if check_1 == 2:
                if state == 5:
                    V,A = turn(speed_turn,90) #Left
                elif state == 6:
                    V,A = GTNN(1) #Forward
                elif state == 7: #### check
                    if all_wall == [0,1,0,1]:
                        start = 0
                        Nowposi = "z3L"
                    elif all_wall == [0,0,0,1]:
                        start = 0
                        Nowposi = "y2F"
            
        if check == "0101":
            if state == 1:
                V,A = GTNN(1) # forward
            elif state == 2:  #### check
                if all_wall == [0,1,0,0]:
                    start = 0
                    Nowposi = "x2F"
                elif all_wall == [1,0,0,1]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [1,1,0,0]:
                    check_1 = 2
                    state = state + 1
            if check_1 == 1:
                if state == 3:
                    V,A = turn(speed_turn,90) #left
                if state == 4:
                    V,A = GTNN(1) #forward
                elif state == 5: #### check
                    if all_wall == [0,1,0,1]:
                        start = 0
                        Nowposi = "y4F"
                    elif all_wall == [0,0,0,1]:
                        check_2 = 1
                        state = state + 1
            elif check_1 == 2:
                if state == 3:
                    V,A = turn(speed_turn,-90) #right
                if state == 4:
                    V,A = GTNN(1) #forward
                elif state == 5: #### check
                    if all_wall == [0,1,0,1]:
                        start = 0
                        Nowposi = "z3L"
                    elif all_wall == [0,0,0,1]:
                        start = 0
                        Nowposi = "y2F"
            if check_2 == 1:
                if state == 6:
                    V,A = turn(speed_turn,90) #left
                if state == 7:
                    V,A = GTNN(1) #forward
                elif state == 8: #### check
                    if all_wall == [0,1,0,0]:
                        start = 0
                        Nowposi = "y1R"
                    elif all_wall == [1,1,0,1]:
                        start = 0
                        Nowposi = "y3B"          
        
        if check == "0011":
            if state == 1:
                V,A = turn(speed_turn,90) #left
            elif state == 2:
                V,A = GTNN(1) # forward
            elif state == 3:  #### check
                if all_wall == [0,1,0,1]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [0,1,0,0]:
                    start = 0
                    Nowposi = "y0F"
                elif all_wall == [1,1,0,0]:
                    start = 0
                    Nowposi = "z0L"
                elif all_wall == [0,0,0,1]:
                    start = 0
                    Nowposi = "y2F"
            if check_1 == 1:
                if state == 4:
                    V,A = GTNN(1) #forward
                elif state == 5: #### check
                    if all_wall == [0,1,0,0]:
                        start = 0
                        Nowposi = "x2R"
                    elif all_wall == [1,1,0,0]:
                        check_2 = 1
                        state = state + 1
            if check_2 == 1:
                if state == 6:
                    V,A = turn(speed_turn,-90) #right
                elif state == 7:
                    V,A = GTNN(1) # forward
                elif state == 8: #### check
                    if all_wall == [0,1,0,1]:
                        start = 0
                        Nowposi = "z3L"
                    elif all_wall == [0,0,0,1]:
                        start = 0
                        Nowposi = "y2F"

        if check == "0110":
            if state == 1:
                V,A = GTNN(1) # forward
            elif state == 2:  #### check
                if all_wall == [0,1,0,1]:
                    check_1 = 1
                    state = state + 1
                elif all_wall == [0,1,0,0]:
                    start = 0
                    Nowposi = "y0F"
                elif all_wall == [1,1,0,0]:
                    start = 0
                    Nowposi = "z0L"
                elif all_wall == [0,0,0,1]:
                    start = 0
                    Nowposi = "y2F"
            if check_1 == 1:
                if state == 3:
                    V,A = GTNN(1) #forward
                elif state == 4: #### check
                    if all_wall == [0,1,0,0]:
                        start = 0
                        Nowposi = "x2R"
                    elif all_wall == [1,1,0,0]:
                        check_2 = 1
                        state = state + 1
            if check_2 == 1:
                if state == 5:
                    V,A = turn(speed_turn,-90) #right
                elif state == 6:
                    V,A = GTNN(1) # forward
                elif state == 7: #### check
                    if all_wall == [0,1,0,1]:
                        start = 0
                        Nowposi = "z3L"
                    elif all_wall == [0,0,0,1]:
                        start = 0
                        Nowposi = "y2F"
    if start == 0:
        state = 1
        startposi =0
        start = 5
    if startposi == 0:
        check = "0000"
        check = 0
        check_1 = 0
        check_2 = 0
        check_3 = 0
        print("state = ",state)
        #######    z0
        if Nowposi == "z0L": #z0L
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "z0F"
                state = 1
        elif Nowposi == "z0F": #z0F
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "y0F"
                state = 1
        elif Nowposi == "z0R": #z0R
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,90)
            elif state == 2:
                Nowposi = "z0F"
                state = 1
        elif Nowposi == "z0B": #z0B
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,180)
            elif state == 2:
                Nowposi = "z0F"
                state = 1
        #######    z1
        elif Nowposi == "z1L": #z1L
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:   
                Nowposi = "z0L"
                state = 1
        elif Nowposi == "z1F": #z1F
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,90)
            elif state == 2:
                Nowposi = "z1L"
                state = 1
        elif Nowposi == "z1R": #z1R
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,180)
            elif state == 2:
                Nowposi = "z1L"
                state = 1
        elif Nowposi == "z1B": #z1B
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "z1L"
                state = 1
        #######    z2
        elif Nowposi == "z2F": #z2F
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "y2F"
                state = 1
        elif Nowposi == "z2B": #z2B
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,90)
            elif state == 2:
                Nowposi = "z2R"
                state = 1
        elif Nowposi == "z2R": #z2R
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,90)
            elif state == 2:
                Nowposi = "z2F"
                state = 1
        elif Nowposi == "z2L": #z2L
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "z2F"
                state = 1
        #######    z3 
        elif Nowposi == "z3L": #z3L
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "z2L"
                state = 1
        elif Nowposi == "z3B": #z2B
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "z3L"
                state = 1
        #######    z4

        #######    y0  y0B
        elif Nowposi == "y0F": #y0F
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "x0F"
                state = 1
        #######    y1
        elif Nowposi == "y1R": #y1R
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "y2R"
                state = 1
        elif Nowposi == "y1L": #y1L
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "y1F"
                state = 1
        elif Nowposi == "y1F": #y1F
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "y1R"
                state = 1
        elif Nowposi == "y1B": #y1B
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,90)
            elif state == 2:    
                Nowposi = "y1R"
                state = 1
        #######    y2  
        elif Nowposi == "y2F": #y2F
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "x2F"
                state = 1
        elif Nowposi == "y2L": #y2L
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "y2F"
                state = 1
        elif Nowposi == "y2R": #y2R
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,90)
            elif state == 2:
                Nowposi = "y2F"
                state = 1
        elif Nowposi == "y2B": #y2B
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,90)
            elif state == 2:
                Nowposi = "y2R"
                state = 1
        #######    y3   Goal
        elif Nowposi == "y3F": #y3F
            print("Goal: ", Nowposi)
            if state == 1:
                V,A = 0.0,0.0
            elif state == 2:
                Nowposi = "y3R"
                state = 1
        elif Nowposi == "y3R": #y3R
            print("Goal: ", Nowposi)
            if state == 1:
                V,A = 0.0,0.0
            elif state == 2:
                Nowposi = "y3B"
                state = 1
        elif Nowposi == "y3B": #y3B
            print("Goal: ", Nowposi)
            if state == 1:
                V,A = 0.0,0.0
            elif state == 2:
                Nowposi = "y3L"
                state = 1
        elif Nowposi == "y3L": #y3L
            print("Goal: ", Nowposi)
            if state == 1:
                V,A = 0.0,0.0
            elif state == 2:
                Nowposi = "y3F"
                state = 1
        #######    y4
        elif Nowposi == "y4F": #y4F
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "x4F"
                state = 1

        #######    x0
        elif Nowposi == "x0R": #x0R
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "x1R" 
                state = 1
        elif Nowposi == "x0F": #x0F
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "x0R"
                state = 1
        #######    x1
        elif Nowposi == "x1R": #x1R
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "x2R"
                state = 1
        elif Nowposi == "x1L": #x1L
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "x1F"
                state = 1
        elif Nowposi == "x1F": #x1F
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "x1R"
                state = 1
        
        #######    x2
        elif Nowposi == "x2R": #x2R
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "x3R"
                state = 1
        elif Nowposi == "x2F": #x2F
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "x2R"
                state = 1
        elif Nowposi == "x2L": #x2L
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "x2F"
                state = 1
        #######    x3
        elif Nowposi == "x3R": #x3R
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,-90)
            elif state == 2:
                Nowposi = "x3B"
                state = 1
        elif Nowposi == "x3B": #x3B
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "y3B" ### Goal
                state = 1
        #######    x4
        elif Nowposi == "x4F": #x4F
            print(Nowposi)
            if state == 1:
                V,A = turn(speed_turn,90)
            elif state == 2:
                Nowposi = "x4L"
                state = 1
        elif Nowposi == "x4L": #x4L
            print(Nowposi)
            if state == 1:
                V,A = GTNN(1)
            elif state == 2:
                Nowposi = "x3R"
                state = 1

    return V,A
def robotStop():
    node = rclpy.create_node('tb3Stop')
    publisher = node.create_publisher(Twist, 'cmd_vel', 1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tb3ControllerNode = Turtlebot3Controller()
    print('tb3ControllerNode created')
    try:
        #Spin the node in the same thread if only callbacks are used
        rclpy.spin(tb3ControllerNode)

       #TODO: find method to spin the node asychronously, so that linear non-looped task can be programmed.
    except KeyboardInterrupt:
       pass
    tb3ControllerNode.publishVelocityCommand(0.0,0.0)
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

start = 1
state = 1
deltaturn = 0.0
totalturn = 0.0
previousturn = 0.0
previousx = 0.0
previousy = 0.0
totaldistance = 0.0
deltadistance = 0.0
goaldistance = 0.0
totaldegree = 0
predegree = 0
nowdegree = 0
count = 0
ran = 0
K = 0
ran2 = 0
front = 0
right = 0
left = 0
back = 0
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
if __name__ == '__main__':
    main()
