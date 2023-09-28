# Authors : Ben nakmai , Worawat Naiwan

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

        Sensor()
        linearVelocity,angularVelocity = HW12()
        self.publishVelocityCommand(linearVelocity,angularVelocity)

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
                Sfront = Sfront1/x
                Senfront = []
                Sfront1 = 0

    for DATA2 in data[70:110]:
            Senleft.append(DATA2)
            x = len(Senleft)
            Sleft1 = Sleft1 + DATA2
            if x == 41 :
                Sleft = Sleft1/x
                Senleft = []
                Sleft1 = 0

    for DATA3 in data[160:200]:
            Senback.append(DATA3)
            x = len(Senback)
            Sback1 = Sback1 + DATA3
            if x == 41 :
                Sback = Sback1/x
                Senback = []
                Sback1 = 0

    for DATA4 in data[250:290]:
            Senright.append(DATA4)
            x = len(Senright)
            Sright1 = Sright1 + DATA4
            if x == 41 :
                Sright = Sright1/x
                Senright = []
                Sright1 = 0

def HW10(Aspeed,angle):
    global V
    global A
    global state
    global totaldegree
    global predegree
    global nowdegree

    an = 0

    #if totaldegree == 0:
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
        euler_from_quaternion(orienx, orieny, orienz, orienw)
        V=0.0
        A=0.0
        nowdegree = degreeZ
        if predegree == 0:
            predegree = degreeZ
        if totaldegree < abs(angle) :
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

        degreeZ = (yaw_z/math.pi)*180 

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