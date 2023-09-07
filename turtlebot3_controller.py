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

start = 1
state = 1
totaldegree = 0
predegree = 0
nowdegree = 0
count = 0

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
        global laser,recentx,recenty,dataPosix,dataPosiy,linear,angular,orienx,orieny,orienz,orienw
        dataPosix = self.valueOdometry['positionX']
        dataPosiy = self.valueOdometry['positionY']
        orienx = self.valueOdometry['orientationX']
        orieny = self.valueOdometry['orientationY']
        orienz = self.valueOdometry['orientationZ']
        orienw = self.valueOdometry['orientationW']
        laser = self.valueLaserRaw['ranges']
        recentx = self.valueOdometry['positionX']
        recenty = self.valueOdometry['positionY']
        #DumbWander()
        #TurnClosest()
        if state ==1:
            GoTo(300)
        if state ==2:
            TurnTo(-1210)
        if state ==3:
            GoTo(250)
        if state ==4:
            TurnTo(270)
        if state ==5:
            GoTo(50)
        if state ==6:
            TurnTo(-450)
        if state ==7:
            GoTo(50)
        if state ==8:
            TurnTo(-650)
        if state ==9:
            GoTo(100)
        if state ==10:
            TurnTo(-850)
        if state ==11:
            GoTo(100)
        if state ==12:
            TurnTo(-500)    
        if state ==12:
            GoTo(120)         
        linearVelocity = linear #m/s
        angularVelocity = angular #rad/s
        self.publishVelocityCommand(linearVelocity,angularVelocity)

def DumbWander():
    global linear
    global angular
    if any((r < 0.3 and r > 0) for r in laser[0:15]+laser[345:360]):
        print('Obstacle detected. Stopping.')
        linear = 0.0
        angular = 0.0
    else:
        linear = 0.1
        angular = 0.0
        print('The way is clear sir')

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
    global direction,previousx,previousy,totaldistance,deltadistance,goaldistance,count,linear,angular,state
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
            linear = 0.0 #m/s
            angular = 0.0
            count = 0
            direction = 0
            state = state+1
        passwalk = goaldistance - totaldistance
        totaldistance = 0.0    
        return linear,angular#,passwalk
    
def TurnTo(angle):
    global linear, angular, state, totaldegree, predegree, nowdegree
    ##if totaldegree == 0:
    if angle>360:
        angle = angle-((math.floor(angle/360))*360)
    elif angle < -360:
        angle = angle+((math.floor(abs(angle)/360))*360)
    try:
        euler_from_quaternion(orienx, orieny, orienz, orienw) ## ใช้ degreeZ :หมุนซ้าย 0 ถึง 180 ต่อด้วย -180 ถึง 0
        linear=0.0
        angular=0.0
        nowdegree = degreeZ
        if predegree == 0:
            predegree = degreeZ
        if totaldegree<abs(angle) :
            totaldegree = totaldegree + abs(abs(nowdegree)-abs(predegree))
            print('pre = ' , abs(predegree))
            print('now = ' , abs(nowdegree))
            print('dis = ' , abs(abs(nowdegree)-abs(predegree)))
            print('totaldegree' , totaldegree)
            predegree = nowdegree
            if angle > 0:
                if angle - totaldegree < 15:
                    angular = 0.05
                else:
                    angular = 0.2
            elif angle < 0:
                if abs(angle) - totaldegree < 15:
                    angular = -0.05
                else:                
                    angular = -0.2
        else:
            totaldegree = 0
            predegree = 0
            print("destination reached")
            state = state + 1
    except Exception as e:
        print(f"An error occurred: {str(e)}")

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
        rclpy.spin(tb3ControllerNode)
    except KeyboardInterrupt:
        print('Keyboard interrupt detected. Stopping...')
    finally:
        tb3ControllerNode.publishVelocityCommand(0.0, 0.0)
        tb3ControllerNode.destroy_node()
        robotStop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

























