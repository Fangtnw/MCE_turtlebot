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

start = 1
state = 1
totaldegree = 0
predegree = 0
nowdegree = 0
count = 0

wandstate = 1
target_angle = None

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
        global laser,recentx,recenty,dataPosix,dataPosiy,linear,angular,orienx,orieny,orienz,orienw,theta
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
        # WhatDoISee()
        SmartWander()
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
    global linear, angular, wandstate, passcal, theta, target_angle
    front_laser = laser[0:25] + laser[335:360]
    avoid_laser = laser[0:15] + laser[345:360]

    if wandstate == 0 or wandstate == 2 :
        linear, angular, passcal = TurnTo(70)
    elif any((r < 0.25 and r > 0) for r in front_laser):
        print('Obstacle detected. Avoiding.')
        if wandstate == 1:
            linear = 0.0
            angular = 0.0
            print('STOP')
            wandstate = 0
    elif wandstate == 1:
        print('The way is clear sir') 
        linear = 1.5  # Move forward when the way is clear
        angular = 0.0
        
def WhatDoISee():
    global linear
    global angular
    front_laser = laser[0:15]+laser[345:360]
    left_laser = laser[60:120]
    right_laser = laser[240:300]
    back_laser = laser[165:195]

    obstacle_char = 'X'
    linear = 0.0
    angular = 0.0

    # Create a 2D grid to represent the environment
    grid = [[' ' for _ in range(5)] for _ in range(5)]

    if any((r < 0.3 and r > 0) for r in front_laser):
        grid[0][2] = obstacle_char
    if any((r < 0.3 and r > 0) for r in left_laser):
        grid[2][0] = obstacle_char
    if any((r < 0.3 and r > 0) for r in right_laser):
        grid[2][4] = obstacle_char
    if any((r < 0.3 and r > 0) for r in back_laser):
        grid[4][2] = obstacle_char

    # Display the grid in the terminal
    for row in grid:
        print(' '.join(row))

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

def TurnTo(degrees_to_turn):
    global wandstate, linear, angular, theta, passcal,target_angle,kp
    passcal = 0.0

    if wandstate == 0:
        kp = 6

        # Calculate the target angle by adding the desired turn to the current angle
        target_angle = math.degrees(theta) + degrees_to_turn

        # Normalize the target angle to be in the range of [-180, 180] degrees
        target_angle = (target_angle + 180) % 360 - 180

        wandstate = 2

    # Calculate the error as the difference between the target angle and the current angle
    error = target_angle - math.degrees(theta)

    # Set the proportional control gain (kp) and limit the maximum angular velocity
    max_angular_velocity = 3.0  # You can adjust this value
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



# def robotLoop():
#     while True:
#         SmartWander()
    # WhatDoISee()
    #TurnClosest()


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
