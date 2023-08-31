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
            'orientation':msg.pose.pose.orientation,
            'linearVelocity':msg.twist.twist.linear,
            'angularVelocity':msg.twist.twist.angular,
        }

    def timerCallback(self):
        global laser
        global linear
        global angular
        global odom
        global pos
        laser = self.valueLaserRaw['ranges']
        odom = self.valueOdometry['orientation']
        pos = self.valueOdometry['position'].x
        #DumbWander()
        TurnClosest()
        #TurnTo(30)
        linearVelocity = linear #m/s
        angularVelocity = angular #rad/s
        self.publishVelocityCommand(linearVelocity,angularVelocity)

def DumbWander():
    global linear
    global angular
    if any((r < 0.3 and r > 0) for r in laser[0:20]+laser[340:360]):
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

def GoTo(distance_cm):
      # Adjust linear velocity as needed
    global linear
    global angular
    global pos
    linear=0.0
    angular=0.0
    if distance_cm < 0:
        linear *= -1
    distance_to_travel = abs(distance_cm / 10.0)
    initial_position = pos
    current_position = initial_position

    while abs(current_position - initial_position) < distance_to_travel:
        linear = 0.2
        angular = 0.0
        rclpy.spin_once(timeout_sec=0.1)
        current_position = pos
    linear = 0.0
    angular = 0.0

def TurnTo(tenths_of_degrees):
    global linear
    global angular
    current_orientation = odom
    current_yaw = quaternion_to_yaw(current_orientation)

    target_yaw = math.radians(tenths_of_degrees / 10.0)

    while abs(current_yaw - target_yaw) > 0.01:  # Adjust threshold as needed
        error = target_yaw - current_yaw

            # Determine the shortest direction to turn
        if error > math.pi:
            error -= 2 * math.pi
        elif error < -math.pi:
            error += 2 * math.pi

        angular_velocity = 0.2 if error > 0 else -0.2  # Adjust angular velocity as needed

        linear = 0
        angular = angular_velocity

        rclpy.spin_once(timeout_sec=0.1)
        current_yaw = quaternion_to_yaw(odom)

        linear = 0
        angular = 0

def quaternion_to_yaw(quaternion):
    t3 = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    t4 = +1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.degrees(math.atan2(t3, t4))

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

























