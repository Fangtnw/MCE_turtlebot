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

    # def GoTo(self, distance_cm):
    #     linear_velocity = 0.2  # Adjust linear velocity as needed

    #     if distance_cm < 0:
    #         linear_velocity *= -1

    #     distance_to_travel = abs(distance_cm / 10.0)
    #     initial_position = self.valueOdometry['position'].x
    #     current_position = initial_position

    #     while abs(current_position - initial_position) < distance_to_travel:
    #         self.publishVelocityCommand(linear_velocity, 0.0)

    #         rclpy.spin_once(self, timeout_sec=0.1)
    #         current_position = self.valueOdometry['position'].x

    #     self.publishVelocityCommand(0.0, 0.0)

    # def TurnTo(self, tenths_of_degrees):
    #     current_orientation = self.valueOdometry['orientation']
    #     current_yaw = self.quaternion_to_yaw(current_orientation)

    #     target_yaw = math.radians(tenths_of_degrees / 10.0)

    #     while abs(current_yaw - target_yaw) > 0.01:  # Adjust threshold as needed
    #         error = target_yaw - current_yaw

    #         # Determine the shortest direction to turn
    #         if error > math.pi:
    #             error -= 2 * math.pi
    #         elif error < -math.pi:
    #             error += 2 * math.pi

    #         angular_velocity = 0.2 if error > 0 else -0.2  # Adjust angular velocity as needed

    #         self.publishVelocityCommand(0.0, angular_velocity)

    #         rclpy.spin_once(self, timeout_sec=0.1)
    #         current_yaw = self.quaternion_to_yaw(self.valueOdometry['orientation'])

    #     self.publishVelocityCommand(0.0, 0.0)

    # def quaternion_to_yaw(self, quaternion):
    #     t3 = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    #     t4 = +1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    #     return math.degrees(math.atan2(t3, t4))

    def timerCallback(self):
        global laser
        # global linear
        # global angular
        # laser = self.valueLaserRaw['ranges']
        print("-----------------------------------------------------------")
        print('timer triggered')
        msg = self.scanSubscriber.get_last_message()

        # Call the obstacle avoidance function
        self.DumbWander00(self, msg)
            
    # def DumbWander(self, msg): #HW00
    #     forward_ranges = msg.ranges[0:30]
    #     # for forward_ranges in msg.ranges[0:30]+ msg.ranges[330:360]:
    #     for forward_ranges in msg.ranges[0:360]:
    #         if forward_ranges < 0.3:
    #             print('Stop')
    #             self.publishVelocityCommand(0.0, 0.0)
    #         else:
    #             print('forword')
    #             self.publishVelocityCommand(0.1, 0.0)

    # def HW01(self, msg):
    #         min = 5.0
    #         for i in msg.ranges[0:360]:
    #             if min > i and i > 0.0:
    #                 min = i
    #         r = msg.index(min)
    #         if 350 < r < 360 or 0 < r < 10:
    #             self.publishVelocityCommand(0, 0)
    #         elif 11 < r < 180:
    #             self.publishVelocityCommand(0, -((r/180)-1)*1.5)
    #         elif 181 < r < 349:
    #             self.publishVelocityCommand(0, (((r-180)/180)-1)*1.5)
                
    # def TurnClosest(self, msg): #HW01
    #     min_distance = 5.0  
    #     closest_angle = None
        
    #     for angle, distance in enumerate(msg.ranges):
    #         if 0.0 < distance < min_distance:
    #             min_distance = distance
    #             closest_angle = angle
        
    #     if closest_angle is None:
    #         self.publishVelocityCommand(0, 0)  # No valid detection
    #     else:
    #         if 350 < closest_angle < 360 or 0 <= closest_angle < 10:
    #             self.publishVelocityCommand(0, 0)  # Closest object is directly in front
    #         elif 10 < closest_angle <= 180:
    #             angular_speed = -((closest_angle / 180) - 1) * 1.5
    #             self.publishVelocityCommand(0, angular_speed)
    #         elif 180 < closest_angle <= 349:
    #             angular_speed = (((closest_angle - 180) / 180) - 1) * 1.5
    #             self.publishVelocityCommand(0, angular_speed)

    def DumbWander00(self, msg):
            if any((r < 0.2 and r > 0) for r in msg.ranges[0:30]+laser.ranges[330:360]):
                self.publishVelocityCommand(0.0, 0.0)
                self.get_logger().info('Obstacle detected. Stopping.')
            else:
                self.publishVelocityCommand(0.1, 0.0)
                self.get_logger().info('The way is clear sir')

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

























