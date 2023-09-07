# Author : Khantaphon Chaiyo & 
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from std_msgs.msg import String
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
class Turtlebot3Controller(Node):
    
    global destination
    global kp
    global turncase
    
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
            'positionX':0.0,
            'positionY':0.0,
            'positionZ':0.0,
            'orientation':None,     #Datatype: geometry_msg/Quaternion (x,y,z,w)
            'orientationX':0.0,
            'orientationY':0.0,
            'orientationZ':0.0,
            'orientationW':0.0,
            'linearVelocity':None,  #Datatype: geometry_msg/Vector3 (x,y,z)
            'XlinearVelocity':0.0,
            'YlinearVelocity':0.0,
            'ZlinearVelocity':0.0,
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
            'positionX':msg.pose.pose.position.x,
            'positionY':msg.pose.pose.position.y,
            'positionZ':msg.pose.pose.position.z,
            'orientation':msg.pose.pose.orientation,
            'orientationX':msg.pose.pose.orientation.x,
            'orientationY':msg.pose.pose.orientation.y,
            'orientationZ':msg.pose.pose.orientation.z,
            'orientationW':msg.pose.pose.orientation.w,
            'linearVelocity':msg.twist.twist.linear,
            'XlinearVelocity':msg.twist.twist.linear.x,
            'YlinearVelocity':msg.twist.twist.linear.y,
            'ZlinearVelocity':msg.twist.twist.linear.z,
            'angularVelocity':msg.twist.twist.angular,
        }
    def timerCallback(self):
        #print(self.valueLaserRaw["ranges"][-180])
        #print('before map =',self.valueOdometry["orientationZ"])
        #global recentz
        #theta = math.acos(self.valueOdometry["orientationW"]) * 2
        #print(theta)
        #xangle = self.valueOdometry["orientationX"] / math.sin((theta/2))
        #yangle = self.valueOdometry["orientationY"] / math.sin((theta/2))
        #zangle = self.valueOdometry["orientationZ"] / math.sin((theta/2))
        #recentz = self.valueOdometry["orientationZ"]
        #print("theta=",theta,"x=",xangle,"y=",yangle,"z=",zangle)
        global recentx
        global recenty
        global theta
       
        recentx = self.valueOdometry['positionX']
        recenty = self.valueOdometry['positionY']

        siny_cosp =  2 * ((self.valueOdometry["orientationW"]*self.valueOdometry["orientationZ"]) + (self.valueOdometry["orientationX"]*self.valueOdometry["orientationY"]))
        cosy_cosp = 1 - ( 2 * ((self.valueOdometry["orientationY"]*self.valueOdometry["orientationY"]) + (self.valueOdometry["orientationZ"]*self.valueOdometry["orientationZ"])))
        theta = math.atan2(siny_cosp,cosy_cosp)
        #print(theta)

        linearVelocity,angularVelocity = robotloop()
        self.publishVelocityCommand(linearVelocity,angularVelocity)
        

def remap(num, in_min, in_max, out_min, out_max):
    return (num - in_min)*(out_max - out_min) / (in_max - in_min) + out_min

def updatestep():
    global step
    step = step + 1
    return step

def GoTo(centimeter):
    global direction
    global previousx
    global previousy
    global totaldistance
    global deltadistance
    global goaldistance 
    global count
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
                linearVelocity = direction * 0.08 #m/s
                angularVelocity = 0.0


            else:
                totaldistance = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0

            return linearVelocity,angularVelocity#,0
        else:
            print("Go done")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            count = 0
            direction = 0
            updatestep()
        passwalk = goaldistance - totaldistance
        totaldistance = 0.0    
        return linearVelocity,angularVelocity#,passwalk
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
                linearVelocity = direction * 0.08 #m/s
                angularVelocity = 0.0

            else:
                totaldistance = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0

            return linearVelocity,angularVelocity#,0
        else:
            print("Go done")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            count = 0
            direction = 0
            updatestep()
        passwalk = goaldistance - totaldistance
        totaldistance = 0.0    
        return linearVelocity,angularVelocity#,passwalk

def calibratewalk(centimgocal):
    global direction
    global previousx
    global previousy
    global totaldistance
    global deltadistance
    global goaldistance 
    global count
    centimeters = centimeters/100

    #gocal)
    if centimeters >= 0:
        direction = -1
        while goaldistance - totaldistance <= -0.001 :    
            deltadistance = math.sqrt(((recentx-previousx)*(recentx-previousx))+((recenty-previousy)*(recenty-previousy)))  
            #print('deltadistane = ',deltadistance)
            #print('recentx = ',recentx)
            #print('recenty = ',recenty)
            #print('previousx = ',previousx)
            #print('previousy = ',previousy)
            totaldistance = totaldistance - deltadistance
            previousx = recentx
            previousy = recenty
            #print(goaldistance-totaldistance)

            if(deltadistance==0.0) and count != 1:
                count = 1
            else:
                pass

            if count == 1:
                linearVelocity = direction * 0.01 #m/s
                angularVelocity = 0.0


            else:
                totaldistance = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0

            return linearVelocity,angularVelocity
        else:
            print("calibrate w done")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            count = 0
            direction = 0
            updatestep()
        totaldistance = 0.0    
        return linearVelocity,angularVelocity
    else:
        direction = 1
        while goaldistance - totaldistance >= 0.001  :    
            deltadistance = math.sqrt(((recentx-previousx)*(recentx-previousx))+((recenty-previousy)*(recenty-previousy)))  
            #print('deltadistane = ',deltadistance)
            #print('recentx = ',recentx)
            #print('recenty = ',recenty)
            #print('previousx = ',previousx)
            #print('previousy = ',previousy)
            totaldistance = totaldistance + deltadistance
            previousx = recentx
            previousy = recenty
            #print(goaldistance - totaldistance)

            if(deltadistance==0.0) and count != 1:
                count = 1
            else:
                pass

            if count == 1:
                linearVelocity = direction * 0.01 #m/s
                angularVelocity = 0.0

            else:
                totaldistance = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0

            return linearVelocity,angularVelocity 
        else:
            print("calibrate w done")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            count = 0
            direction = 0
            updatestep()
        totaldistance = 0.0    
        return linearVelocity,angularVelocity

def TurnTo(degrees):
    global previousturn
    global deltaturn
    global totalturn
    global bount
    global turncase
    passcal = 0.0
    diff = 0.0
    fordir = 0.0
    kp = 2
    #destination config
    destination = degrees
    if destination >= 360  :
        destination = destination % 360
    elif destination <= -360 :
        destination = destination % -360
    else:
        pass

    #print(destination)
    if destination >= 0 :
        if destination >= 180 :
            #print('rotateright')
            turncase = 'right'
            destination = 360 - destination
        elif destination < 180 :
             #print('rotateleft')
            turncase = 'left'   
    elif destination < 0 :
        if destination >= -180 :
            #print('rotateright')
            turncase = 'right'
        elif destination < -180 :
             #print('rotateleft')
            turncase = 'left'  
            destination = -360 - destination

    #find a solution to change orient.z to degree then boom finish
    if degrees >= 0 : 
        while destination-totalturn >= 0.1 :
            startdegree = theta
            if startdegree >= 0.0 :
                startdegree = remap(startdegree,0.0,3.14,0.0,180)
            else:
                startdegree = remap(startdegree,-3.14,-0.0,0.0,180) + 180
            #print('after map =',startdegree)

            deltaturn = abs(startdegree - previousturn)
            if deltaturn >= 100:
                deltaturn = abs(360-deltaturn)
            else:
                pass

            totalturn = totalturn + deltaturn
            previousturn = startdegree
            print(totalturn)

            #กันerror
            b4dif = abs(totalturn-destination) 
            if b4dif > 180:
                b4dif = abs(b4dif - 360) 
            else:
                pass
            #print(b4dif)    
            diff = b4dif/180
            diff = (kp*diff)+0.4
            diff = round(diff,1)

            if(deltaturn<=0.1) and bount != 1:
                bount = 1
            else:
                pass

            if bount == 1:
                if turncase == 'right':
                    #print('rotatingright')
                    linearVelocity = 0.0 #m/s
                    angularVelocity = -0.8*diff #rad/s
                elif turncase == 'left':
                    #print('rotatingleft')
                    linearVelocity = 0.0 #m/s
                    angularVelocity = 0.8*diff #rad/s  
            else:
                totalturn = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0        
            return linearVelocity,angularVelocity,0

        else:
            print("turn done")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            passcal = destination-totalturn
            bount = 0
            totalturn = 0.0
            updatestep()
            return linearVelocity,angularVelocity,passcal
        

    else :
        while destination-totalturn <= -0.1 :
            startdegree = theta
            if startdegree >= 0.0 :
                startdegree = remap(startdegree,0.0,3.14,0.0,180)
            else:
                startdegree = remap(startdegree,-3.14,-0.0,0.0,180) + 180
            #print('after map =',startdegree)

            deltaturn = abs(startdegree - previousturn)
            if deltaturn >= 100:
                deltaturn = abs(360-deltaturn)
            else:
                pass

            totalturn = totalturn - deltaturn
            previousturn = startdegree
            print(totalturn)

            #กันerror
            b4dif = abs(totalturn-destination) 
            if b4dif > 180:
                b4dif = abs(b4dif - 360) 
            else:
                pass
            #print(b4dif)    
            diff = b4dif/180
            diff = (kp*diff)+0.4
            diff = round(diff,1)

            if(deltaturn<=0.1) and bount != 1:
                bount = 1
            else:
                pass

            if bount == 1:
                if turncase == 'right':
                    #print('rotatingright')
                    linearVelocity = 0.0 #m/s
                    angularVelocity = -0.8*diff #rad/s
                elif turncase == 'left':
                    #print('rotatingleft')
                    linearVelocity = 0.0 #m/s
                    angularVelocity = 0.8*diff #rad/s  
            else:
                totalturn = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0        
            return linearVelocity,angularVelocity,0

        else:
            print("turn done")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            passcal = destination-totalturn
            bount = 0
            totalturn = 0.0
            updatestep()
            return linearVelocity,angularVelocity,passcal
    
def CheckTurn(degrees,cal):
    global previousturn
    global deltaturn
    global totalturn
    global bount
    global turncase
    passcal = 0.0
    diff = 0.0
    fordir = 0.0
    kp = 2
    #destination config
    destination = degrees
    if destination >= 360  :
        destination = destination % 360
    elif destination <= -360 :
        destination = destination % -360
    else:
        pass

    if destination >= 0 :
        if cal >= 0 :
            #print('rotateright')
            turncase = 'right'
        elif cal < 0 :
            #print('rotateleft')
            turncase = 'left'   
    elif destination < 0 :
        if cal >= 0 :
            #print('rotateright')
            turncase = 'left'
        elif cal < 0 :
             #print('rotateleft')
            turncase = 'right'  
    #find a solution to change orient.z to degree then boom finish
    if cal>= 0 : 
        while cal-totalturn >= 0.05 :
            startdegree = theta
            if startdegree >= 0.0 :
                startdegree = remap(startdegree,0.0,3.14,0.0,180)
            else:
                startdegree = remap(startdegree,-3.14,-0.0,0.0,180) + 180
            #print('after map =',startdegree)

            deltaturn = abs(startdegree - previousturn)
            if deltaturn >= 100:
                deltaturn = abs(360-deltaturn)
            else:
                pass

            totalturn = totalturn + deltaturn
            previousturn = startdegree
            print(totalturn)

            #กันerror
            b4dif = abs(totalturn-destination) 
            if b4dif > 180:
                b4dif = abs(b4dif - 360) 
            else:
                pass
            #print(b4dif)    
            diff = 0.13 #default 1 1.25 need to june

            if(deltaturn<=0.1) and bount != 1:
                bount = 1
            else:
                pass

            if bount == 1:
                if turncase == 'right':
                    #print('rotatingright')
                    linearVelocity = 0.0 #m/s
                    angularVelocity = -1*diff #rad/s
                elif turncase == 'left':
                    #print('rotatingleft')
                    linearVelocity = 0.0 #m/s
                    angularVelocity = diff #rad/s  
            else:
                totalturn = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0        
            return linearVelocity,angularVelocity

        else:
            print("calibrate t done")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            bount = 0
            totalturn = 0.0
            updatestep()
            return linearVelocity,angularVelocity
        

    else :
        while cal-totalturn <= -0.05 :
            startdegree = theta
            if startdegree >= 0.0 :
                startdegree = remap(startdegree,0.0,3.14,0.0,180)
            else:
                startdegree = remap(startdegree,-3.14,-0.0,0.0,180) + 180
            #print('after map =',startdegree)

            deltaturn = abs(startdegree - previousturn)
            if deltaturn >= 100:
                deltaturn = abs(360-deltaturn)
            else:
                pass

            totalturn = totalturn - deltaturn
            previousturn = startdegree
            print(totalturn)

            #กันerror
            b4dif = abs(totalturn-destination) 
            if b4dif > 180:
                b4dif = abs(b4dif - 360) 
            else:
                pass
            #print(b4dif)    
            diff = 1.25 #default 1 1.25 need to june

            if(deltaturn<=0.1) and bount != 1:
                bount = 1
            else:
                pass

            if bount == 1:
                if turncase == 'right':
                    #print('rotatingright')
                    linearVelocity = 0.0 #m/s
                    angularVelocity = -1*diff #rad/s
                elif turncase == 'left':
                    #print('rotatingleft')
                    linearVelocity = 0.0 #m/s
                    angularVelocity = diff #rad/s  
            else:
                totalturn = 0.0
                linearVelocity = 0.0 #m/s
                angularVelocity = 0.0        
            return linearVelocity,angularVelocity

        else:
            print("calibrate t done")
            linearVelocity = 0.0 #m/s
            angularVelocity = 0.0
            bount = 0
            totalturn = 0.0
            updatestep()
            return linearVelocity,angularVelocity

def robotloop():
    global turncal
    global gocal
    if step == 1:
        linearVelocity,angularVelocity = GoTo(300)
    elif step == 2 :
        linearVelocity,angularVelocity,turncal = TurnTo(-1210)
    elif step == 3 :
        linearVelocity,angularVelocity = CheckTurn(-1210,turncal)
    elif step == 4:
        linearVelocity,angularVelocity = GoTo(250)
    elif step == 5 :
        linearVelocity,angularVelocity,turncal = TurnTo(270)    
    elif step == 6 :
        linearVelocity,angularVelocity = CheckTurn(270,turncal)
    elif step == 7:
        linearVelocity,angularVelocity = GoTo(50)
    elif step == 8 :
        linearVelocity,angularVelocity,turncal = TurnTo(-450)
    elif step == 9 :
        linearVelocity,angularVelocity = CheckTurn(-450,turncal)
    elif step == 10:
        linearVelocity,angularVelocity = GoTo(50)
    elif step == 11 :
        linearVelocity,angularVelocity,turncal = TurnTo(-650)    
    elif step == 12 :
        linearVelocity,angularVelocity = CheckTurn(-650,turncal)
    elif step == 13:
        linearVelocity,angularVelocity = GoTo(100)
    elif step == 14 :
        linearVelocity,angularVelocity,turncal = TurnTo(850)
    elif step == 15 :
        linearVelocity,angularVelocity = CheckTurn(850,turncal)
    elif step == 16:
        linearVelocity,angularVelocity = GoTo(100)
    elif step == 17 :
        linearVelocity,angularVelocity,turncal = TurnTo(-500)    
    elif step == 18 :
        linearVelocity,angularVelocity = CheckTurn(-500,turncal)
    elif step == 19:
        linearVelocity,angularVelocity = GoTo(120)
    else :
        linearVelocity = 0.0 #m/s
        angularVelocity = 0.0
    return linearVelocity,angularVelocity 
    

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


if __name__ == '__main__':
    main()
