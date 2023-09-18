def TurnTo(degrees):
    global previousturn
    global deltaturn
    global totalturn
    global bount
    global turncase , wandstate, linear , angular
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
                    linear = 0.0 #m/s
                    angular = -1.5*diff #rad/s
                elif turncase == 'left':
                    #print('rotatingleft')
                    linear = 0.0 #m/s
                    angular = 1.5*diff #rad/s  
            else:
                totalturn = 0.0
                linear = 0.0 #m/s
                angular = 0.0        
            return linear,angular,0

        else:
            print("turn done")
            linear = 0.0 #m/s
            angular = 0.0
            passcal = destination-totalturn
            bount = 0
            totalturn = 0.0
            wandstate = 1
            # updatestep()
            return linear,angular,passcal
        

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
                    linear = 0.0 #m/s
                    angular = -1.5*diff #rad/s
                elif turncase == 'left':
                    #print('rotatingleft')
                    linear = 0.0 #m/s
                    angular = 1.5*diff #rad/s  
            else:
                totalturn = 0.0
                linear = 0.0 #m/s
                angular = 0.0        
            return linear,angular,0

        else:
            print("turn done")
            linear = 0.0 #m/s
            angular = 0.0
            passcal = destination-totalturn
            bount = 0
            totalturn = 0.0
            wandstate = 1
            # updatestep()
            return linear,angular,passcal

def remap(num, in_min, in_max, out_min, out_max):
    return (num - in_min)*(out_max - out_min) / (in_max - in_min) + out_min
