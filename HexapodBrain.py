import math
import threading
import time
import inputs  # For reading the controller input
from inputs import devices
import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface
i2c = busio.I2C(board.SCL, board.SDA)

# Create a PCA9685 object
pca = PCA9685(i2c)
pca.frequency = 50  # Standard for servos (50Hz)

def angle_to_pwm(angle):
    pwm_min = 150  # 500us
    pwm_max = 600  # 2500us
    return int(pwm_min + (angle / 180.0) * (pwm_max - pwm_min))
I2CLock = threading.Lock()

# A global variable to store the input state
controller_state = {}

# Function to check if the controller is connected
def is_controller_connected():
    # Get the list of connected devices
    for device in devices:
        # Check if a DualShock 4 controller is connected (you can also check for other controllers)
        print(device.name)
        if 'Sony' in device.name:  # This checks for the presence of a DualShock 4
            return True
    return False

def wait_for_controller():
    print("Waiting for DualShock 4 controller to be connected...")
    while not is_controller_connected():
        time.sleep(1)  # Wait for 1 second before checking again
        print("Controller not connected. Trying again...")
    
    print("Controller connected!")

# Define a function that reads DualShock 4 input
def read_dualshock4_input():
    global controller_state
    while True:
        events = inputs.get_key()
        
        for event in events:
            # Process the event (key/button press)
            if event.ev_type == 'Key':
                controller_state[event.ev_type] = event.ev_code  # Store the event code (e.g., button press)
                print(f"Button: {event.ev_code} Value: {event.ev_value}")

        time.sleep(0.01)  # Add a small delay to avoid hogging the CPU

# Mechanical leg segments measured from axis to axis
L1 = 6.0
L2 = 4.0
L3 = 6.0

# Mechanical joint limits
S_LIM = (-90, 90)
E_LIM = (-90, 90)
W_LIM = (-140, 0)

# Defined gait patterns
TRIPOD_GAIT = True
WAVE_GAIT = False
BOUNDING_GAIT = False

# debug flag for printing effector end location
DEBUG = False

class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Leg:
    def __init__(self, offset, servoS, servoE, servoW, Soffset):
        self.offset = offset
        self.servoS = servoS
        self.servoE = servoE
        self.servoW = servoW
        self.currentPos = Point3D(0, 0, 0)
        self.angleS = 0.0
        self.angleE = 0.0
        self.angleW = 0.0
        self.Soffest = Soffset

def write_leg(leg, point):
    try:
        #Find dx, dy, and dz based on initial offset
        dx = point.x - leg.offset.x
        dy = point.y - leg.offset.y
        dz = point.z - leg.offset.z
        
        #Solve for s, account for offset
        s = math.atan2(dy, dx)
        
        #Solve for f, flat leg length
        fx = dx - L1 * math.cos(s)
        fy = dy - L1 * math.sin(s)
        f = math.sqrt(fx * fx + fy * fy + dz * dz)
        
        #Use f to find e and w
        e = math.asin(dz / f) + math.acos((L2**2 - L3**2 + f**2) / (2 * L2 * f))
        w = -math.acos((L2**2 - L3**2 + f**2) / (2 * L2 * f)) - math.acos((L3**2 - L2**2 + f**2) / (2 * L3 * f))
        
        #Convert to degrees
        finalS = math.degrees(s) - leg.Soffest
        finalE = math.degrees(e)
        finalW = math.degrees(w)

        if finalS > 180:
            finalS = finalS - 360

        if finalS < -180:
            finalS = finalS + 360
        # Boundary check the leg angles
        if not (S_LIM[0] <= finalS <= S_LIM[1]):
            print("Invalid S angle!")
            print(leg.servoS)
            print(finalS)
            return 1
        if not (E_LIM[0] <= finalE <= E_LIM[1]):
            print("Invalid E angle!")
            print(leg.servoE)
            print(finalE)
            return 1
        if not (W_LIM[0] <= finalW <= W_LIM[1]):
            print("Invalid W angle!")
            print(leg.servoW)
            print(finalW)
            return 1
        
        # Write new angles and point
        leg.angleS = finalS
        leg.angleE = finalE
        leg.angleW = finalW

        # If error accumulates, use DEBUG to make controlled movements
        if DEBUG:
            endPoint = Point3D(L3 * math.cos(e) * math.cos(s) * math.cos(w) - L3 * math.sin(e) * math.cos(s) * math.sin(w) 
                                + L2 * math.cos(e) * math.sin(s)
                                + L1 * math.cos(s) + leg.offset.x,
                            L3 * math.cos(e) * math.sin(s) * math.cos(w) - L3 * math.sin(e) * math.sin(s) * math.sin(w) 
                                + L2 * math.cos(e) * math.sin(s)
                                + L1 * math.sin(s) + leg.offset.y,
                            L3 * math.sin(e) * math.cos(w) + L3 * math.cos(e) * math.sin(w)
                                + L2 * math.sin(e) + leg.offset.z)
            print(endPoint.x)
            print(endPoint.y)
            print(endPoint.z)
            print(finalS)
            print(f)
            print(finalE)
            print(finalW)
        
        return 0
    except:
        print("Math domain error!")
        return 1

def move_leg(leg, point, steps = 10):
    if write_leg(leg, point) != 0:
        #If leg cannot go to position
        print("Error in writeLeg")
        return 1

    startPoint = leg.currentPos
    for i in range(1,steps):
        
        t = i / steps  # Normalized interpolation factor (0 to 1)

        # Compute interpolated point
        interp_x = (1 - t) * startPoint.x + t * point.x
        interp_y = (1 - t) * startPoint.y + t * point.y
        interp_z = (1 - t) * startPoint.z + t * point.z

        interp_point = Point3D(interp_x, interp_y, interp_z)
        
        # Move leg to the interpolated position
        if write_leg(leg, interp_point) != 0:
            #print(f"Interpolation step {i} failed at ({interp_point.x}, {interp_point.y}, {interp_point.z})")
            #print(f"Associated servo numbers: {leg.servoS}, {leg.servoE}, {leg.servoW}")
            return 1  # Stop if movement fails
        # Use I2C to move servos here
        with I2CLock:
            #print(f"Moving servo {leg.servoS}, to {leg.angleS}")
            pwm = angle_to_pwm(leg.angleS)
            pca.channels[leg.servoS].duty_cycle = pwm
            #print(f"Moving servo {leg.servoE}, to {leg.angleE}")
            pwm = angle_to_pwm(leg.angleE)
            pca.channels[leg.servoE].duty_cycle = pwm
            #print(f"Moving servo {leg.servoW}, to {leg.angleW}")
            pwm = angle_to_pwm(leg.angleW)
            pca.channels[leg.servoW].duty_cycle = pwm

    return 0
# Bezier curve function to calculate the interpolated position at time t
def bezier_curve(t, p0, p1, p2, p3):
    # Calculate the cubic Bézier curve position
    x = (1 - t)**3 * p0.x + 3 * (1 - t)**2 * t * p1.x + 3 * (1 - t) * t**2 * p2.x + t**3 * p3.x
    y = (1 - t)**3 * p0.y + 3 * (1 - t)**2 * t * p1.y + 3 * (1 - t) * t**2 * p2.y + t**3 * p3.y
    z = (1 - t)**3 * p0.z + 3 * (1 - t)**2 * t * p1.z + 3 * (1 - t) * t**2 * p2.z + t**3 * p3.z
    return Point3D(x, y, z)

def move_leg_curve(leg, control1, control2, control3, steps = 360, phaseOffset = 0):
    for i in range(0, steps):
        phase = math.fmod((i + phaseOffset) / steps, 1)
        setPoint = bezier_curve(phase, leg.currentPos, control1, control2, control3)
        if write_leg(leg, setPoint) != 0:
            print(f"Unable to reach point: {setPoint.x}, {setPoint.y}, {setPoint.z}!")
            return
         # Use I2C to move servos here
        with I2CLock:
            print(f"Moving servo {leg.servoS}, to {leg.angleS}")
            """ 
            pwm = angle_to_pwm(leg.angleS)
            pca.channels[leg.servoS].duty_cycle = pwm
            """
            print(f"Moving servo {leg.servoE}, to {leg.angleE}")
            """ 
            pwm = angle_to_pwm(leg.angleE)
            pca.channels[leg.servoE].duty_cycle = pwm
            """
            print(f"Moving servo {leg.servoW}, to {leg.angleW}")
            """ 
            pwm = angle_to_pwm(leg.angleW)
            pca.channels[leg.servoW].duty_cycle = pwm
            """
startingHeight = 3
# Initialize legs
legs = [
    Leg(Point3D(-4.317, 4.317, startingHeight), 0, 1, 2, 135), #Front left
    Leg(Point3D(-6.106, 0, startingHeight), 3, 4, 5, 180), #Center left
    Leg(Point3D(-4.317, -4.317, startingHeight), 6, 7, 8, -135), #Back left
    Leg(Point3D(4.317, -4.317, startingHeight), 9, 10, 11, -45), #Back right
    Leg(Point3D(6.106, 0, startingHeight), 12, 13, 14, 0), #Center right
    Leg(Point3D(4.317, 4.317, startingHeight), 15, 16, 17, 45) #Front right
]
# Leg Aliases for readability
FL = legs[0]
CL = legs[1]
BL = legs[2]
BR = legs[3]
CR = legs[4]
FR = legs[5]
# Home positions for each leg
FLHome = Point3D(-14, 14, 0)
CLHome = Point3D(-20, 0, 0)
BLHome = Point3D(-14, -14, 0)
BRHome = Point3D(14, -14, 0)
CRHome = Point3D(20, 0, 0)
FRHome = Point3D(14, 14, 0)
# Communication loop
# Wait for controller to connect
# Main function to handle waiting for a controller to be connected
wait_for_controller()
ControlThread = threading.Thread(target=read_dualshock4_input)
ControlThread.daemon = True
ControlThread.start()
#Stand up sequence
FL.currentPos = Point3D(FLHome.x, FLHome.y, startingHeight)
CL.currentPos = Point3D(CLHome.x, CLHome.y, startingHeight)
BL.currentPos = Point3D(BLHome.x, BLHome.y, startingHeight)
FR.currentPos = Point3D(FRHome.x, FRHome.y, startingHeight)
CR.currentPos = Point3D(CRHome.x, CRHome.y, startingHeight)
BR.currentPos = Point3D(BRHome.x, BRHome.y, startingHeight)
#Initialize thread for synchronous movement
TFL = threading.Thread(target=move_leg, args=(FL, FL.currentPos))
TCL = threading.Thread(target=move_leg, args=(CL, CL.currentPos))
TBL = threading.Thread(target=move_leg, args=(BL, BL.currentPos))
TFR = threading.Thread(target=move_leg, args=(FR, FR.currentPos))
TCR = threading.Thread(target=move_leg, args=(CR, CR.currentPos))
TBR = threading.Thread(target=move_leg, args=(BR, BR.currentPos))
#Set 1 stand
TFL.start()
TCR.start()
TBL.start()

#Set 2 stand
TFR.start()
TCL.start()
TBR.start()

#Thread lock
TFL.join()
TCL.join()
TBL.join()
TFR.join()
TCR.join()
TBR.join()

move_leg(FL, FL.currentPos)
move_leg(CL, CL.currentPos)
move_leg(BL, BL.currentPos)
move_leg(FR, FR.currentPos)
move_leg(CR, CR.currentPos)
move_leg(BR, BR.currentPos)

move_leg(FL, FLHome)
move_leg(CL, CLHome)
move_leg(BL, BLHome)
move_leg(BR, BRHome)
move_leg(CR, CRHome)
move_leg(FR, FRHome)
# This is a filler number, will need to calculate later
# Center of the stride based on the CR leg, every leg should stride approximately this distance from the origin
strideCenter = 17
strideRadius = 4

# Given distance, as a proportion of radius and angle for given stride return control point tupple
def completeStrideCalculation(leg, distance, angle, angleOffset):
    centerPoint = Point3D(math.cos(math.radians(angleOffset)) * strideCenter, math.sin(math.radians(angleOffset)) * strideCenter, startingHeight / 2)
    reachPoint = Point3D(centerPoint.x + (distance * math.cos(math.radians(angle)) * strideRadius), centerPoint.y + (distance * math.sin(math.radians(angle)) * strideRadius), 0)
    pullPoint = Point3D(centerPoint.x - (distance * math.cos(math.radians(angle)) * strideRadius), centerPoint.y - (distance * math.sin(math.radians(angle)) * strideRadius), 0)
    return(reachPoint, pullPoint, centerPoint)

def completeHeadingCalculation(leg, distance, strafe, heading, offset):
    SP = completeStrideCalculation(leg, distance, strafe, offset)
    HP = completeStrideCalculation(leg, distance, heading, offset)
    path = [None] * 3  # Initialize list with 3 None values
    path[0] = Point3D((SP[0].x + HP[0].x) / 2, (SP[0].y + HP[0].y) / 2, (SP[0].z + HP[0].z) / 2)
    path[1] = Point3D((SP[1].x + HP[1].x) / 2, (SP[1].y + HP[1].y) / 2, (SP[1].z + HP[1].z) / 2)
    path[2] = SP[2]  # Keep the center point from SP
    return path


# Main loop - your other robot code can go here
try:
    while True: # will run once every time a stride is completed
        # Filler value, need to figure out when I have the controller connected
        thisDistance = 0
        # Filler value, need to connect controller, might be +/- 90 out of phase
        thisStrafe = 0
        # Filler value, same as before
        thisHeading = 0
    
        if is_controller_connected() == False:
            print("Lost controller connection!")
            thisDistance = 0
            thisStrafe = 0
            thisHeading = 0
            wait_for_controller()
            continue
        if thisDistance == 0:
            TFL = threading.Thread(target=move_leg, args=(FL, FLHome))
            TCL = threading.Thread(target=move_leg, args=(CL, CLHome))
            TBL = threading.Thread(target=move_leg, args=(BL, BLHome))
            TFR = threading.Thread(target=move_leg, args=(FR, FRHome))
            TCR = threading.Thread(target=move_leg, args=(CR, CRHome))
            TBR = threading.Thread(target=move_leg, args=(BR, BRHome))
            time.sleep(0.1)
            continue
        #Legs move in delta formations
        if TRIPOD_GAIT:
            #Define behaviour for each leg
            PFL = completeHeadingCalculation(FL, thisDistance, thisStrafe, thisHeading, FL.Soffest)
            PFR = completeHeadingCalculation(FL, thisDistance, thisStrafe, math.fmod(thisHeading + 180, 360), FR.Soffest)
            PCL = completeHeadingCalculation(FL, thisDistance, thisStrafe, thisHeading, CL.Soffest)
            PCR = completeHeadingCalculation(FL, thisDistance, thisStrafe, math.fmod(thisHeading + 180, 360), CR.Soffest)
            PBL = completeHeadingCalculation(FL, thisDistance, thisStrafe, thisHeading, BL.Soffest)
            PBR = completeHeadingCalculation(FL, thisDistance, thisStrafe, math.fmod(thisHeading + 180, 360), BR.Soffest)

            TFL = threading.Thread(target=move_leg_curve, args=(FL, PFL[0], PFL[1], PFL[2], 360, 0))
            TFR = threading.Thread(target=move_leg_curve, args=(FR, PFR[0], PFR[1], PFR[2], 360, 180))
            TCL = threading.Thread(target=move_leg_curve, args=(FL, PCL[0], PCL[1], PCL[2], 360, 0))
            TCR = threading.Thread(target=move_leg_curve, args=(FR, PCR[0], PCR[1], PCR[2], 360, 180))
            TBL = threading.Thread(target=move_leg_curve, args=(FL, PBL[0], PBL[1], PBL[2], 360, 0))
            TBR = threading.Thread(target=move_leg_curve, args=(FR, PBR[0], PBR[1], PBR[2], 360, 180))
        #Legs move one after the other
        if WAVE_GAIT:
            print("UNDEFINED!")
            break

        #Legs move in bounding strides
        if BOUNDING_GAIT:
            print("UNDEFINED!")
            break
        
        # Move the legs
        TFL.start()
        TFR.start()
        TCL.start()
        TCR.start()
        TBL.start()
        TBR.start()

        # Wait until all legs are returned
        TFL.join()
        TFR.join()
        TCL.join()
        TCR.join()
        TBL.join()
        TBR.join()


    ControlThread.join()

        

except KeyboardInterrupt:
    print("Program interrupted, shutting down.")
