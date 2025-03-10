import math

# Mechanical leg segments measured from axis to axis
L1 = 6.0
L2 = 4.0
L3 = 6.0

# Mechanical joint limits
S_LIM = (-90, 90)
E_LIM = (-90, 90)
W_LIM = (-90, 0)

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

    def write_leg(self, point):
        try:
            #Find dx, dy, and dz based on initial offset
            dx = point.x - self.offset.x
            dy = point.y - self.offset.y
            dz = point.z - self.offset.z
            
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
            finalS = math.degrees(s) - self.Soffest
            finalE = math.degrees(e)
            finalW = math.degrees(w)

            # Boundary check the leg angles
            if not (S_LIM[0] <= finalS <= S_LIM[1]):
                print("Invalid S angle!")
                print(finalS)
                return 1
            if not (E_LIM[0] <= finalE <= E_LIM[1]):
                print("Invalid E angle!")
                print(finalE)
                return 1
            if not (W_LIM[0] <= finalW <= W_LIM[1]):
                print("Invalid W angle!")
                print(finalW)
                return 1
            
            # Write new angles and point
            self.angleS = finalS
            self.angleE = finalE
            self.angleW = finalW

            # If error accumulates, use DEBUG to make controlled movements
            if DEBUG:
                endPoint = Point3D(L3 * math.cos(e) * math.cos(s) * math.cos(w) - L3 * math.sin(e) * math.cos(s) * math.sin(w) 
                                    + L2 * math.cos(e) * math.sin(s)
                                    + L1 * math.cos(s) + self.offset.x,
                                L3 * math.cos(e) * math.sin(s) * math.cos(w) - L3 * math.sin(e) * math.sin(s) * math.sin(w) 
                                    + L2 * math.cos(e) * math.sin(s)
                                    + L1 * math.sin(s) + self.offset.y,
                                L3 * math.sin(e) * math.cos(w) + L3 * math.cos(e) * math.sin(w)
                                    + L2 * math.sin(e) + self.offset.z)
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

    def move_leg(self, point, steps = 10):
        if self.write_leg(point) != 0:
            #If leg cannot go to position
            print("Error in writeLeg")
            return 1

        startPoint = self.currentPos
        for i in steps:
            
            t = i / steps  # Normalized interpolation factor (0 to 1)

            # Compute interpolated point
            interp_x = (1 - t) * startPoint.x + t * point.x
            interp_y = (1 - t) * startPoint.y + t * point.y
            interp_z = (1 - t) * startPoint.z + t * point.z

            interp_point = Point3D(interp_x, interp_y, interp_z)
            
            # Move leg to the interpolated position
            if self.write_leg(interp_point) != 0:
                print(f"Interpolation step {i} failed at {interp_point}")
                return 1  # Stop if movement fails
            # Use I2C to move servos here

        return 0
startingHeight = -3
# Initialize legs
legs = [
    Leg(Point3D(-4.317, 4.317, startingHeight), 0, 1, 2, 135), #Front left
    Leg(Point3D(-6.106, 0, startingHeight), 3, 4, 5, 180), #Center left
    Leg(Point3D(-4.317, -4.317, startingHeight), 6, 7, 8, 225), #Back left
    Leg(Point3D(4.317, -4.317, startingHeight), 9, 10, 11, 45), #Back right
    Leg(Point3D(6.106, 0, startingHeight), 12, 13, 14, 0), #Center right
    Leg(Point3D(4.317, 4.317, startingHeight), 15, 16, 17, -45) #Front right
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
CLHome = Point3D(-19, 0, 0)
BLHome = Point3D(-14, -14, 0)
BRHome = Point3D(14, -14, 0)
CRHome = Point3D(19, 0, 0)
FRHome = Point3D(14, 14, 0)

FL.move_leg(FLHome)
CL.move_leg(CLHome)
BL.move_leg(BLHome)
BR.move_leg(BRHome)
CR.move_leg(CRHome)
FR.move_leg(FRHome)

# Initialize communication loop
Direction = 0
Speed = 0



# Build stride pattern
