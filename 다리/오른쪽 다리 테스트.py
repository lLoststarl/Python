from dynamixel_sdk import *  # Dynamixel SDK import
import math
import time

# Initialize PortHandler instance
portHandler = PortHandler('/dev/ttyUSB0')
DXL_ID = [1,2,]
# Initialize PacketHandler instance
packetHandler = PacketHandler(2.0)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 116, 4)  # Address 116 is for goal position, and data size is 4

# Dynamixel motor IDs
HIP_L = DXL_ID[0]
KNEE_L = DXL_ID[3]
HIP_R = DXL_ID[1]
KNEE_R = DXL_ID[2]

# Constants
l1 = 7  # Length of the first limb
l2 = 16  # Length of the second limb
stepHeight = 5
stepClearance = 3

def deg_to_dxl(deg):
    return int((deg) * (4096 / 360.0))  # Convert from degrees to Dynamixel units. Adjust this function as needed.

def update_motor_Pos(target1, target2, char_leg):
    if char_leg == 'l':
        hip_goal = deg_to_dxl(target1)
        knee_goal = deg_to_dxl(target2)
        groupSyncWrite.addParam(HIP_L, [DXL_LOBYTE(DXL_LOWORD(hip_goal)), DXL_HIBYTE(DXL_LOWORD(hip_goal)), DXL_LOBYTE(DXL_HIWORD(hip_goal)), DXL_HIBYTE(DXL_HIWORD(hip_goal))])
        groupSyncWrite.addParam(KNEE_L, [DXL_LOBYTE(DXL_LOWORD(knee_goal)), DXL_HIBYTE(DXL_LOWORD(knee_goal)), DXL_LOBYTE(DXL_HIWORD(knee_goal)), DXL_HIBYTE(DXL_HIWORD(knee_goal))])
    
    elif char_leg == 'r':
        hip_goal = deg_to_dxl(target1)
        knee_goal = deg_to_dxl(target2)
        groupSyncWrite.addParam(HIP_R, [DXL_LOBYTE(DXL_LOWORD(hip_goal)), DXL_HIBYTE(DXL_LOWORD(hip_goal)), DXL_LOBYTE(DXL_HIWORD(hip_goal)), DXL_HIBYTE(DXL_HIWORD(hip_goal))])
        groupSyncWrite.addParam(KNEE_R, [DXL_LOBYTE(DXL_LOWORD(knee_goal)), DXL_HIBYTE(DXL_LOWORD(knee_goal)), DXL_LOBYTE(DXL_HIWORD(knee_goal)), DXL_HIBYTE(DXL_HIWORD(knee_goal))])
    
    groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

def pos(x, z, leg):
    x = float(x)
    z = float(z)
    
    theta1 = math.atan(x/z)
    theta1Deg = math.degrees(theta1)
    
    alpha = math.sqrt(z**2 + x**2)
    theta2 = math.acos((l1**2 + alpha**2 - l2**2)/(2*l1*l2))
    theta2Deg = math.degrees(theta2)
     
    gamma1 = math.acos(( l1 ** 2 + l2 ** 2 - alpha ** 2) / (2 * l1 * l2))
    gamma1Deg = math.degrees(gamma1)
    print(gamma1Deg)
    theta = theta1Deg + theta2Deg
    update_motor_Pos(theta, gamma1Deg, leg)


def takeStep(stepLength, stepVelocity):
    for i in range(int(stepLength * 2), int(-stepLength * 2), -1):
        i /= 2.0
        pos(i, stepHeight, 'r')
        pos(-i, stepHeight - stepClearance, 'l')
        time.sleep(stepVelocity / 1000.0)

    for i in range(int(stepLength * 2), int(-stepLength * 2), -1):
        i /= 2.0
        pos(-i, stepHeight - stepClearance, 'r')
        pos(i, stepHeight, 'l')
        time.sleep(stepVelocity / 1000.0)

def initialize():
    for i in range(1070, int(stepHeight * 10), -1):
        i /= 100.0
        pos(0, i, 'l')
        pos(0, i, 'r')

if __name__ == '__main__':
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        exit()

    # Set port baudrate
    if portHandler.setBaudRate(57600):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        exit()

    # Enable Dynamixel Torque
    packetHandler.write1ByteTxRx(portHandler, HIP_R, 64, 1)
    packetHandler.write1ByteTxRx(portHandler, KNEE_R, 64, 1)

    # Initialize and start moving
    initialize()
    while True:
        takeStep(2, 0)
