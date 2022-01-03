import time
import cppimport.import_hook
import pylibflexiv

robot = pylibflexiv.Robot()
robot.init("192.168.2.100", "192.168.15.169")
robot.connect()

while not robot.isConnected():
    time.sleep(0.001)
print("Robot connected, reading state")
robot_state = pylibflexiv.RobotStates()
robot.getRobotStates(robot_state)
print(robot_state.m_q)


input("Robot will start some moving")
input("Be very careful! Put your hand on estop")
robot.enable()
while not robot.isOperational():
    time.sleep(0.001)

print("Robot is operational, going home")
robot.setMode(flexiv.Mode.MODE_PRIMITIVE_EXECUTION)
while robot.getMode() != flexiv.Mode.MODE_PRIMITIVE_EXECUTION:
    time.sleep(0.001)

robot.executePrimitive("Home()")

while True:
    time.sleep(0.001)
