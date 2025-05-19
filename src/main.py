import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # load plane
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot_id = p.loadURDF("../urdf/rrr_robot.urdf", useFixedBase=True)

# Set all joints to 0 radians
for i in range(p.getNumJoints(robot_id)):
    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=0)

while True:
    p.stepSimulation()
    time.sleep(1./240.)
