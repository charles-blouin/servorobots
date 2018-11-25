import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))

import pybullet as p
import pybullet_data
import time

#class DCMotor:

class MotorTest:
    def __init__(self):
        p.connect(p.GUI)
        self.timeStep = 0.01
        self.gravity = -9.8

        self.frictionId = p.addUserDebugParameter("jointFriction", 0, 20, 10)
        self.torqueId = p.addUserDebugParameter("joint torque", 0, 20, 5)

    def step(self, action):
        frictionForce = p.readUserDebugParameter(self.frictionId)
        jointTorque = p.readUserDebugParameter(self.torqueId)
        print(frictionForce)
        # set the joint friction
        p.setJointMotorControl2(self.wheel, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
        # apply a joint torque
        p.setJointMotorControl2(self.wheel, 1, p.TORQUE_CONTROL, force=jointTorque)

        p.stepSimulation()

    def reset(self):

        p.resetSimulation()

        # Loading the model and initial pose
        wheel_dir = os.path.join(currentdir, "motor_test_01.urdf")
        self.wheel = p.loadURDF(wheel_dir)

        p.changeDynamics(self.wheel, -1, linearDamping=0, angularDamping=0)
        for j in range(p.getNumJoints(self.wheel)):
            p.changeDynamics(self.wheel, j, linearDamping=0, angularDamping=0)
            print(p.getJointInfo(self.wheel, j))

        visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1],
                                            specularColor=[0.4, .4, 0])
        self.desired_pos_sphere = p.createMultiBody(baseMass=0, baseInertialFramePosition=[0, 0, 0.2],
                                                   baseVisualShapeIndex=visualShapeId)

        p.setGravity(0,0, self.gravity)
        p.setTimeStep(self.timeStep)
        p.setRealTimeSimulation(0)

        filename = os.path.join(pybullet_data.getDataPath(), "plane_stadium.sdf")
        # self.ground_plane_mjcf = p.loadSDF(filename)



def main():
    print("Hi!")

if __name__ == '__main__':
    main()
    m = MotorTest()
    m.reset()

    while 1:
        time.sleep(0.01)
        m.step(1)
