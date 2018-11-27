import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))

import pybullet as p
import pybullet_data
import time

class GearedDcMotor:
    def __init__(self, R, Kv, K_viscous, K_load):
        # Motor internal resistance
        self.R = R
        self.Kv = Kv
        self.K_viscous = K_viscous
        self.K_load = K_load

    def torque_from_voltage(self, Vin, omega):

        # Torque output of the motor before gear
        # Nm = V/Ohm / (rad/s/V) = I * V * s = W * s = Nm/s * s
        torque_raw = (Vin - omega / self.Kv) / self.R / self.Kv

        # Kt = Kv
        # Current = Torque * Kt = Torque * Kv
        # A = Nm * (rad/s/V) = W*s / V / s = V * A * s / V / s = A
        current = torque_raw * self.Kv

        torque_friction = omega * self.K_viscous

        torque_out = (torque_raw - torque_friction) / (1 + self.K_load)

        # To find K_viscous, run the motor with the gearbox attached and not other load.
        # Note the motor speed and the current. Then, calculate Kv with :
        # Kv = omega / (Vin - current * R) <==> current * R = Vin - omega/ Kv
        # With no load, torque_raw = torque_viscous. Calculate torque_raw with
        # torque_raw = current / Kv
        # Then, calculate K_viscous with
        # K_viscous = torque_raw/omega



        # To find K_load when the motor is pulling a known load torque_out.
        # torque_out = torque_raw - torque_friction - torque_out * K_load
        # torque_out (1 + K_load) =  torque_raw - torque_friction
        # torque_out = (torque_raw - torque_friction) / (1 + K_load) < True in general for this model
        # K_load = (torque_raw - torque_friction)/torque_out - 1
        # K_load = ((Vin - omega / self.Kv) / self.R / self.Kv) - omega * self.K_viscous)/torque_out - 1

        print(torque_out)
        # Motor current
        return torque_out, current

class MotorTest:
    def __init__(self):
        p.connect(p.GUI)
        self.timeStep = 0.01
        self.gravity = -9.8
        self.time = 0

        self.voltageId = p.addUserDebugParameter("Input Voltage", 0, 20, 6)
        self.frictionId = p.addUserDebugParameter("jointFriction", 0, 20, 0)
        self.torqueId = p.addUserDebugParameter("joint torque", 0, 20, 5)

        self.motor_1 = GearedDcMotor(R=4, Kv=12, K_viscous=0.000122489, K_load=0)

        self.display_joint_torque = p.addUserDebugText('NA', [1.5, 0, 0.1], textColorRGB=[0, 0, 0],
                                                  textSize=1)
        self.display_joint_current = p.addUserDebugText('NA', [1.5, 0, 0.3], textColorRGB=[0, 0, 0],
                                                       textSize=1)
        self.display_time = p.addUserDebugText('NA', [1.5, 0, 0.3], textColorRGB=[0, 0, 0],
                                                        textSize=1)

    def step(self, action):
        applied_Voltage = p.readUserDebugParameter(self.voltageId)
        frictionForce = p.readUserDebugParameter(self.frictionId)
        jointTorque = p.readUserDebugParameter(self.torqueId)

        # Calculate motor output torque and current
        pos, vel, _, _ = p.getJointState(self.wheel, 1)
        motor_torque, motor_current = self.motor_1.torque_from_voltage(applied_Voltage, vel)

        # apply a joint torque from motor
        p.setJointMotorControl2(self.wheel, 1, p.TORQUE_CONTROL, force=motor_torque)

        # set the joint friction
        p.setJointMotorControl2(self.wheel, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)


        # Display
        p.addUserDebugText("Motor Torque: " + str(motor_torque), [1.2, 0, 0.1], textColorRGB=[0, 0, 0],
                                                  textSize=1, replaceItemUniqueId=self.display_joint_torque)
        p.addUserDebugText("Motor Current: " + str(motor_current), [1.2, 0, 0.3], textColorRGB=[0, 0, 0],
                                                  textSize=1, replaceItemUniqueId=self.display_joint_current)
        p.addUserDebugText("Time: " + str(self.time), [1.2, 0, 0.6], textColorRGB=[0, 0, 0],
                           textSize=1, replaceItemUniqueId=self.display_time)
        p.stepSimulation()
        self.time = self.time + self.timeStep

    def reset(self):

        p.resetSimulation()

        # Loading the model and initial pose
        wheel_dir = os.path.join(currentdir, "motor_test_01.urdf")
        self.wheel = p.loadURDF(wheel_dir, flags=p.URDF_USE_INERTIA_FROM_FILE)

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
        time.sleep(m.timeStep)
        m.step(1)
