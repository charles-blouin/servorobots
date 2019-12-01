import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))

import pybullet as p
import pybullet_data
import time
import collections

class TimestampInput:
    def __init__(self, i, t):
        self.i = i
        self.t = t


class GearedDcMotor:
    def __init__(self, R, Kv, K_viscous, K_load, timestep, V_max = 6, latency = 0):
        # Motor internal resistance
        self.R = R
        self.Kv = Kv
        self.K_viscous = K_viscous
        self.K_load = K_load
        self.timestep = timestep
        self.V_max = V_max

        # The voltage the motor is currently receiving
        self.applied_v = TimestampInput(0, -100)
        self.v_buffer = collections.deque()
        if latency < 0:
            raise ValueError("Latency < 0. You can't see the future")
        self.latency = latency

        #
        self.active_backlash = False
        # Backlash in rad
        self.backlash = 1/360*6.3
        self.pos_backlash = 0
        self.vel_motor = 0
        # Backlash low speed threshold (backlash will be deactivated above this speed for numerical stability)
        self.backlash_threshold = 10


    def torque_from_voltage(self, v_in, omega):
        # This section handles latency
        motor_time = v_in.t - self.latency
        if (len(self.v_buffer) > 0) and (v_in.t < self.v_buffer[-1].t):
            raise ValueError('Input Voltage timestamp is lower than stored timestamp')
        else:
            self.v_buffer.append(v_in)

        while 1:
            # Check if the queue is empty
            if len(self.v_buffer) == 0:
                break
            # Check if leftmost item should be used as new voltage
            if self.v_buffer[0].t <= motor_time:
                self.applied_v = self.v_buffer.popleft()
            else:
                break

        Vin = min(max(self.applied_v.i, -self.V_max), self.V_max)

        # This section handles backlash

        # varies between +- backlash/2

        # When accelerating or decelerating with no backlash.

        # print('Start: backlash' + str(self.pos_backlash))
        if self.active_backlash:
            if self.pos_backlash >= self.backlash/2 and (self.vel_motor >= omega):
                # print('No backlash, motor going forward')
                self.pos_backlash = self.backlash/2
                self.vel_motor = omega
            elif self.pos_backlash <= -self.backlash/2 and (self.vel_motor <= omega):
                # print('No backlash, motor going backward')
                self.pos_backlash = -self.backlash/2
                self.vel_motor = omega
            else:
                print('Backlash!')
        else:
            self.vel_motor = omega


        # Torque output of the motor before gear
        # Nm = V/Ohm / (rad/s/V) = I * V * s = W * s = Nm/s * s
        torque_raw = (Vin - self.vel_motor / self.Kv) / self.R / self.Kv

        # Kt = Kv
        # Current = Torque * Kt = Torque * Kv
        # A = Nm * (rad/s/V) = W*s / V / s = V * A * s / V / s = A
        current = torque_raw * self.Kv

        torque_friction = self.vel_motor * self.K_viscous

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

        # We update the velocity and position of the motor. If we are outside of the backlash, \
        # the position will be reset next loop.
        if self.active_backlash:
            self.vel_motor = self.vel_motor + torque_out/0.000005 * self.timestep
            # self.vel_motor = self.Kv * Vin
            self.pos_backlash = self.timestep * (self.vel_motor - omega) + self.pos_backlash

            if abs(self.pos_backlash) <= self.backlash/2:
                torque_out = 0
                print('Torque out:' + str(torque_out))

        # Motor current
        return torque_out, current

class MotorTest:
    def __init__(self):
        p.connect(p.GUI)
        self.timeStep = 0.01
        self.gravity = -9.8
        self.time = 0

        self.voltageId = p.addUserDebugParameter("Input Voltage", -5, 5, 0)
        self.frictionId = p.addUserDebugParameter("jointFriction", 0, 0.1, 0)
        self.torqueId = p.addUserDebugParameter("joint torque", 0, 20, 5)

        self.motor_1 = GearedDcMotor(R=4, Kv=12, K_viscous=0.000122489, K_load=0, timestep = self.timeStep, latency=0)

        self.display_joint_torque = p.addUserDebugText('NA', [1.5, 0, 0.1], textColorRGB=[0, 0, 0],
                                                  textSize=1)
        self.display_joint_current = p.addUserDebugText('NA', [1.5, 0, 0.3], textColorRGB=[0, 0, 0],
                                                       textSize=1)
        self.display_time = p.addUserDebugText('NA', [1.5, 0, 0.3], textColorRGB=[0, 0, 0],
                                                        textSize=1)

    def step(self, action):
        self.time = self.time + self.timeStep

        applied_Voltage = p.readUserDebugParameter(self.voltageId)
        frictionForce = p.readUserDebugParameter(self.frictionId)
        jointTorque = p.readUserDebugParameter(self.torqueId)

        # Calculate motor output torque and current
        pos, vel, _, _ = p.getJointState(self.wheel, 1)
        motor_torque, motor_current = self.motor_1.torque_from_voltage(TimestampInput(applied_Voltage, self.time), vel)

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


class CharacterizeMotor:
    def __init__(self, max_motor_in = 400, frame_length = 0.02, GUI = 0):
        if GUI:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        self.frame_length = frame_length
        self.gravity = -9.8
        self.time = 0
        self.max_motor_in = max_motor_in

        self.motor_1 = GearedDcMotor(R=22, Kv=8, K_viscous=0.0005, K_load=0, timestep = self.frame_length, latency=0.02)


    def step(self, action):
        self.time = self.time + self.frame_length

        base_voltage = action[0]
        command_in = action[1]
        frictionForce = 0

        applied_Voltage = command_in * base_voltage / self.max_motor_in

        # Calculate motor output torque and current
        pos, vel, _, _ = p.getJointState(self.wheel, 1)
        motor_torque, motor_current = self.motor_1.torque_from_voltage(TimestampInput(applied_Voltage, self.time), vel)

        # apply a joint torque from motor
        p.setJointMotorControl2(self.wheel, 1, p.TORQUE_CONTROL, force=motor_torque)

        # set the joint friction
        p.setJointMotorControl2(self.wheel, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)

        if self.frame_length == 0.01:
            p.stepSimulation()
        elif self.frame_length == 0.02:
            p.stepSimulation()
            p.stepSimulation()
        else:
            raise Exception('Choose a valid timestep or fix the code logic to step in the sim!')
        pos, vel, _, _ = p.getJointState(self.wheel, 1)

        return pos, vel


    def reset(self, R=22, Kv=8):

        p.resetSimulation()

        self.motor_1 = GearedDcMotor(R=R, Kv=Kv, K_viscous=0.0005, K_load=0, timestep=self.frame_length, latency=0.02)

        # Loading the model and initial pose
        wheel_dir = os.path.join(currentdir, "motor_test_01.urdf")
        self.wheel = p.loadURDF(wheel_dir, flags=p.URDF_USE_INERTIA_FROM_FILE)

        p.changeDynamics(self.wheel, -1, linearDamping=0, angularDamping=0)
        for j in range(p.getNumJoints(self.wheel)):
            p.changeDynamics(self.wheel, j, linearDamping=0, angularDamping=0)

        p.setGravity(0,0, self.gravity)
        p.setTimeStep(0.01)
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
