import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)
import time
import pybullet as p
import pybullet_data

p.connect(p.GUI)
timeStep = 0.01
p.setRealTimeSimulation(0)
p.setTimeStep(timeStep)

p.resetSimulation()
cube0 = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"r2d2.urdf"),[0,0,0])
cube1 = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"r2d2.urdf"),[2,0,0], [0, 0.7071, 0, 0.7071])
cube2 = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"r2d2.urdf"),[0,2,0])
cube3 = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"r2d2.urdf"),[2,2,0], [0, 0.7071, 0, 0.7071])


while 1:
    p.stepSimulation()
    time.sleep(timeStep)
    p.applyExternalTorque(cube0, -1, [1, 0, 0], p.LINK_FRAME)
    p.applyExternalTorque(cube1, -1, [1, 0, 0], p.LINK_FRAME)
    p.applyExternalTorque(cube2, -1, [1, 0, 0], p.WORLD_FRAME)
    p.applyExternalTorque(cube3, -1, [1, 0, 0], p.WORLD_FRAME)