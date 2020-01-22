from odio_urdf import *

def assign_inertia(im):
    return Inertia(ixx=im[0], ixy=im[1], ixz=im[2], iyy=im[3], iyz=im[4], izz=im[5])

my_robot = Robot("walker_a")
contact = Contact(Lateral_Friction("100"))

s = 1
inertia_matrix_body = [0.6363636364, 4.908571429, 4.51012987, 4.51012987, 0.6363636364, 4.908571429]
inertia_arm_0 = [3.11E-04, 0.003766478343, 0.007532956685, 0.007532956685, 2.25E-03, 0.003766478343]
inertia_arm_1 = [0.4103896104, 0.003291744093, 0.04189009822, 0.04189009822, 0.04194319087, 0.003291744093]
mass_body = str(40 * s * s * s)
mass_arm_0 = str(2 * s * s * s)
mass_arm_1 = str(2 * s * s * s)
joint_X_0_loc = str(0.35*s) +", " + str(-0.3 *s) + ", 0"
leg_X_0_inertial = "0.05, 0, 0"
leg_X_1_loc = "0.15, 0, 0"
leg_X_1_inertial = "0, 0, -0.21"
leg_X_2_loc = "0, 0, -0.50"
leg_X_2_inertial = "0, 0, -0.21"


inertia_matrix_body = [x * s * s for x in inertia_matrix_body]
inertia_arm_0 = [x * s * s for x in inertia_arm_0]
inertia_arm_1 = [x * s * s for x in inertia_arm_1]


inertia_body = assign_inertia(inertia_matrix_body)
inertia_arm_0 = assign_inertia(inertia_arm_0)
inertia_arm_1 = assign_inertia(inertia_arm_1)

base_link = Link("base_link", contact,
                 Inertial(inertia_body, Mass(mass_body)),
                 Visual(Geometry(Mesh(filename="body.obj"))),
                 Collision(Geometry(Mesh(filename="body.obj")))
                 )
link_0_0 = Link("link_0_0", contact,
                Inertial(inertia_arm_0, Mass(mass_arm_0), Origin(joint_X_0_loc)),
                Visual(Geometry(Mesh(filename="leg_X_0.obj"))),
                Collision(Geometry(Mesh(filename="leg_X_0.obj")))
                )

#Add first elements to robot

my_robot(base_link,link_0_0)

base = Parent("base_link")
joint1 = Joint(base, Child("link_0_0"), type="continuous")

my_robot(joint1)

print(my_robot)