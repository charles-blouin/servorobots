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
joint_X_0_loc = [str( 0.35*s) +", " + str(-0.3 *s) + ", 0",
                 str( 0.35*s) +", " + str( 0.3 *s) + ", 0",
                 str(-0.35*s) +", " + str( 0.3 *s) + ", 0",
                 str(-0.35*s) +", " + str(-0.3 *s) + ", 0"]
joint_X_0_rot = ["0, 0, 0", "0, 0, 0", "0, 0, 3.14159", "0, 0, 3.14159"]
leg_X_0_inertial = str(0.05*s) + ", 0, 0"
joint_X_1_loc = str(0.15*s) + ", 0, 0"
leg_X_1_inertial = "0, 0, " + str(-0.21*s)
joint_X_2_loc = "0, 0, " + str(-0.50*s)
leg_X_2_inertial = "0, 0, " + str(-0.21*s)


inertia_matrix_body = [x * s * s for x in inertia_matrix_body]
inertia_arm_0 = [x * s * s for x in inertia_arm_0]
inertia_arm_1 = [x * s * s for x in inertia_arm_1]


inertia_body = assign_inertia(inertia_matrix_body)
inertia_arm_0 = assign_inertia(inertia_arm_0)
inertia_arm_1 = assign_inertia(inertia_arm_1)

base_link = Link("base_link", contact,
                 Inertial(inertia_body, Mass(mass_body)),
                 Visual(Geometry(Mesh(filename="body.obj", scale=f"{s} {s} {s}"))),
                 Collision(Geometry(Mesh(filename="body.obj", scale=f"{s} {s} {s}")))
                 )

link_X_0 = []
for i in range(4):
    link_X_0.append(Link("link_" + str(i) + "_0", contact,
                    Inertial(inertia_arm_0, Mass(mass_arm_0), Origin(leg_X_0_inertial)),
                    Visual(Geometry(Mesh(filename="leg_X_0.obj", scale=f"{s} {s} {s}"))),
                    Collision(Geometry(Mesh(filename="leg_X_0.obj", scale=f"{s} {s} {s}")))
                    ))
link_X_1 = []
for i in range(4):
    link_X_1.append(Link(f"link_{i}_1", contact,
                    Inertial(inertia_arm_0, Mass(mass_arm_0), Origin(leg_X_1_inertial)),
                    Visual(Geometry(Mesh(filename="leg_X_1.obj", scale=f"{s} {s} {s}"))),
                    Collision(Geometry(Mesh(filename="leg_X_1.obj", scale=f"{s} {s} {s}")))
                    ))
link_X_2 = []
for i in range(4):
    link_X_2.append(Link(f"link_{i}_2", contact,
                    Inertial(inertia_arm_0, Mass(mass_arm_0), Origin(leg_X_1_inertial)),
                    Visual(Geometry(Mesh(filename="leg_X_1.obj", scale=f"{s} {s} {s}"))),
                    Collision(Geometry(Mesh(filename="leg_X_1.obj", scale=f"{s} {s} {s}")))
                    ))

#Add first elements to robot

my_robot(base_link,
         link_X_0[0], link_X_1[0], link_X_2[0],
         link_X_0[1], link_X_1[1], link_X_2[1],
         link_X_0[2], link_X_1[2], link_X_2[2],
         link_X_0[3], link_X_1[3], link_X_2[3])

joint_X_0 = []
for i in range(4):
    joint_X_0.append(Joint(Parent("base_link"), Child("link_" + str(i) + "_0"),
                           Origin(xyz=joint_X_0_loc[i], rpy=joint_X_0_rot[i]),
                           Axis("1, 0, 0"), type="continuous", name=f"joint_{i}_0"))
joint_X_1 = []
for i in range(4):
    joint_X_1.append(Joint(Parent("link_{}_0".format(i)), Child(f"link_{i}_1"), Origin(xyz=joint_X_1_loc),
                           Axis("0, 1, 0"), type="continuous", name=f"joint_{i}_1"))
joint_X_2 = []
for i in range(4):
    joint_X_2.append(Joint(Parent("link_{}_1".format(i)), Child(f"link_{i}_2"), Origin(xyz=joint_X_2_loc),
                           Axis("0, 1, 0"), type="continuous", name=f"joint_{i}_2"))

my_robot(joint_X_0[0], joint_X_1[0], joint_X_2[0],
         joint_X_0[1], joint_X_1[1], joint_X_2[1],
         joint_X_0[2], joint_X_1[2], joint_X_2[2],
         joint_X_0[3], joint_X_1[3], joint_X_2[3],)
f = open("walker_a/urdf/walker_a_0_5.urdf", "w")
f.write(str(my_robot))
f.close()