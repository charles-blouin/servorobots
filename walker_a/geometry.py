import pybullet as p


def create_robot():
    robot_urdf = p.loadURDF('walker_a/urdf/walker_a.urdf', basePosition=[0, 0, 1],
                            flags=p.URDF_USE_INERTIA_FROM_FILE & p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
# <mesh filename="body.obj" scale="0 0 0"/>
    return robot_urdf
