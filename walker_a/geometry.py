import pybullet as p


def create_robot():
    mass = 1
    s = 1  # Scale
    wheel_base = 0.5 * s
    body_width = 0.15 * s
    elbow_width = 0.05 * s
    elbow_length = 0.075 * s
    leg_width = 0.05 * s
    leg_length = 0.25 * s
    body_height = 0.1 * s
    baseOrientation = [0, 0, 0, 1]
    basePosition = [0, 0, 1 * s]
    sphereRadius = 0.05
    ### Main Body ###
    col_body = p.createCollisionShape(p.GEOM_BOX,
                                      halfExtents=[wheel_base, body_width, body_height])
    vis_body = p.createVisualShape(p.GEOM_BOX,
                                   halfExtents=[wheel_base, body_width, body_height])
    ### Elbow ###
    col_elbow = p.createCollisionShape(p.GEOM_BOX,
                                       halfExtents=[elbow_length, elbow_width, elbow_width],
                                       collisionFramePosition=[elbow_length / 2, 0, 0])
    vis_elbow = p.createVisualShape(p.GEOM_BOX,
                                    halfExtents=[elbow_length, elbow_width, elbow_width],
                                    visualFramePosition=[elbow_length / 2, 0, 0])
    ### Upper Leg ###
    col_upper_leg = p.createCollisionShape(p.GEOM_BOX,
                                           halfExtents=[leg_width, leg_width, leg_length],
                                           collisionFramePosition=[0, 0, -leg_length])
    vis_upper_leg = p.createVisualShape(p.GEOM_BOX,
                                        halfExtents=[leg_width, leg_width, leg_length],
                                        visualFramePosition=[0, 0, -leg_length])

    ### Lower Leg ###
    col_lower_leg = p.createCollisionShape(p.GEOM_BOX,
                                           halfExtents=[leg_width, leg_width, leg_length],
                                           collisionFramePosition=[0, 0, -leg_length])
    vis_lower_leg = p.createVisualShape(p.GEOM_BOX,
                                        halfExtents=[leg_width, leg_width, leg_length],
                                        visualFramePosition=[0, 0, -leg_length])
    # [leg1 elbow, leg1 upper, leg1 lower, leg2 elbow ... leg4 lower]
    link_Masses = [1, 1, 1,
                   1, 1, 1,
                   1, 1, 1,
                   1, 1, 1]
    linkCollisionShapeIndices = [col_elbow, col_upper_leg, col_lower_leg,
                                 col_elbow, col_upper_leg, col_lower_leg,
                                 col_elbow, col_upper_leg, col_lower_leg,
                                 col_elbow, col_upper_leg, col_lower_leg]
    linkVisualShapeIndices = [vis_elbow, vis_upper_leg, vis_lower_leg,
                              vis_elbow, vis_upper_leg, vis_lower_leg,
                              vis_elbow, vis_upper_leg, vis_lower_leg,
                              vis_elbow, vis_upper_leg, vis_lower_leg]
    linkPositions = [[(0.5 - elbow_length / 2) * s, -0.3 * s, 0 * s], [elbow_length / 2 * s, 0 * s, 0 * s], [0 * s, 0 * s, -leg_length*2 * s],
                     [(0.5 - elbow_length / 2) * s, 0.3 * s, 0 * s], [elbow_length / 2 * s, 0 * s, 0 * s], [0 * s, 0 * s, -leg_length*2 * s],
                     [-(0.5 - elbow_length / 2) * s, -0.3 * s, 0 * s], [-elbow_length / 2 * s, 0 * s, 0 * s], [0 * s, 0 * s, -leg_length*2 * s],
                     [-(0.5 - elbow_length / 2) * s, 0.3 * s, 0 * s], [-elbow_length / 2 * s, 0 * s, 0 * s], [0 * s, 0 * s, -leg_length*2 * s]]
    linkOrientations = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1],
                        [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1],
                        [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1],
                        [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]]
    linkInertialFramePositions = [[0, 0, 0], [0, 0, -leg_length*2], [0, 0, -leg_length],
                                  [0, 0, 0], [0, 0, -leg_length], [0, 0, -leg_length],
                                  [0, 0, 0], [0, 0, -leg_length], [0, 0, -leg_length],
                                  [0, 0, 0], [0, 0, -leg_length], [0, 0, -leg_length]]
    linkInertialFrameOrientations = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1],
                                     [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1],
                                     [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1],
                                     [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]]
    linkParentIndices = [0, 1, 2,
                         0, 4, 5,
                         0, 7, 8,
                         0, 10, 11]
    jointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE,
                  p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE,
                  p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE,
                  p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
    axis = [[1, 0, 0], [0, 1, 0], [0, 1, 0],
            [1, 0, 0], [0, 1, 0], [0, 1, 0],
            [1, 0, 0], [0, 1, 0], [0, 1, 0],
            [1, 0, 0], [0, 1, 0], [0, 1, 0]]

    print(len(link_Masses))
    print(len(linkCollisionShapeIndices))
    print(len(linkVisualShapeIndices))
    print(len(linkPositions))
    print(len(linkOrientations))
    print(len(linkInertialFramePositions))
    print(len(linkInertialFrameOrientations))
    print(len(linkParentIndices))
    print(len(jointTypes))
    print(len(axis))

    sphereUid = p.createMultiBody(baseMass=mass,
                                  baseCollisionShapeIndex=col_body,
                                  baseVisualShapeIndex=vis_body,
                                  basePosition=basePosition,
                                  baseOrientation=baseOrientation,
                                  linkMasses=link_Masses,
                                  linkCollisionShapeIndices=linkCollisionShapeIndices,
                                  linkVisualShapeIndices=linkVisualShapeIndices,
                                  linkPositions=linkPositions,
                                  linkOrientations=linkOrientations,
                                  linkInertialFramePositions=linkInertialFramePositions,
                                  linkInertialFrameOrientations=linkInertialFrameOrientations,
                                  linkParentIndices=linkParentIndices,
                                  linkJointTypes=jointTypes,
                                  linkJointAxis=axis)

    return sphereUid
