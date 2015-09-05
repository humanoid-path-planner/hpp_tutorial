from hpp import Transform
gripperInJoint = Transform (0, 0, 0.2, 0.5, 0.5, 0.5, 0.5)

j = 'dlr/schunk_wsg50_fixed_base_joint'
robot.getLinkName (j)
linkPosition = Transform (robot.getLinkPosition (j))

gripperPosition = linkPosition * gripperInJoint

handlePosition = Transform (robot.getLinkPosition ('box/base_joint_SO3'))

r.client.gui.createGroup ('gripper')
r.client.gui.addToGroup ('gripper', '0_scene_hpp_')
r.client.gui.removeFromGroup ('gripper', '0_scene_hpp_')
r.client.gui.addLandmark ('gripper', .2)
r.client.gui.applyConfiguration ('gripper', tuple (gripperPosition))
