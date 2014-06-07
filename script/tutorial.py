from hpp.corbaserver.pr2 import Robot
from hpp_ros import ScenePublisher

robot = Robot ('pr2')
r = ScenePublisher (robot)
q = robot.getCurrentConfig ()
r (q)

res = robot.distancesToCollision ()
x = zip (*res)
y = filter (lambda t:t [0] <= 0.01, x)
y = filter (lambda t:t[1].find ('caster')!=-1 or t[2].find('caster')!=-1, x)

with file ("./srdf/collision", "w") as f:
    for l in y:
        f.write ('  <disable_collisions link1="%s" link2="%s"/>\n'%l[1:3])
