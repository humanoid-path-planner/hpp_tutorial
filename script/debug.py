from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2', False)

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from gepetto.corbaserver import Client as GuiClient
guiClient = GuiClient ()

from hpp.gepetto import Viewer
Viewer.sceneName = '0_scene_hpp_'
r = Viewer (ps, guiClient)

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, r)

