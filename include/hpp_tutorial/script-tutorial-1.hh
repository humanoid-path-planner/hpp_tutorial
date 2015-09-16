//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
//
// This file is part of hpp_tutorial
// hpp_tutorial is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp_tutorial is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp_tutorial  If not, see
// <http://www.gnu.org/licenses/>.

/// \page hpp_tutorial_script_1 Explanation about script/tutorial_1.py
///
/// \code{.py}
/// from hpp.corbaserver.dlr_miiwa import Robot
/// robot = Robot ('dlr')
/// robot.setJointBounds ("miiwa_joint_x", [-4, -3,])
/// robot.setJointBounds ("miiwa_joint_y", [-6.5, -3,])
/// \endcode
/// Import class \c Robot, create an instance and set bounds of
/// translation degrees of freedom of the base. \c Robot derives from python
/// class hpp.corbaserver.robot.Robot.
/// Note that the constructor of the instance calls idl method
/// hpp::corbaserver::Robot::loadRobotModel. This triggers the loading of the
/// urdf/srdf model of the dlr_miiwa robot in \c hppcorbaserver executable.
///
/// \code{.py}
/// from hpp.corbaserver import ProblemSolver
/// ps = ProblemSolver (robot)
/// \endcode
/// Import class hpp.corbaserver.problem_solver.ProblemSolver and create an
/// instance. This class is a helper class to define and solve path planning
/// problems. it implements a client to \c hppcorbaserver.
///
/// \code{.py}
/// from hpp.gepetto import Viewer
/// r = Viewer (ps)
/// \endcode
/// Import class gepetto.viewer.Viewer and create an instance.
/// This object takes as input the \c ProblemSolver instance that enables the
/// viewer client to also control \c hppcorbaserver executable in order to
/// synchronize the models in the graphical interface and in \c hppcorbaserver.
///
/// \code{.py}
/// r.loadObstacleModel ("hpp_tutorial", "box", 'box')
/// r.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")
/// \endcode
/// Load obstacles from urdf file.
/// \note This method loads the objects defined in the urdf file both in
/// \c hppcorbaserver and in \c gepetto-viewer-server.
///
/// \code{.py}
/// q_box = (-2.5, -4.0, 0.7555686333723849, 0.7071067811865475, 0., 0.7071067811865475, 0.)
/// r.moveObstacle ("box/base_link_0", q_box)
/// \endcode
/// Move box to a given configuration. The 7 numbers stand for translation and
/// unit quaternion. Again this method moves the object in the GUI and in
/// the algorithmic part.
/// \code{.py}
/// q_init = robot.getCurrentConfig ()
/// q_init [0:2] = [-3.5, -6]
/// q_init [2:4] = [0,1]
/// rank = robot.rankInConfiguration ['schunk_wsg50_joint_left_jaw']
/// q_init [rank:rank+2] = [0.05, 0.05]
/// r (q_init)
/// \endcode
/// Define and display initial configuration.
/// \note Initial configuration is built from configuration of the robot at
/// construction, and by modification of joints retrieved by name. This method
/// is more robust than specifying a hard-coded configuration vector since the
/// ordering of joints in the configuration vector is not stable.
///
/// \code{.py}
/// q_goal = (-3.3668608663093553, -3.8721030610816953, 0.4203599258539973,
///           -0.9073574448562275, 2.596600874869132, -1.4551576036136797,
///           -2.594171477219449, -1.3380271850818217, 2.1077576346524514,
///           -0.645492511677414, -3.03775218971459, 0.05, 0.05,
///           0.5157503047048388, 1.261014876256842)
/// r (q_goal)
/// \endcode
/// Define and display goal configuration.
/// \note This configuration is provided as such in this tutorial. Tutorial on
///       manipulation shows how to produce such a configuration.
///
/// \code{.py}
/// ps.setInitialConfig (q_init)
/// ps.addGoalConfig (q_goal)
/// \endcode
/// Define initial and goal configurations for the path planning problem.
///
/// \code{.py}
/// ps.selectPathValidation ("Dichotomy", 0.)
/// \endcode
/// Select the method used to check collision checking for paths. "Dichotomy"
/// is an exact method. See class documentation of
/// hpp::core::continuousCollisionChecking::Dichotomy.
/// \code{.py}
/// ps.addPathOptimizer ("RandomShortcut")
/// \endcode
/// Add a path optimizer (hpp::core::RandomShortcut).
///
/// \code{.py}
/// ps.solve ()
/// \endcode
/// Solve the problem.
///
/// \code{.py}
/// from hpp.gepetto import PathPlayer
/// pp = PathPlayer (robot.client, r)
/// \endcode
/// Import and create an instance of PathPlayer. This class samples a path in
/// \c hppcorbaserver and displays it in \c gepetto-viewer-server.
///
/// \code{.py}
/// pp (0)
/// \endcode
/// Display first path, result of RRT.
///
/// \code{.py}
/// pp (1)
/// \endcode
/// Display second path after optimization.

