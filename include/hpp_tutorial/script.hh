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

/// \page hpp_tutorial_script Explanation about script/tutorial_1.py
///
/// \code
/// from hpp.corbaserver.pr2 import Robot
/// robot = Robot ('pr2')
/// robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])
/// \endcode
/// Import class pr2.robot.Robot and create an instance and set bounds of
/// translation degrees of freedom of the base.
/// Note that the constructor of the instance calls idl method
/// hpp::corbaserver::Robot::loadRobotModel. This triggers the loading of the
/// urdf/srdf model of the PR2 robot in \c hppcorbaserver executable.
///
/// \code
/// from hpp.corbaserver import ProblemSolver
/// ps = ProblemSolver (robot)
/// \endcode
/// Import class hpp.corbaserver.problem_solver.ProblemSolver and create an
/// instance. This class is a helper class to define and solve path planning
/// problems.
///
/// \code
/// from hpp.gepetto import Viewer
/// r = Viewer (ps)
/// \endcode
/// Import class gepetto.viewer.Viewer and create an instance.
/// This object takes as input the \c ProblemSolver instance that enables the
/// viewer client to also control \c hppcorbaserver executable
///
/// \code
/// q_init = robot.getCurrentConfig ()
/// q_goal = q_init [::]
/// q_init [0:2] = [-3.2, -4]
/// rank = robot.rankInConfiguration ['torso_lift_joint']
/// q_init [rank] = 0.2
/// r (q_init)
/// \endcode
/// Define and display initial configuration.
/// \note Initial configuration is built from configuration of the robot at
/// construction, and by modification of joints retrieved by name. This method
/// is more robust than specifying a hard-coded configuration vector since the
/// ordering of joints in the configuration vector is not unique.
///
/// \code
/// q_goal [0:2] = [-3.2, -4]
/// rank = robot.rankInConfiguration ['l_shoulder_lift_joint']
/// q_goal [rank] = 0.5
/// rank = robot.rankInConfiguration ['l_elbow_flex_joint']
/// q_goal [rank] = -0.5
/// rank = robot.rankInConfiguration ['r_shoulder_lift_joint']
/// q_goal [rank] = 0.5
/// rank = robot.rankInConfiguration ['r_elbow_flex_joint']
/// q_goal [rank] = -0.5
/// r (q_goal)
/// \endcode
/// Define and display goal configuration.
///
/// \code
/// r.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")
/// \endcode
/// Load obstacle from urdf file.
/// \note this method loads the objects defined in the urdf file both in
/// hppcorbaserver and in \c gepetto-viewer-server.
///
/// \code
/// ps.setInitialConfig (q_init)
/// ps.addGoalConfig (q_goal)
/// \endcode
/// Define initial and goal configurations.
///
/// \code
/// ps.addPathOptimizer ("RandomShortcut")
/// \endcode
/// Add a path optimizer (hpp::core::RandomShortcut).
///
/// \code
/// ps.solve ()
/// \endcode
/// Solve problem.
///
/// \code
/// from hpp.gepetto import PathPlayer
/// pp = PathPlayer (robot.client, r)
/// \endcode
/// Import and create an instance of PathPlayer. This class samples a path in
/// \c hppcorbaserver and displays it in \c gepetto-viewer-server.
///
/// \code
/// pp (0)
/// \endcode
/// Display first path, result of RRT.
///
/// \code
/// pp (1)
/// \endcode
/// Display second path after optimization.

