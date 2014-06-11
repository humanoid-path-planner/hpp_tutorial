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

/// \page hpp_tutorial_script Explanation of script <code>script/tutorial.py
/// </code>
///
/// \code
/// from hpp.corbaserver.pr2 import Robot
/// robot = Robot ('pr2')
/// \endcode
/// Import class pr2.robot.Robot and create an instance.
/// Note that the constructor of the instance calls idl method
/// hpp::corbaserver::Robot::loadRobotModel. This triggers the loading of the
/// urdf/srdf model of the PR2 robot in \c hppcorbaserver executable.
///
/// \code
/// from hpp_ros import ScenePublisher
/// r = ScenePublisher (robot)
/// \endcode
/// Import class hpp_ros.scene_publisher.ScenePublisher and create an instance.
/// This object takes as input a list of joint values and broadcasts it
/// for display in rviz.
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
/// q_init = [-3.2, -4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.15, 0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.15, 0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
/// r (q_init)
/// \endcode
/// Define and display in rviz initial configuration.
///
/// \code
/// q_goal = [-3.2, -4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.5, 0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
/// r (q_goal)
/// \endcode
/// Define and display in rviz goal configuration.
///
/// \code
/// ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area")
/// \endcode
/// Load obstacle from urdf file. Note that you need to load the same file as
/// defined in the launch file. Again, \c hppcorbaserver executable and rviz
/// are completely independent and need to be synchronized by python.
///
/// \code
/// ps.setInitialConfig (q_init)
/// ps.addGoalConfig (q_goal)
/// \endcode
/// Define initial and goal configurations.
///
/// \code
/// ps.solve ()
/// \endcode
/// Solve problem.
///
/// \code
/// from hpp_ros import PathPlayer
/// pp = PathPlayer (robot.client, r)
/// \endcode
/// Import and create an instance of PathPlayer. This class samples a path in
/// \c hppcorbaserver and displays it in rviz.
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

