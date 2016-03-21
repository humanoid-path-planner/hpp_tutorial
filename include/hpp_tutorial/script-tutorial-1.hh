//
// Copyright (c) 2016 CNRS
// Authors: Anna Seppala
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
/// from hpp.corbaserver.hpp_ipa import Robot
/// robot = Robot ('ipa')
/// \endcode
/// Import class \c Robot and create an instance.
/// \c Robot derives from python class hpp.corbaserver.robot.Robot.
/// Note that the constructor of the instance calls idl method
/// hpp::corbaserver::Robot::loadRobotModel. This triggers the loading of the
/// urdf/srdf model of the Frauenhofer ipa robot in \c hppcorbaserver executable.
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
/// synchronise the models in the graphical interface and in \c hppcorbaserver.
///
/// \code{.py}
/// r.loadObstacleModel ("hpp-ipa", 'door', 'Door')
/// \endcode
/// Load an obstacle from urdf file.
/// \note This method loads the objects defined in the urdf file both in
/// \c hppcorbaserver and in \c gepetto-viewer-server.
///
/// \code{.py}
/// q_door = (0.5,1.6,0.8,1,0,0,0)
/// ps.getObstacleNames(True, False)
/// r.moveObstacle ("Door/door_frame_0", q_door)
/// \endcode
/// Move door to a given configuration. The 7 numbers stand for translation and
/// unit quaternion. Again this method moves the object in the GUI and in
/// the algorithmic part.
/// In order to find the name of the object to be moved, the function getObstacleNames
/// may be called. The two boolean parameters determine which types of objects are searched
/// (collision parameters or distance parameters).
///
/// \code{.py}
/// q_init = robot.getCurrentConfig ()
/// r (q_init)
/// \endcode
/// Define and display initial configuration.
///
/// \code{.py}
/// q_goal = [0.934, -3.266, -1.212,-0.823, -1.488, 0.269]
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
/// ps.client.problem.getAvailable('PathValidation')
/// ps.selectPathValidation ("Dichotomy", 0.)
/// \endcode
/// Select the method used to check collision checking for paths. "Dichotomy"
/// is an exact method. See class documentation of
/// hpp::core::continuousCollisionChecking::Dichotomy.
/// The function getAvailable shows all available methods for path validation.
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
/// ps.client.problem.getAvailable('PathOptimizer')
/// ps.addPathOptimizer ("RandomShortcut")
/// \endcode
/// Add a path optimiser (hpp::core::RandomShortcut). Again,
/// the available optimiser methods may be listed using the
/// function getAvailable.
///
/// \code{.py}
/// ps.numberPaths()
/// ps.optimizePath(0)
/// ps.numberPaths()
/// \endcode
/// Show initial number of solved paths, optimise, and show new number of paths.
/// Optimising a path automatically adds a new path into the path vector.
///
/// \code{.py}
/// pp (1)
/// \endcode
/// Display second path after optimisation.
///
/// If a path optimiser is added before solving, the path is automatically optimised.
/// Just as before, both the unoptimised and the optimised path are added to the path vector.

