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

/// \page hpp_tutorial_tutorial_1 Tutorial 1 - Python
///
/// To run the tutorial, open a terminal and open 3 tabs by typing
/// \c CTRL+SHIFT+T twice. When the terminal is selected, you can select a tab
/// by typing \c ALT-[1|2|3].
///
/// \section hpp_tutorial_1_starting_hppcorbaserver Starting hppcorbaserver
///
/// In the first tab, type
/// \code
/// hppcorbaserver
/// \endcode
/// See package \c hpp-corbaserver for details.
///
/// \section hpp_tutorial_1_starting_gui Starting gepetto-gui
///
/// In the  second tab, type
/// \code
/// gepetto-gui -c basic
/// \endcode
/// A window opens and is ready to display the scene containing the robot. The
/// robot and environment will appear later.
///
/// Note that \c gepetto-gui and \c hppcorbaserver executables are
/// completely independent.
///
/// \section hpp_tutorial_1_python Controlling via a python terminal
///
/// In the third tab, type
/// \code
/// cd script
/// python -i tutorial_1.py
/// \endcode
/// to run the script<code><a href="script/tutorial_1.py">script/tutorial_1.py</a></code> in an interactive python terminal.
///
/// To display the scene, type
/// \code
/// >>> v = vf.createViewer ()
/// \endcode
/// gepetto-gui window should now display a scene containing a PR2 robot in a
/// kitchen environment.
///
/// To display initial and goal configurations type the following commands
/// \code
/// >>> v (q_init)
/// >>> v (q_goal)
/// \endcode
///
/// To solve the path planning problem between those configurations, type
/// \code
/// >>> ps.solve ()
/// \endcode
///
/// To display the resulting of RRT, type
/// \code
/// >>> from hpp.gepetto import PathPlayer
/// >>> pp = PathPlayer (v)
/// >>> pp (0)
/// \endcode
///
/// To display an optimized solution,
/// \code
/// >>> pp (1)
/// \endcode
///
/// \section hpp_tutorial_1_script Detailed explanation
/// This section presents in more details the content of \c script/tutorial_1.py.
///
/// \code
/// from hpp.corbaserver.pr2 import Robot
/// robot = Robot ('pr2')
/// robot.setJointBounds ("root_joint", [-4, -3, -5, -3])
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
/// from hpp.gepetto import ViewerFactory
/// vf = ViewerFactory (ps)
/// \endcode
/// Import class gepetto.viewerFactory.ViewerFactory and create an instance.
/// This object takes as input the \c ProblemSolver instance that enables the
/// viewer client to also control \c hppcorbaserver executable
///
/// \code
/// q_init = robot.getCurrentConfig ()
/// q_goal = q_init [::]
/// q_init [0:2] = [-3.2, -4]
/// rank = robot.rankInConfiguration ['torso_lift_joint']
/// q_init [rank] = 0.2
/// \endcode
/// Define initial configuration.
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
/// \endcode
/// Define goal configuration.
///
/// \code
/// vf.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")
/// \endcode
/// Load obstacles from urdf file.
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
/// print (ps.solve ())
/// \endcode
/// Solve problem and print the results.
///
/// \code
/// v = vf.createViewer()
/// from hpp.gepetto import PathPlayer
/// pp = PathPlayer (v)
/// \endcode
/// Create the display window.
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

/// \page hpp_tutorial_tutorial_1_cpp Tutorial 1 - C++
///
/// Currently, there is no visualization with the C++ version.
///
/// \section hpp_tutorial_tutorial_1_cpp_source Understanding the source code.
///
/// Have a look at the file \c src/tutorial_1.cc
///
/// \section hpp_tutorial_tutorial_1_cpp_execution Execute the binary.
///
/// Compile the code and run the following command in a terminal, type
/// \code
/// build-folder/src/hpp-tutorial-1
/// \endcode
///

