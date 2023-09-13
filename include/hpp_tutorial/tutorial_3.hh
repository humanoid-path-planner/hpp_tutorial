//
// Copyright (c) 2017 CNRS
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

/// \page hpp_tutorial_tutorial_3 Tutorial 3 - manipulation
///
/// To run the tutorial, open a terminal and open 3 tabs by typing
/// \c CTRL+SHIFT+T twice. When the terminal is selected, you can select a tab
/// by typing \c ALT-[1|2|3].
///
/// \section hpp_tutorial_3_starting_hpp_manipulation_server Starting
/// hppcorbaserver
///
/// In the first tab, type
/// \code
/// hppcorbaserver
/// \endcode
/// See package \c hpp-manipulation-corba for details.
///
/// \section hpp_tutorial_3_python Controlling via a python terminal
///
/// In the second tab, type
/// \code
/// cd script
/// python -i tutorial_3.py
/// \endcode
/// Script <code><a href="script/tutorial_3.py">script/tutorial_3.py</a></code>
/// defines a manipulation planning problem.
///
/// \section hpp_tutorial_3_starting_gui Starting gepetto-gui
///
/// In the third tab, type
/// \code
/// gepetto-gui
/// \endcode
/// A window opens and is ready to display the scene containing the robot. The
/// robot, environment and object will appear later.
///
/// Note that \c gepetto-gui and \c hppcorbaserver executables are
/// completely independent.
///
/// \section hpp_tutorial_3_python Controlling via a python terminal
///
/// To display the scene, create a client to the viewer in the python terminal.
/// \code
/// >>> v = vf.createViewer ()
/// \endcode
/// The robot and environment should appear in the viewer. If the viewer
/// window is black, select the window and hit space.
///
/// To solve the problem, type
/// \code
/// >>> ps.solve ()
/// \endcode
///
/// and to display the (non optimized) solution path, type
/// \code
/// >>> pp = PathPlayer (v)
/// >>> pp (0)
/// \endcode
///
/// \section hpp_tutorial_3_optimization Optimizing the solution path
///
/// To optimize the solution path, select a path optimizer:
/// \code
/// >>> ps.addPathOptimizer('Graph-RandomShortcut')
/// >>> ps.optimizePath(0)
/// \endcode
/// To display the solution:
/// \code
/// >>> pp(1)
/// \endcode
