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

/// \page hpp_tutorial_tutorial_3 Tutorial 3
///
/// To run the tutorial, open a terminal and open 3 tabs by typing
/// \c CTRL+SHIFT+T twice. When the terminal is selected, you can select a tab
/// by typing \c ALT-[1|2|3].
///
/// \section hpp_tutorial_3_starting_gui Starting gepetto-gui
///
/// In the first tab, type
/// \code
/// gepetto-gui
/// \endcode
/// A window opens and is ready to display the scene containing the robot. The
/// robot, environment and object will appear later.
///
/// \section hpp_tutorial_3_starting_hpp_manipulation_server Starting hpp-manipulation-server
///
/// In the second tab, type
/// \code
/// hpp-manipulation-server
/// \endcode
/// See package \c hpp-manipulation-corba for details.
///
/// Note that \c gepetto-gui and \c hpp-manipulation-server executables are
/// completely independent.
///
/// \section hpp_tutorial_3_python Controlling via a python terminal
///
/// In the third tab, type
/// \code
/// python -i script tutorial_3.py
/// \endcode
///
/// This will define and solve a manipulation planning problem. After a while,
/// hpp-gui window should display a scene containing a PR2 robot in a
/// kitchen environment.
///
/// type
/// \code
/// pp (0)
/// \endocode to display the (non optimized) solution path.
