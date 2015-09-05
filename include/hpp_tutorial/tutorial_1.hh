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

/// \page hpp_tutorial_tutorial_1 Tutorial 1
///
/// To run the tutorial, open a terminal and open 3 tabs by typing
/// \c CTRL+SHIFT+T twice. When the terminal is selected, you can select a tab
/// by typing \c ALT-[1|2|3].
///
/// \section hpp_tutorial_starting_gui Starting gepetto-viewer-server
///
/// In the first tab, type
/// \code
/// gepetto-viewer-server
/// \endcode
/// Nothing apparently happens, but the graphical user interface is ready to
/// create a new window and to display a scene.
///
/// \section hpp_tutorial_starting_hppcorbaserver Starting hppcorbaserver
///
/// In the second tab, type
/// \code
/// hppcorbaserver
/// \endcode
/// See package \c hpp-corbaserver for details.
///
/// Note that \c gepetto-viewer-server and \c hppcorbaserver executables are
/// completely independent.
///
/// \section hpp_tutorial_python Controlling via a python terminal
///
/// In the third tab, type
/// \code
/// python
/// \endcode
/// to open an interactive python terminal.
/// \code
/// Python 2.7.6 (default, Jun 22 2015, 17:58:13) 
/// [GCC 4.8.2] on linux2
/// Type "help", "copyright", "credits" or "license" for more information.
/// >>> 
/// \endcode
/// Copy-paste the content of file
/// <code><a href="script/tutorial_1.py">script/tutorial_1.py</a></code> in the
/// python terminal.
///
/// A window should pop-up, displaying a scene containing the DLR-miiwa robot
/// in a kitchen environment.
///
/// See \link hpp_tutorial_script this page \endlink for details about the
/// instructions of <code>script/tutorial_1.py</code>.
