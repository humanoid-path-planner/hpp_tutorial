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

/// \mainpage
///
/// \section hpp_tutorial_intro Introduction
///
/// This tutorial explains how to define and solve a path planning problem
/// and how to display the resulting paths in rviz.
///
/// \subsection hpp_tutorial_setting_up_environment Setting up your environment
/// 
/// Before starting, make sure that
/// \li \c bash is you default shell script language,
/// \li the line \code source $DEVEL_DIR/config.sh \endcode is in your .bashrc
/// file, where \c DEVEL_DIR is the environment variable defined in the
/// <a href="https://github.com/humanoid-path-planner/hpp-doc/tree/ros-hydro">
/// installation instructions</a>.
///
/// To launch the tutorial, open a terminal and open 3 tabs by typing
/// \c CTRL+SHIFT+T twice. When the terminal is selected, you can select a tab
/// by typing \c ALT-[1|2|3].
///
/// \section hpp_tutorial_starting_rviz Starting rviz
///
/// In the first tab, type
/// \code
/// roslaunch hpp_tutorial tutorial.launch
/// \endcode
/// Rviz should open a new window as below.
///
/// \image html rviz-start.png
///
/// See \link hpp_tutorial_ros_displaying this page \endlink for
/// details about the nodes started by this command.
///
/// \section hpp_tutorial_starting_hppcorbaserver Starting hppcorbaserver
///
/// In the second tab, type
/// \code
/// hppcorbaserver
/// \endcode
/// See package \c hpp-corbaserver for details.
///
/// Note that ros nodes and hppcorbaserver executables are for now completely
/// independent.
///
/// \section hpp_tutorial_python Connecting hpp and ros via a python terminal
///
/// In the third tab, type
/// \code
/// python
/// \endcode
/// to open an interactive python terminal.
/// \code
/// Python 2.7.3 (default, Feb 27 2014, 20:00:17) 
/// [GCC 4.6.3] on linux2
/// Type "help", "copyright", "credits" or "license" for more information.
/// >>> 
/// \endcode
/// Copy-paste the content of file
/// <code><a href="script/tutorial.py">script/tutorial.py</a></code> in the
/// python terminal.
///
/// See \link hpp_tutorial_script this page \endlink for details about the
/// instructions of <code>script/tutorial.py</code>.
