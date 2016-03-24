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

/// \mainpage
///
/// \section hpp_tutorial_intro Introduction
///
/// These tutorials explain
/// \li how to define and solve a path planning problem with HPP core and HPP manipulastion,
/// \li how to optimise a solved path,
/// \li how to implement a new path planning algorithm,
/// \li how to use gepetto viewer to visualise the problem setting.
///
/// \subsection hpp_tutorial_setting_up_environment Setting up your environment
///
/// Before starting, make sure that
/// \li \c bash is you default shell script language,
/// \li the line \code source $DEVEL_DIR/config.sh \endcode is in your .bashrc
/// file, where \c DEVEL_DIR is the environment variable defined in the
/// <a href="https://github.com/humanoid-path-planner/hpp-doc/tree/euroc-c1">
/// installation instructions</a>.
///
/// \section hpp_tutorial_tutorials Tutorials
///
/// Tutorials 1, 3 and 4 are based on an assembly scene with the Fraunhofer IPA robotic arm.
/// The objective is to reach certain contact points on the car door provided within the robot
/// package
/// (<a href="https://github.com/humanoid-path-planner/hpp-ipa/tree/master">
/// hpp-ipa</a>)
/// that should have been automatically downloaded as part of the installation
/// procedure of HPP. Tutorial 2 provides an example of implementing a custom path planning algorithm.
///
/// \li \link hpp_tutorial_tutorial_1 Tutorial 1 \endlink : how to define and
///     solve a simple path planning problem with HPP core,
/// \li \link hpp_tutorial_tutorial_2 Tutorial 2 \endlink : how to implement a new path planning algorithm in C++,
/// \li \link hpp_tutorial_tutorial_3 Tutorial 3 \endlink : how to define and
///     solve a path planning problem with HPP manipulation,
/// \li \link hpp_tutorial_tutorial_4 Tutorial 4 \endlink : how to add obstacles
///     in the above (Tutorial 3) setting.
