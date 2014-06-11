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

/// \page hpp_tutorial_ros_displaying Displaying using ROS
///
/// \section hpp_tutorial_sec_rviz Starting rviz
///
/// In this page, we give a brief overview of the connection between HPP and
/// rviz using ROS. We refer the user to the official
/// <a href="http://wiki.ros.org">ROS documentation</a> for details.
///
/// The command
/// \code
/// roslaunch hpp_tutorial tutorial.launch
/// \endcode
/// starts a graphical display and several nodes that you can list by typing
/// \code
/// rosnode list
/// \endcode
/// in another terminal (you can open a new tab for that).
/// You should see
/// \code
/// /kitchen_link_broadcaster
/// /kitchen_state_publisher
/// /robot_state_publisher
/// /rosout
/// /rviz
/// \endcode
/// 
/// \section hpp_tutorial_sec_ros_nodes Ros nodes
///
/// In this section, we give a brief overview of the Ros nodes that have been
/// launched. See file
/// <code><a href="launch/tutorial.launch">launch/tutorial.launch</a></code>.
///
/// \code
/// /kitchen_state_publisher
/// \endcode
/// The obstacle is a kitchen represented as a kinematic chain as a robot. This
/// node reads the \c urdf description in ros parameter
/// \c kitchen_description and takes as input the configuration vector
/// describing the positions of translation joints (drawers) and rotation
/// joints (doors). The node computes the relative positions of the moving
/// parts of the obstacle and exports them as \c tf relative positions.
///
/// \code
/// /kitchen_link_broadcaster
/// \endcode
/// This node publishes a constant transformation between frames \c /map and
/// \c /iai_kitchen/kitchen_link. Note that \c urdf describes the robot
/// kinematic chain starting from the root link. Therefore, the position of
/// the root link in the global frame is handled separately.
///
/// \code
/// /robot_state_publisher
/// \endcode
/// The robot is a PR2 described in file
/// <code><a href="urdf/pr2.urdf">urdf/pr2.urdf</a></code> and accessible in
/// ros parameter \c robot_description. The node takes as input a vector of
/// joint values, computes the relative positions between the different links
/// and exports them as \c tf relative positions.

