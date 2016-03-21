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

/// \page hpp_tutorial_script_4 Explanation about script/tutorial_4.py
///
/// This tutorial only deals with the part of <code><a href="script/tutorial_4.py">script/tutorial_4.py</a></code> that
/// differs from the previous tutorial <code><a href="script/tutorial_3.py">(script/tutorial_3.py)</a></code>.
///
/// \code{.py}
/// class Box (object):
///   rootJointType = 'anchor'
///   packageName = 'hpp_tutorial'
///   meshPackageName = 'hpp_tutorial'
///   urdfName = 'box'
///   urdfSuffix = ""
///   srdfSuffix = ""
/// \endcode
/// Create class Box in order to create objects of this kind.
///
/// \code {.py}
/// r.loadObjectModel (Box, 'box1')
/// robot.client.manipulation.robot.setRootJointPosition('box1', [0.3,1,0.9,1,0,0,0])
/// r.loadObjectModel (Box, 'box2')
/// robot.client.manipulation.robot.setRootJointPosition('box2', [0.6,1.05,1.0,1,0,0,0])
/// r.loadObjectModel (Box, 'box3')
/// robot.client.manipulation.robot.setRootJointPosition('box3', [1.1,1,0.9,1,0,0,0])
/// r.loadObjectModel (Box, 'box4')
/// robot.client.manipulation.robot.setRootJointPosition('box4', [0.9,1.05,1.2,1,0,0,0])
/// \endcode
/// Create four box objects and set their position.
///
/// The rest of tutorial 4 is identical to the previous tutorial.
/// Depending on the specifications of your computer, the computational time
/// needed to solve the problem may be significantly affected by making the scene more complicated.
