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

/// \page hpp_tutorial_tutorial_3 Tutorial 3
///
/// This tutorial explains how to define and solve a manipulation planning
/// problem. In manipulation planning, a so-called constraint graph is used.
///
/// \section starting_the_script Starting the software
///
/// To start the software, follow the same steps as in \link
/// hpp_tutorial_tutorial_1 tutorial 1\endlink replacing \c hppcorbaserver
/// with \c hpp-manipulation-server.
///
/// \note Only by launching \c hpp-manipulation-server, will the required functionality
/// be enabled.
///
/// \section understanding_the_software Understanding the script
///
/// Copy-paste the content of file
/// <code><a href="script/tutorial_3.py">script/tutorial_3.py</a></code> in the
/// python terminal.
///
/// See \link hpp_tutorial_script_3 this page \endlink for details about the
/// instructions of <code>script/tutorial_3.py</code>.
///
/// \section monitoring_the_constraint_graph Monitoring the constraint graph
///
/// In another terminal, type hpp-plot-manipulation-graph, a window appears.
///
/// Click on "Refresh" and then on "Statistics" to display the manipulation
/// graph. Clicking on nodes and edges shows the success rate of the projection
/// function.
