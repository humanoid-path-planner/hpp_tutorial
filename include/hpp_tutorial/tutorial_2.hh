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

/// \page hpp_tutorial_tutorial_2 Tutorial 2
///
/// <h1>Implementing a new path planning algorithm</h1>
///
/// The code of this tutorial can be found in \c src/tutorial.cc.
/// The compilation and installation instructions can be found in
/// \c CMakeLists.txt.
///
/// \section hpp_tutorial_tutorial_2_class_planner Implementation of class Planner
/// File \c src/tutorial_2.cc implements \c class hpp::tutorial::Planner,
/// deriving from abstract class hpp::core::PathPlanner. In this section,
/// we explain some specific parts of the code.
///
/// \code HPP_PREDEF_CLASS (Planner);\endcode
/// is a macro containing the forward declaration of class \c Planner as well as
/// \c PlannerWkPtr_t for weak pointer to class \c Planner. See
/// \c hpp/util/pointer.hh
/// for details.
///
/// \code
///      static PlannerPtr_t create (const core::Problem& problem,
///                                  const core::RoadmapPtr_t& roadmap)
/// \endcode
/// As most classes, hpp::core::PathPlanner and any derived class are
/// manipulated
/// via shared pointers. Users are not allowed to define variables of the type.
/// Instead, they are required to call method \c create and to store the
/// resulting shared pointer. For this reason, the constructors are all
/// protected.
/// \note method \c create calls protected method \c init that is explained
///       later on.
/// 
/// \code
/// virtual void oneStep ()
/// \endcode
/// This method runs one step of our the algorithm. The new algorithm is a
/// basic version of PRM. Notice the compactness of the code.
///
/// \code
/// void init (const PlannerWkPtr_t& weak)
/// \endcode
/// Method \c init takes as input a weak pointer to a new instance and stores
/// this weak pointer as a private member. This enables any object to
/// create a shared pointer to itself on demand using the following line of code
/// \code
/// weakPtr_.lock ();
/// \endcode
/// which is the shared pointer equivalent to \c this when using simple pointers.
/// \note Method \c init always calls the parent implementation so that the
/// parent part of the object also stores a weak pointer to itself.
///
/// \section hpp_tutorial_tutorial_2_hpp_tutorial_server Implementation of executable hpp-tutorial-2-server
///
/// Now that the new class \c hpp::tutorial::Planner has been implemented, we
/// are going to use it in a new executable. The new executable is defined by
/// \code
/// int main (int argc, const char* argv[])
/// \endcode
/// This executable creates an instance of hpp::core::ProblemSolver,
/// \code
/// hpp::core::ProblemSolverPtr_t problemSolver =
///   hpp::core::ProblemSolver::create ();
/// \endcode
/// adds the constructor of class hpp::tutorial::Planner in the map of the
/// ProblemSolver instance with key "PRM",
/// \code
///  hpp::core::PathPlannerBuilder_t factory (hpp::tutorial::Planner::create);
///  problemSolver->pathPlanners.add ("PRM", factory);
/// \endcode
/// creates a CORBA server with the ProblemSolver instance,
/// \code
/// hpp::corbaServer::Server server (problemSolver, argc, argv, true);
///\endcode
/// starts the CORBA server, and
/// \code
/// server.startCorbaServer ();
/// \endcode
/// process client requests forever.
/// \code
/// server.processRequest (true);
/// \endcode
///
/// \section hpp_tutorial_tutorial_2_CMakeLists Compilation and installation
///
/// The compilation and installation is done in file \c CMakeLists.txt by
/// the following lines
/// \code
/// CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
/// INCLUDE(cmake/base.cmake)
///
/// SET(PROJECT_NAME hpp_tutorial)
/// SET(PROJECT_DESCRIPTION
///   "Tutorial for humanoid path planner platform."
/// )
/// SET(PROJECT_URL "")
///
/// SETUP_PROJECT()
/// ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
///
/// # Create and install executable running the corba server
/// ADD_EXECUTABLE (hpp-tutorial-2-server
///   src/tutorial_2.cc
/// )
/// # Link executable with hpp-corbaserver library
/// PKG_CONFIG_USE_DEPENDENCY (hpp-tutorial-2-server hpp-corbaserver)
/// # Install executable
/// INSTALL (TARGETS hpp-tutorial-2-server DESTINATION ${CMAKE_INSTALL_BINDIR})
/// SETUP_PROJECT_FINALIZE()
/// \endcode
///
/// \section hpp_tutorial_tutorial_2_running Running the server and solving a problem.
///
/// To run your executable and solve a problem with your path planning
/// algorithm, you simply need to follow the same steps as in tutorial 1,
/// except that you should start
/// \code
/// hpp-tutorial-2-server
/// \endcode
/// instead of \c hppcorbaserver and source \c script/tutorial_2.py instead of
/// \c script/tutorial_1.py, or make sure that you add the following line before
/// solving the problem
/// \code
/// ps.selectPathPlanner ("PRM")
/// \endcode
///
/// \warning Basic PRM is very inefficient. Resolution can take a long time,
/// especially if you have compiled in debug mode.
///
/// \section hpp_tutorial_tutorial_2_external_package Moving the code into an external package
///
/// To implement the above executable in an external package, you should do the
/// following steps.
/// \li create a new directory in \c src, for instance
/// \code
/// mkdir my-hpp-project
/// \endcode
/// \li create a file \c README.md describing the new package
/// \code
/// echo "Implementation of a new path planning algorithm embedded in a CORBA server" > my-hpp-project/README.md
/// \endcode
/// \li create an empty \c Doxyfile.extra.in file
/// \code
/// mkdir my-hpp-project/doc; touch my-hpp-project/doc/Doxyfile.extra.in
/// \endcode
/// \li copy the above \c cmake code into \c my-hpp-project/CMakeLists.txt, after replacing names by the names you have chosen,
/// \li copy file \c src/tutorial.cc into \c my-hpp-project/src
/// \code
/// mkdir my-hpp-project/src
/// cp hpp_tutorial/src/tutorial.cc my-hpp-project/src/.
/// \endcode
/// Go into the project directory and initialize git.
/// \code
/// cd my-hpp-project; git init; git add .
/// \endcode
/// Import the cmake git sub-module
/// \code
/// git submodule add git://github.com/jrl-umi3218/jrl-cmakemodules.git cmake
/// \endcode
/// Commit this first version.
/// \code
/// git commit -m "My first hpp project"
/// \endcode
///
/// \subsection hpp_tutorial_tutorial_2_external_package_installation Installation
/// The package is ready for installation. Create a build directory
/// \code
/// mkdir build; cd build
/// \endcode
/// configure and install
/// \code
/// cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_DIR}/install ..
/// make install
/// \endcode
///
/// Executable \c hpp-tutorial-2-server is installed and can be run from the
/// terminal.
