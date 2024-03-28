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

/// \page hpp_tutorial_tutorial_2 Tutorial 2 - Plugin
///
/// In this tutorial, we are going to implement a new path planning algorithm
/// within a plugin.
///
/// \section hpp_tutorial_tutorial_2_implementation Implementing a new path planning algorithm in a plugin
///
/// The code of this tutorial can be found in \c src/tutorial_2.cc.
/// The compilation and installation instructions can be found in
/// \c src/CMakeLists.txt.
///
/// \subsection hpp_tutorial_tutorial_2_class_planner Implementation of class Planner
///
/// File \c src/tutorial_2.cc implements \c class
/// hpp::tutorial::Planner, deriving from abstract class hpp::core::PathPlanner.
/// In this section, we explain some specific parts of the code.
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
/// which is the shared pointer equivalent to \c this when using simple
/// pointers. \note Method \c init always calls the parent implementation so
/// that the parent part of the object also stores a weak pointer to itself.
///
/// \subsection hpp_tutorial_tutorial_2_plugin Implementation of plugin tutorial-2.so
///
/// Now that the new class \c hpp::tutorial::Planner has been implemented, we
/// are going to add it via a plugin.
///
/// \code
/// class Plugin : public core::ProblemSolverPlugin {
/// public:
///   Plugin() : ProblemSolverPlugin("TutorialPlugin", "0.0") {}
/// protected:
///   virtual bool impl_initialize(core::ProblemSolverPtr_t ps) {
///     ps->pathPlanners.add("TutorialPRM", Planner::create);
///     return true;
///   }
/// }; // class Plugin
/// \endcode
/// class \c hpp::tutorial::Plugin derives from abstract class
/// \c hpp::core::ProblemSolverPlugin. Upon loading of the plugin by
/// \c hppcorbaserver, method \c impl_initialize is called. This method register
/// our new path planning class with key "TutorialPRM"
///
/// \code
/// HPP_CORE_DEFINE_PLUGIN(hpp::tutorial::Plugin)
/// \endcode
/// This macro register the new plugin.
///
/// \section hpp_tutorial_tutorial_2_CMakeLists Compilation and installation
///
/// The compilation and installation is done in file \c src/CMakeLists.txt by
/// the following lines
/// \code
/// ## Tutorial 2
/// include(${HPP_CORE_CMAKE_PLUGIN})
/// # Create and install the plugin
/// hpp_add_plugin(tutorial-2 SOURCES tutorial_2.cc LINK_DEPENDENCIES
///   hpp-corbaserver::hpp-corbaserver)
/// \endcode
/// These two lines declare a new plugin the source file of which is
/// tutorial-2.cc and install this plugin into lib/hppPlugins subdirectory
/// of the installation prefix.
///
/// \section hpp_tutorial_tutorial_2_running Using the plugin and solving a problem.
///
/// To solve a problem with the new path planning
/// algorithm, we simply need to follow the same steps as in tutorial 1,
/// except that we should  source \c script/tutorial_2.py instead of
/// \c script/tutorial_1.py
///
/// \code
/// loaded = ps.client.problem.loadPlugin("tutorial-2.so")
/// assert(loaded)
///
/// ps.selectPathPlanner("TutorialPRM")
/// \endcode
/// The above lines load the plugin and select the new path planner.
///
